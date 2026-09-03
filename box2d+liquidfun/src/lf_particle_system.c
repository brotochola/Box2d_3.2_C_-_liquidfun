// liquidfun-c - src/lf_particle_system.c
// SPDX-License-Identifier: Zlib
//
// Original implementation. Re-derives the general weakly-compressible,
// grid-accelerated particle approach popularized by LiquidFun, but the code
// itself is written from scratch against Box2D 3's public C API - it does
// not contain or translate any LiquidFun source.

#include "liquidfun/lf_particle_system.h"

#include "box2d/constants.h"
#include "physics_world.h"
#include "scheduler.h"

#include <float.h>
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// SSE2 is required (Emscripten -msimd128 -msse2, see wasm/CMakeLists.txt,
// translated to real wasm128 instructions - same technique Box2D 3's own
// contact_solver.c uses for B2_SIMD_SSE2 on B2_CPU_WASM). Fail the build
// loudly instead of silently going scalar if that flag is ever missing.
#if defined( _MSC_VER ) && !defined( __clang__ )
#include <intrin.h>
#if !defined( __SSE2__ )
#define __SSE2__ 1
#endif
#endif
#if !defined( __SSE2__ )
#error "liquidfun-c requires SSE2 (-msimd128 -msse2). Check wasm/CMakeLists.txt compile flags."
#endif
#include <emmintrin.h>

// Google LiquidFun 1.1.0 constants (b2_particleStride, b2_minParticleWeight, …).
#define LF_PARTICLE_STRIDE 0.75f
#define LF_MIN_PARTICLE_WEIGHT 1.0f
#define LF_MAX_PARTICLE_PRESSURE 0.25f
#define LF_MAX_PARTICLE_FORCE 0.5f
#define LF_NO_PRESSURE_FLAGS ( lf_powderParticle | lf_tensileParticle )
// Google b2_barrierCollisionTime. tmax = this * dt.
#define LF_BARRIER_COLLISION_TIME 2.5f
#define LF_PAIR_CAPTURE_FLAGS ( lf_springParticle | lf_barrierParticle )

// ------------------------------------------------------------------------
// Shared Box2D task system (same pthreads as b2World_Step)
// ------------------------------------------------------------------------

typedef void lfParallelForCallback( int startIndex, int endIndex, int workerIndex, void* context );

static b2EnqueueTaskCallback* g_lfEnqueue = NULL;
static b2FinishTaskCallback* g_lfFinish = NULL;
static void* g_lfTaskUser = NULL;
static void ( *g_lfResetTasks )( void* userContext ) = NULL;
static int g_lfWorkerCount = 1;

void lfSetTaskSystem( b2EnqueueTaskCallback* enqueue, b2FinishTaskCallback* finish, void* userContext, int workerCount,
					  void ( *resetTasks )( void* userContext ) )
{
	g_lfEnqueue = enqueue;
	g_lfFinish = finish;
	g_lfTaskUser = userContext;
	g_lfResetTasks = resetTasks;
	if ( workerCount < 1 )
	{
		workerCount = 1;
	}
	if ( workerCount > B2_MAX_WORKERS )
	{
		workerCount = B2_MAX_WORKERS;
	}
	g_lfWorkerCount = workerCount;
}

int lfGetWorkerCount( void )
{
	return g_lfWorkerCount;
}

static void lfResetBox2dScheduler( void* user )
{
	if ( user != NULL )
	{
		b2ResetScheduler( (b2Scheduler*)user );
	}
}

void lfBindBox2dWorld( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL )
	{
		lfSetTaskSystem( NULL, NULL, NULL, 1, NULL );
		return;
	}
	lfSetTaskSystem( world->enqueueTaskFcn, world->finishTaskFcn, world->userTaskContext, world->workerCount,
					 world->scheduler != NULL ? lfResetBox2dScheduler : NULL );
}

typedef struct lfParallelForShared
{
	atomic_int nextBlock;
	int blockCount;
	int blockSize;
	int itemCount;
	lfParallelForCallback* callback;
	void* context;
} lfParallelForShared;

typedef struct lfParallelForTask
{
	lfParallelForShared* shared;
	int workerIndex;
} lfParallelForTask;

static void lfParallelForTrampoline( void* taskContext )
{
	lfParallelForTask* task = (lfParallelForTask*)taskContext;
	lfParallelForShared* shared = task->shared;
	const int workerIndex = task->workerIndex;
	const int blockCount = shared->blockCount;
	const int blockSize = shared->blockSize;
	const int itemCount = shared->itemCount;
	lfParallelForCallback* callback = shared->callback;
	void* context = shared->context;

	for ( ;; )
	{
		int blockIndex = atomic_fetch_add( &shared->nextBlock, 1 );
		if ( blockIndex >= blockCount )
		{
			break;
		}
		int start = blockIndex * blockSize;
		int end = start + blockSize;
		if ( end > itemCount )
		{
			end = itemCount;
		}
		callback( start, end, workerIndex, context );
	}
}

static void lfParallelFor( lfParallelForCallback* callback, int itemCount, int minRange, void* context )
{
	if ( itemCount <= 0 )
	{
		return;
	}
	if ( minRange < 1 )
	{
		minRange = 1;
	}
	const int workerCount = g_lfWorkerCount;
	if ( workerCount <= 1 || g_lfEnqueue == NULL || g_lfFinish == NULL )
	{
		callback( 0, itemCount, 0, context );
		return;
	}

	int blocksPerWorker = 4;
	int maxBlockCount = blocksPerWorker * workerCount;
	int blockSize;
	int blockCount;
	if ( itemCount <= minRange * maxBlockCount )
	{
		blockSize = minRange;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	else
	{
		blockSize = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	if ( blockCount < 1 )
	{
		blockCount = 1;
	}

	int taskCount = workerCount < blockCount ? workerCount : blockCount;

	lfParallelForShared shared;
	shared.blockCount = blockCount;
	shared.blockSize = blockSize;
	shared.itemCount = itemCount;
	shared.callback = callback;
	shared.context = context;
	atomic_store( &shared.nextBlock, 0 );

	lfParallelForTask tasks[B2_MAX_WORKERS];
	void* handles[B2_MAX_WORKERS];
	for ( int i = 0; i < taskCount; ++i )
	{
		tasks[i].shared = &shared;
		tasks[i].workerIndex = i;
		handles[i] = g_lfEnqueue( &lfParallelForTrampoline, &tasks[i], g_lfTaskUser );
	}
	for ( int i = 0; i < taskCount; ++i )
	{
		if ( handles[i] != NULL )
		{
			g_lfFinish( handles[i], g_lfTaskUser );
		}
	}
}

// ------------------------------------------------------------------------
// Internal types
// ------------------------------------------------------------------------

typedef struct lfParticleContact
{
	uint16_t a, b; // a < b, indices into the particle buffers
	b2Vec2 normal; // unit vector pointing from particle a to particle b
	float weight;  // 1 - distance / diameter, in (0, 1]
} lfParticleContact;

typedef struct lfBodyContact
{
	uint16_t index;	  // particle index
	b2BodyId bodyId;  // rigid body in contact
	b2ShapeId shapeId;
	b2Vec2 normal;	  // Google: -ComputeDistance n (particle toward body)
	float weight;	  // 1 - signedDist / diameter (can be > 1 when inside)
	float invMassA;	  // particle inverse mass (0 for wall particles)
	float mass;		  // reduced mass 1/invMassSum
} lfBodyContact;

typedef struct lfParticlePair
{
	uint16_t a, b;
	uint32_t flags; // SPRING and/or BARRIER stamped at capture
	float distance;
	float strength;
} lfParticlePair;

typedef struct lfParticleGroup
{
	uint32_t flags;		 // particle flags stamped at create (elastic/spring/…)
	uint32_t groupFlags; // lfParticleGroupFlag (solid/rigid/…)
	float strength;
	float viscousScale;
	float mass;
	float invMass;
	float invInertia;
	b2Vec2 center;
	b2Vec2 linearVelocity;
	float angularVelocity;
	float angle;
	int firstIndex; // inclusive
	int lastIndex;	// exclusive; count = lastIndex - firstIndex
	int count;
	bool alive;
	float accDot;
	float accCross;
	float accSpin;
	float accR2;
	float accI;
} lfParticleGroup;

struct lfParticleSystem
{
	b2WorldId worldId;
	lfParticleSystemDef def;

	float diameter;
	float invDiameter;
	float restSpacing; // incompressible target separation (stride)
	float particleMass;
	float particleInvMass;

	int count;
	int capacity;
	float* posX;
	float* posY;
	float* velX;
	float* velY;
	float* weight; // per-particle accumulated contact weight (density proxy)
	uint32_t* flags;
	int* groupIndex;
	b2Vec2* restOffset;
	b2Vec2* force; // accumulated ApplyForce; drained in SolveForce
	bool hasForce;
	float* depth;  // solid particle depth (Google m_depthBuffer)
	// Age-based destruction (lfParticleGroupDef.lifetimeMin/Max). totalLife
	// == 0 means "not tracked" - the common case, untouched by SolveLifetime
	// past its one comparison. totalLife > 0 means tracked + fade-to-0;
	// totalLife < 0 means tracked, opaque until destroy (|totalLife| is the
	// life). remainingLife is always the positive countdown. renderAlpha is
	// exposed to WASM/JS for the optional fade visual.
	float* remainingLife;
	float* totalLife;
	float* renderAlpha;
	// Per-particle multiplier on def.viscousStrength (default 1). Stamped at
	// create; SetGroupViscousScale rewrites members. SolveViscous reads this.
	float* viscousScale;

	lfParticleGroup* groups;
	int groupCount;
	int groupCapacity;

	// Spatial hash grid, rebuilt every sub-step. cellSize == diameter, so a
	// 3x3 neighborhood around a particle's cell is always enough to find
	// every other particle within `diameter` of it.
	int hashSize; // power of two, sized from live count (not capacity)
	uint16_t* cellHead;
	uint16_t* next; // parallel to position/velocity/weight/flags, size == capacity
	// Cached GetCell(position[i]) from the last BuildGrid this sub-step - pure
	// scratch, fully overwritten every BuildGrid call (no need to keep in sync
	// across SolveZombie's order-preserving compact). Avoids recomputing floorf/mul for
	// the same particle in FindParticleContacts / ForEachParticleNearShape /
	// SolveBarrier's inner grid walk.
	int* cellX;
	int* cellY;
	uint32_t flagOr;  // OR of all particle flags; skip unused solvers
	uint32_t allGroupFlags; // OR of alive groupFlags
	bool hasShapeGroups; // any elastic/spring group alive
	float* staticPressure;		// Poisson field; 0 if particle lacks the flag
	float* staticPressureAccum; // scratch: Poisson then pressure h[]
	b2Vec2* accumulation2;		// tensile weighted normals, size == capacity
	float* accumulation;		// solid depth scratch, size == capacity

	lfParticleContact* particleContacts;
	int particleContactCount;
	int particleContactCapacity;
	// Per-worker scratch for parallel FindParticleContacts. Merge is serial.
	lfParticleContact* contactBucket[B2_MAX_WORKERS];
	int contactBucketCount[B2_MAX_WORKERS];
	int contactBucketCap[B2_MAX_WORKERS];
	// SolveStaticPressure scratch: indices into particleContacts that qualify
	// (either endpoint flagged lf_staticPressureParticle), compacted once per
	// sub-step instead of re-filtering the full list every Poisson iteration.
	// Sized/grown in lockstep with particleContactCapacity.
	int* staticPressureContactIndices;

	lfBodyContact* bodyContacts;
	int bodyContactCount;
	int bodyContactCapacity;

	// Rest-length springs / barrier segments captured at create.
	lfParticlePair* pairs;
	int pairCount;
	int pairCapacity;

	b2ShapeId* queryShapes;
	int queryShapeCount;
	int queryShapeCapacity;
};

// WASM particle step is single-thread; SolveCollision callback reads these.
static float s_collisionDt;
static float s_collisionInvDt;

// ------------------------------------------------------------------------
// Small helpers
// ------------------------------------------------------------------------

static int NextPow2( int x )
{
	int p = 1;
	while ( p < x )
	{
		p <<= 1;
	}
	return p;
}

static uint32_t HashCell( int ix, int iy, int hashSize )
{
	uint32_t h = ( (uint32_t)ix * 92837111u ) ^ ( (uint32_t)iy * 689287499u );
	return h & (uint32_t)( hashSize - 1 );
}

static void GetCell( const lfParticleSystem* sys, b2Vec2 p, int* ix, int* iy )
{
	*ix = (int)floorf( p.x * sys->invDiameter );
	*iy = (int)floorf( p.y * sys->invDiameter );
}

static b2Vec2 RotateOffset( float c, float s, b2Vec2 q )
{
	return ( b2Vec2 ){ c * q.x - s * q.y, s * q.x + c * q.y };
}

static bool GroupIsValid( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return groupId >= 0 && groupId < sys->groupCount && sys->groups[groupId].alive;
}

// Forward decl - defined with the rest of the spatial hash grid machinery
// further down; CapturePairs runs at group-create time, not sub-step time,
// but reuses the same grid buffers (see comment below).
static void EnsureHashForCount( lfParticleSystem* sys );

static void CapturePairs( lfParticleSystem* sys, int start, int n )
{
	float maxDist = sys->diameter * 1.5f;
	float maxDistSqr = maxDist * maxDist;

	// Grid-accelerated instead of all-pairs O(n^2): pre-existing particles are
	// never pair candidates here (only [start, start+n) pairs with itself), so
	// build a scratch grid over just the new range, reusing the same
	// cellHead/next/cellX/cellY buffers the per-step BuildGrid uses. Safe to
	// clobber: this only runs at group-create time (WASM exports drain
	// commands before b2World_Step/lfParticleSystem_Step each frame), and
	// nothing reads grid state until the next real BuildGrid rebuilds it in
	// full anyway. maxDist here (1.5x diameter) exceeds one cell (cellSize ==
	// diameter), so the neighborhood must be 5x5 (+/-2), not
	// FindParticleContacts's 3x3 (which relies on searchRadius <= cellSize).
	EnsureHashForCount( sys );
	if ( sys->cellHead == NULL || sys->hashSize < 1 )
	{
		return;
	}
	memset( sys->cellHead, 0xFF, (size_t)sys->hashSize * sizeof( uint16_t ) );
	for ( int i = start; i < start + n; i++ )
	{
		int ix, iy;
		GetCell( sys, ( (b2Vec2){ sys->posX[i], sys->posY[i] } ), &ix, &iy );
		sys->cellX[i] = ix;
		sys->cellY[i] = iy;
		uint32_t cell = HashCell( ix, iy, sys->hashSize );
		sys->next[i] = sys->cellHead[cell];
		sys->cellHead[cell] = (uint16_t)i;
	}

	for ( int i = start; i < start + n; i++ )
	{
		int ix = sys->cellX[i];
		int iy = sys->cellY[i];
		for ( int dy = -2; dy <= 2; dy++ )
		{
			for ( int dx = -2; dx <= 2; dx++ )
			{
				uint32_t cell = HashCell( ix + dx, iy + dy, sys->hashSize );
				for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
				{
					if ( sys->cellX[j] != ix + dx || sys->cellY[j] != iy + dy )
					{
						continue; // hash collision from a different cell
					}
					if ( (int)j <= i )
					{
						continue; // each unordered pair is only added once, from the lower index
					}
					b2Vec2 delta = b2Sub( ( (b2Vec2){ sys->posX[j], sys->posY[j] } ), ( (b2Vec2){ sys->posX[i], sys->posY[i] } ) );
					float distSqr = b2LengthSquared( delta );
					if ( distSqr < maxDistSqr && distSqr > 1e-9f )
					{
						if ( sys->pairCount == sys->pairCapacity )
						{
							sys->pairCapacity = sys->pairCapacity > 0 ? sys->pairCapacity * 2 : 256;
							sys->pairs = (lfParticlePair*)realloc( sys->pairs, (size_t)sys->pairCapacity * sizeof( lfParticlePair ) );
						}
						lfParticlePair* p = &sys->pairs[sys->pairCount++];
						p->a = (uint16_t)i;
						p->b = (uint16_t)j;
						p->flags = ( sys->flags[i] | sys->flags[j] ) & LF_PAIR_CAPTURE_FLAGS;
						p->distance = sqrtf( distSqr );
						p->strength = sys->def.springStrength;
					}
				}
			}
		}
	}
}

static void MaybeCapturePairs( lfParticleSystem* sys, int start, int n, uint32_t flags )
{
	if ( n > 1 && ( flags & LF_PAIR_CAPTURE_FLAGS ) != 0 )
	{
		CapturePairs( sys, start, n );
	}
}


// ------------------------------------------------------------------------
// Definitions
// ------------------------------------------------------------------------

lfParticleSystemDef lfDefaultParticleSystemDef( void )
{
	lfParticleSystemDef def;
	def.radius = 0.05f;
	def.density = 1.0f;
	def.dampingStrength = 1.0f;
	def.pressureStrength = 0.05f;
	def.viscousStrength = 0.25f;
	def.tensileStrength = 0.2f;
	def.powderStrength = 0.5f;
	def.springStrength = 0.25f;
	def.staticPressureStrength = 0.2f;
	def.staticPressureRelaxation = 0.2f;
	def.staticPressureIterations = 8;
	def.ejectionStrength = 0.5f;
	def.maxParticles = 2048;
	def.growable = true;
	def.strictContactCheck = false;
	return def;
}

lfParticleDef lfDefaultParticleDef( void )
{
	lfParticleDef def;
	def.flags = lf_waterParticle;
	def.position = ( b2Vec2 ){ 0.0f, 0.0f };
	def.velocity = ( b2Vec2 ){ 0.0f, 0.0f };
	return def;
}

lfParticleGroupDef lfDefaultParticleGroupDef( void )
{
	lfParticleGroupDef def;
	def.flags = lf_elasticParticle;
	def.groupFlags = 0;
	def.box = ( b2AABB ){ { 0.0f, 0.0f }, { 0.0f, 0.0f } };
	def.position = ( b2Vec2 ){ 0.0f, 0.0f };
	def.radius = 0.5f;
	def.spacing = 0.0f;
	def.linearVelocity = ( b2Vec2 ){ 0.0f, 0.0f };
	def.angularVelocity = 0.0f;
	def.strength = 0.5f;
	def.viscousScale = 1.0f;
	def.trackGroup = 0;
	def.lifetimeMin = 0.0f;
	def.lifetimeMax = 0.0f; // <= 0 => no age-based destruction (default)
	def.fadeToAlpha0 = 0;	// opaque until destroy unless caller opts in
	return def;
}

// ------------------------------------------------------------------------
// Create / destroy / capacity management
// ------------------------------------------------------------------------

static bool EnsureCapacity( lfParticleSystem* sys, int minCapacity )
{
	if ( minCapacity <= sys->capacity )
	{
		return true;
	}

	if ( !sys->def.growable )
	{
		return false;
	}

	int newCapacity = sys->capacity > 0 ? sys->capacity : 64;
	while ( newCapacity < minCapacity )
	{
		newCapacity *= 2;
	}

	sys->posX = (float*)realloc( sys->posX, (size_t)newCapacity * sizeof( float ) );
	sys->posY = (float*)realloc( sys->posY, (size_t)newCapacity * sizeof( float ) );
	sys->velX = (float*)realloc( sys->velX, (size_t)newCapacity * sizeof( float ) );
	sys->velY = (float*)realloc( sys->velY, (size_t)newCapacity * sizeof( float ) );
	sys->weight = (float*)realloc( sys->weight, (size_t)newCapacity * sizeof( float ) );
	sys->flags = (uint32_t*)realloc( sys->flags, (size_t)newCapacity * sizeof( uint32_t ) );
	sys->next = (uint16_t*)realloc( sys->next, (size_t)newCapacity * sizeof( uint16_t ) );
	sys->cellX = (int*)realloc( sys->cellX, (size_t)newCapacity * sizeof( int ) );
	sys->cellY = (int*)realloc( sys->cellY, (size_t)newCapacity * sizeof( int ) );
	sys->groupIndex = (int*)realloc( sys->groupIndex, (size_t)newCapacity * sizeof( int ) );
	sys->restOffset = (b2Vec2*)realloc( sys->restOffset, (size_t)newCapacity * sizeof( b2Vec2 ) );
	sys->force = (b2Vec2*)realloc( sys->force, (size_t)newCapacity * sizeof( b2Vec2 ) );
	sys->depth = (float*)realloc( sys->depth, (size_t)newCapacity * sizeof( float ) );
	sys->accumulation = (float*)realloc( sys->accumulation, (size_t)newCapacity * sizeof( float ) );
	sys->remainingLife = (float*)realloc( sys->remainingLife, (size_t)newCapacity * sizeof( float ) );
	sys->totalLife = (float*)realloc( sys->totalLife, (size_t)newCapacity * sizeof( float ) );
	sys->renderAlpha = (float*)realloc( sys->renderAlpha, (size_t)newCapacity * sizeof( float ) );
	sys->viscousScale = (float*)realloc( sys->viscousScale, (size_t)newCapacity * sizeof( float ) );
	sys->staticPressure = (float*)realloc( sys->staticPressure, (size_t)newCapacity * sizeof( float ) );
	sys->staticPressureAccum = (float*)realloc( sys->staticPressureAccum, (size_t)newCapacity * sizeof( float ) );
	sys->accumulation2 = (b2Vec2*)realloc( sys->accumulation2, (size_t)newCapacity * sizeof( b2Vec2 ) );

	sys->capacity = newCapacity;
	return true;
}

static bool EnsureGroupCapacity( lfParticleSystem* sys, int minCapacity )
{
	if ( minCapacity <= sys->groupCapacity )
	{
		return true;
	}

	int newCapacity = sys->groupCapacity > 0 ? sys->groupCapacity : 8;
	while ( newCapacity < minCapacity )
	{
		newCapacity *= 2;
	}

	sys->groups = (lfParticleGroup*)realloc( sys->groups, (size_t)newCapacity * sizeof( lfParticleGroup ) );
	sys->groupCapacity = newCapacity;
	return sys->groups != NULL;
}

lfParticleSystem* lfParticleSystem_Create( b2WorldId worldId, const lfParticleSystemDef* def )
{
	lfParticleSystem* sys = (lfParticleSystem*)calloc( 1, sizeof( lfParticleSystem ) );
	sys->worldId = worldId;
	sys->def = def != NULL ? *def : lfDefaultParticleSystemDef();

	if ( sys->def.maxParticles < 1 )
	{
		free( sys );
		return NULL;
	}
	if ( sys->def.maxParticles > LF_MAX_PARTICLES )
	{
		sys->def.maxParticles = LF_MAX_PARTICLES;
	}

	sys->diameter = 2.0f * sys->def.radius;
	sys->invDiameter = 1.0f / sys->diameter;
	sys->restSpacing = LF_PARTICLE_STRIDE * sys->diameter;
	sys->flagOr = 0;
	sys->allGroupFlags = 0;
	sys->hasShapeGroups = false;
	sys->hasForce = false;
	// Mass of a particle modeled as if it packed a `diameter x diameter`
	// square of fluid at rest - a common, simple convention for grid/SPH
	// particle mass that keeps pressure math independent of pi.
	sys->particleMass = sys->def.density * sys->diameter * sys->diameter;
	sys->particleInvMass = sys->particleMass > 0.0f ? 1.0f / sys->particleMass : 0.0f;

	int hint = sys->def.maxParticles > 0 ? sys->def.maxParticles : 64;
	if ( !sys->def.growable )
	{
		// Pin every per-particle buffer now. realloc later would invalidate
		// WASM/SAB TypedArray views onto these pointers.
		sys->posX = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->posY = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->velX = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->velY = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->weight = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->flags = (uint32_t*)malloc( (size_t)hint * sizeof( uint32_t ) );
		sys->next = (uint16_t*)malloc( (size_t)hint * sizeof( uint16_t ) );
		sys->cellX = (int*)malloc( (size_t)hint * sizeof( int ) );
		sys->cellY = (int*)malloc( (size_t)hint * sizeof( int ) );
		sys->groupIndex = (int*)malloc( (size_t)hint * sizeof( int ) );
		sys->restOffset = (b2Vec2*)malloc( (size_t)hint * sizeof( b2Vec2 ) );
		sys->force = (b2Vec2*)calloc( (size_t)hint, sizeof( b2Vec2 ) );
		sys->depth = (float*)calloc( (size_t)hint, sizeof( float ) );
		sys->accumulation = (float*)calloc( (size_t)hint, sizeof( float ) );
		sys->remainingLife = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->totalLife = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->renderAlpha = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->viscousScale = (float*)malloc( (size_t)hint * sizeof( float ) );
		sys->staticPressure = (float*)calloc( (size_t)hint, sizeof( float ) );
		sys->staticPressureAccum = (float*)calloc( (size_t)hint, sizeof( float ) );
		sys->accumulation2 = (b2Vec2*)calloc( (size_t)hint, sizeof( b2Vec2 ) );
		sys->capacity = hint;
		sys->hashSize = 64;
		sys->cellHead = (uint16_t*)malloc( (size_t)sys->hashSize * sizeof( uint16_t ) );
	}
	else
	{
		EnsureCapacity( sys, hint );
	}

	int contactHint = hint * 8 > 2048 ? hint * 8 : 2048;
	sys->particleContactCapacity = contactHint;
	sys->particleContacts = (lfParticleContact*)malloc( (size_t)sys->particleContactCapacity * sizeof( lfParticleContact ) );
	sys->staticPressureContactIndices = (int*)malloc( (size_t)sys->particleContactCapacity * sizeof( int ) );

	int bodyContactHint = hint * 2 > 512 ? hint * 2 : 512;
	sys->bodyContactCapacity = bodyContactHint;
	sys->bodyContacts = (lfBodyContact*)malloc( (size_t)sys->bodyContactCapacity * sizeof( lfBodyContact ) );

	sys->queryShapeCapacity = 128;
	sys->queryShapes = (b2ShapeId*)malloc( (size_t)sys->queryShapeCapacity * sizeof( b2ShapeId ) );

	sys->pairCapacity = hint * 4 > 1024 ? hint * 4 : 1024;
	sys->pairs = (lfParticlePair*)malloc( (size_t)sys->pairCapacity * sizeof( lfParticlePair ) );


	return sys;
}

void lfParticleSystem_Destroy( lfParticleSystem* sys )
{
	if ( sys == NULL )
	{
		return;
	}
	free( sys->posX );
	free( sys->posY );
	free( sys->velX );
	free( sys->velY );
	free( sys->weight );
	free( sys->flags );
	free( sys->next );
	free( sys->cellX );
	free( sys->cellY );
	free( sys->groupIndex );
	free( sys->restOffset );
	free( sys->force );
	free( sys->depth );
	free( sys->accumulation );
	free( sys->remainingLife );
	free( sys->totalLife );
	free( sys->renderAlpha );
	free( sys->viscousScale );
	free( sys->staticPressure );
	free( sys->staticPressureAccum );
	free( sys->accumulation2 );
	free( sys->groups );
	free( sys->cellHead );
	free( sys->particleContacts );
	free( sys->staticPressureContactIndices );
	free( sys->bodyContacts );
	for ( int w = 0; w < B2_MAX_WORKERS; w++ )
	{
		free( sys->contactBucket[w] );
	}
	free( sys->queryShapes );
	free( sys->pairs );
	free( sys );
}

// ------------------------------------------------------------------------
// Particle creation / destruction
// ------------------------------------------------------------------------

int lfParticleSystem_CreateParticle( lfParticleSystem* sys, const lfParticleDef* def )
{
	if ( sys->count >= sys->capacity )
	{
		if ( !EnsureCapacity( sys, sys->count + 1 ) )
		{
			return -1;
		}
	}

	int i = sys->count++;
	sys->posX[i] = def->position.x;
	sys->posY[i] = def->position.y;
	sys->velX[i] = def->velocity.x;
	sys->velY[i] = def->velocity.y;
	sys->weight[i] = 0.0f;
	sys->flags[i] = def->flags;
	sys->flagOr |= def->flags;
	sys->groupIndex[i] = LF_NULL_PARTICLE_GROUP;
	sys->restOffset[i] = ( b2Vec2 ){ 0.0f, 0.0f };
	sys->force[i] = ( b2Vec2 ){ 0.0f, 0.0f };
	sys->depth[i] = 0.0f;
	sys->staticPressure[i] = 0.0f;
	// Not tracked by default (totalLife == 0) - callers that want age-based
	// destruction (CreateParticleGroupBox/Circle with lifetimeMax > 0)
	// overwrite these for their range right after this call.
	sys->remainingLife[i] = 0.0f;
	sys->totalLife[i] = 0.0f;
	sys->renderAlpha[i] = 1.0f;
	sys->viscousScale[i] = 1.0f;
	return i;
}

void lfParticleSystem_DestroyParticle( lfParticleSystem* sys, int index )
{
	if ( index < 0 || index >= sys->count )
	{
		return;
	}
	sys->flags[index] |= lf_zombieParticle;
}

void lfParticleSystem_ClearParticles( lfParticleSystem* sys )
{
	if ( sys == NULL )
	{
		return;
	}
	for ( int i = 0; i < sys->groupCount; i++ )
	{
		sys->groups[i].alive = false;
		sys->groups[i].count = 0;
		sys->groups[i].firstIndex = 0;
		sys->groups[i].lastIndex = 0;
	}
	sys->groupCount = 0;
	sys->pairCount = 0;
	sys->particleContactCount = 0;
	sys->bodyContactCount = 0;
	sys->count = 0;
	sys->flagOr = 0;
	sys->allGroupFlags = 0;
	sys->hasShapeGroups = false;
}

static float DefaultParticleStride( const lfParticleSystem* sys )
{
	return sys->restSpacing;
}

int lfParticleSystem_CreateParticleBox( lfParticleSystem* sys, b2AABB box, float spacing, uint32_t flags,
										b2Vec2 initialVelocity )
{
	if ( spacing <= 0.0f )
	{
		spacing = DefaultParticleStride( sys );
	}

	int start = sys->count;
	int created = 0;
	for ( float y = box.lowerBound.y; y <= box.upperBound.y + 1e-4f; y += spacing )
	{
		for ( float x = box.lowerBound.x; x <= box.upperBound.x + 1e-4f; x += spacing )
		{
			lfParticleDef def = lfDefaultParticleDef();
			def.flags = flags;
			def.position = ( b2Vec2 ){ x, y };
			def.velocity = initialVelocity;
			if ( lfParticleSystem_CreateParticle( sys, &def ) < 0 )
			{
				MaybeCapturePairs( sys, start, created, flags );
				return created;
			}
			created++;
		}
	}
	MaybeCapturePairs( sys, start, created, flags );
	return created;
}

// ------------------------------------------------------------------------
// Groups
// ------------------------------------------------------------------------

static int AllocGroup( lfParticleSystem* sys )
{
	for ( int i = 0; i < sys->groupCount; i++ )
	{
		if ( !sys->groups[i].alive )
		{
			return i;
		}
	}
	if ( !EnsureGroupCapacity( sys, sys->groupCount + 1 ) )
	{
		return -1;
	}
	return sys->groupCount++;
}

// Rest pose + COM/mass/inertia from current positions. Does not touch velocity.
static void RebuildGroupRestFromPositions( lfParticleSystem* sys, lfParticleGroup* g )
{
	int start = g->firstIndex;
	int end = g->lastIndex;
	int n = end - start;
	if ( n <= 0 )
	{
		g->center = ( b2Vec2 ){ 0.0f, 0.0f };
		g->mass = 0.0f;
		g->invMass = 0.0f;
		g->invInertia = 0.0f;
		g->angle = 0.0f;
		return;
	}

	b2Vec2 com = { 0.0f, 0.0f };
	for ( int i = start; i < end; i++ )
	{
		com = b2Add( com, ( (b2Vec2){ sys->posX[i], sys->posY[i] } ) );
	}
	com = b2MulSV( 1.0f / (float)n, com );

	g->center = com;
	g->mass = sys->particleMass * (float)n;
	g->invMass = g->mass > 0.0f ? 1.0f / g->mass : 0.0f;
	g->angle = 0.0f;

	float inertia = 0.0f;
	for ( int i = start; i < end; i++ )
	{
		b2Vec2 offset = b2Sub( ( (b2Vec2){ sys->posX[i], sys->posY[i] } ), com );
		sys->restOffset[i] = offset;
		inertia += sys->particleMass * b2LengthSquared( offset );
	}
	g->invInertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
}

static void InitGroupFromRange( lfParticleSystem* sys, int gid, int start, int n, const lfParticleGroupDef* def )
{
	lfParticleGroup* g = &sys->groups[gid];
	memset( g, 0, sizeof( *g ) );
	g->alive = true;
	g->flags = def->flags;
	g->groupFlags = def->groupFlags;
	g->strength = def->strength;
	g->viscousScale = def->viscousScale > 0.0f ? def->viscousScale : 1.0f;
	g->firstIndex = start;
	g->lastIndex = start + n;
	g->count = n;
	g->linearVelocity = def->linearVelocity;
	g->angularVelocity = def->angularVelocity;

	if ( n <= 0 )
	{
		g->invMass = 0.0f;
		return;
	}

	for ( int i = start; i < start + n; i++ )
	{
		sys->groupIndex[i] = gid;
		sys->flags[i] |= def->flags;
	}

	RebuildGroupRestFromPositions( sys, g );

	for ( int i = start; i < start + n; i++ )
	{
		b2Vec2 offset = sys->restOffset[i];
		b2Vec2 spin = b2MulSV( def->angularVelocity, b2LeftPerp( offset ) );
		{ b2Vec2 __v = b2Add( def->linearVelocity, spin ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
	}

	if ( ( g->groupFlags & lf_solidParticleGroup ) != 0 )
	{
		g->groupFlags |= lf_particleGroupNeedsUpdateDepth;
	}
	sys->allGroupFlags |= g->groupFlags;
}

// Google's testbed generates the random lifetime in caller code and passes a
// concrete value to SetParticleLifetime per particle - straightforward there
// since it creates particles one at a time in a loop. This engine creates
// particles in groups (one call fills an entire box/circle), so the random
// pick happens here instead, once per particle in the just-created range -
// same outcome (every particle in the group independently randomized),
// adapted to how this port's creation API is actually shaped.
static void SeedLifespan( lfParticleSystem* sys, int start, int n, float lifetimeMin, float lifetimeMax,
						  int fadeToAlpha0 )
{
	// wasm/CMakeLists.txt builds this whole module with -ffast-math
	// (-ffinite-math-only), which licenses the compiler to assume NaN never
	// occurs and fold comparisons accordingly - so this must NOT rely on a
	// NaN-safe comparison trick to reject a bad value; callers (JS call
	// sites across the WASM boundary in particular - an omitted arg there
	// coerces to NaN, not 0) must always pass a real, finite number.
	if ( lifetimeMax <= 0.0f )
	{
		return; // not requested - particles keep CreateParticle's untracked default
	}
	if ( lifetimeMin < 0.0f )
	{
		lifetimeMin = 0.0f;
	}
	if ( lifetimeMax < lifetimeMin )
	{
		lifetimeMax = lifetimeMin;
	}
	float range = lifetimeMax - lifetimeMin;
	for ( int i = start; i < start + n; i++ )
	{
		float life = lifetimeMin + ( range > 0.0f ? range * ( (float)rand() / (float)RAND_MAX ) : 0.0f );
		if ( life <= 0.0f )
		{
			life = 1e-3f; // avoid an instant-zombie particle on the same step it's created
		}
		// Sign of totalLife encodes fade: +life = fade, -life = opaque until destroy.
		sys->totalLife[i] = fadeToAlpha0 ? life : -life;
		sys->remainingLife[i] = life;
		sys->renderAlpha[i] = 1.0f;
	}
}

static void StampViscousScaleRange( lfParticleSystem* sys, int start, int n, float scale )
{
	float s = scale > 0.0f ? scale : 1.0f;
	for ( int i = start; i < start + n; i++ )
	{
		sys->viscousScale[i] = s;
	}
}

static int ShouldKeepGroup( const lfParticleGroupDef* def )
{
	if ( ( def->flags & ( lf_elasticParticle | lf_springParticle ) ) != 0 )
	{
		return 1;
	}
	if ( ( def->groupFlags & ( lf_solidParticleGroup | lf_rigidParticleGroup ) ) != 0 )
	{
		return 1;
	}
	if ( def->trackGroup )
	{
		return 1;
	}
	if ( def->viscousScale != 1.0f )
	{
		return 1;
	}
	return 0;
}

static int IsShapeGroupFlags( uint32_t flags )
{
	return ( flags & ( lf_elasticParticle | lf_springParticle ) ) != 0;
}

lfParticleGroupId lfParticleSystem_CreateParticleGroupBox( lfParticleSystem* sys, const lfParticleGroupDef* def )
{
	lfParticleGroupDef local = def != NULL ? *def : lfDefaultParticleGroupDef();
	int start = sys->count;
	float spacing = local.spacing > 0.0f ? local.spacing : DefaultParticleStride( sys );
	int n = lfParticleSystem_CreateParticleBox( sys, local.box, spacing, local.flags, local.linearVelocity );
	if ( n <= 0 )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	SeedLifespan( sys, start, n, local.lifetimeMin, local.lifetimeMax, local.fadeToAlpha0 );
	StampViscousScaleRange( sys, start, n, local.viscousScale );
	if ( !ShouldKeepGroup( &local ) )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	int gid = AllocGroup( sys );
	if ( gid < 0 )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	InitGroupFromRange( sys, gid, start, n, &local );
	if ( IsShapeGroupFlags( local.flags ) )
	{
		sys->hasShapeGroups = true;
	}
	return gid;
}

lfParticleGroupId lfParticleSystem_CreateParticleGroupCircle( lfParticleSystem* sys, const lfParticleGroupDef* def )
{
	lfParticleGroupDef local = def != NULL ? *def : lfDefaultParticleGroupDef();
	float spacing = local.spacing > 0.0f ? local.spacing : DefaultParticleStride( sys );
	float r = local.radius;
	int start = sys->count;
	int n = 0;
	float rSqr = r * r;

	for ( float y = -r; y <= r + 1e-4f; y += spacing )
	{
		for ( float x = -r; x <= r + 1e-4f; x += spacing )
		{
			if ( x * x + y * y > rSqr )
			{
				continue;
			}
			lfParticleDef pdef = lfDefaultParticleDef();
			pdef.flags = local.flags;
			pdef.position = ( b2Vec2 ){ local.position.x + x, local.position.y + y };
			pdef.velocity = local.linearVelocity;
			if ( lfParticleSystem_CreateParticle( sys, &pdef ) < 0 )
			{
				break;
			}
			n++;
		}
	}

	if ( n <= 0 )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	SeedLifespan( sys, start, n, local.lifetimeMin, local.lifetimeMax, local.fadeToAlpha0 );
	MaybeCapturePairs( sys, start, n, local.flags );
	StampViscousScaleRange( sys, start, n, local.viscousScale );
	if ( !ShouldKeepGroup( &local ) )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	int gid = AllocGroup( sys );
	if ( gid < 0 )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	InitGroupFromRange( sys, gid, start, n, &local );
	if ( IsShapeGroupFlags( local.flags ) )
	{
		sys->hasShapeGroups = true;
	}
	return gid;
}

void lfParticleSystem_DestroyParticleGroup( lfParticleSystem* sys, lfParticleGroupId groupId )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	lfParticleGroup* g = &sys->groups[groupId];
	for ( int i = g->firstIndex; i < g->lastIndex; i++ )
	{
		sys->flags[i] |= lf_zombieParticle;
		sys->groupIndex[i] = LF_NULL_PARTICLE_GROUP;
	}
	g->alive = false;
	g->count = 0;
	g->firstIndex = 0;
	g->lastIndex = 0;
}

int lfParticleSystem_GetGroupParticleCount( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].count : 0;
}

int lfParticleSystem_GetGroupFirstIndex( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].firstIndex : 0;
}

int lfParticleSystem_GetGroupLastIndex( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].lastIndex : 0;
}

uint32_t lfParticleSystem_GetGroupFlags( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? ( sys->groups[groupId].groupFlags & ~lf_particleGroupInternalMask ) : 0;
}

void lfParticleSystem_SetGroupFlags( lfParticleSystem* sys, lfParticleGroupId groupId, uint32_t groupFlags )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	lfParticleGroup* g = &sys->groups[groupId];
	uint32_t oldFlags = g->groupFlags;
	uint32_t newFlags = groupFlags;
	if ( ( ( oldFlags ^ newFlags ) & lf_solidParticleGroup ) != 0 )
	{
		newFlags |= lf_particleGroupNeedsUpdateDepth;
	}
	g->groupFlags = newFlags;
	sys->allGroupFlags |= newFlags;
}

b2Vec2 lfParticleSystem_GetGroupCenter( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].center : ( b2Vec2 ){ 0.0f, 0.0f };
}

b2Vec2 lfParticleSystem_GetGroupLinearVelocity( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].linearVelocity : ( b2Vec2 ){ 0.0f, 0.0f };
}

float lfParticleSystem_GetGroupAngularVelocity( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].angularVelocity : 0.0f;
}

float lfParticleSystem_GetGroupAngle( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].angle : 0.0f;
}

float lfParticleSystem_GetGroupViscousScale( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? sys->groups[groupId].viscousScale : 1.0f;
}

int lfParticleSystem_GetGroupSlotCount( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->groupCount : 0;
}

int lfParticleSystem_IsGroupAlive( const lfParticleSystem* sys, lfParticleGroupId groupId )
{
	return GroupIsValid( sys, groupId ) ? 1 : 0;
}

void lfParticleSystem_SetGroupViscousScale( lfParticleSystem* sys, lfParticleGroupId groupId, float scale )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	float s = scale > 0.0f ? scale : 1.0f;
	sys->groups[groupId].viscousScale = s;
	lfParticleGroup* g = &sys->groups[groupId];
	for ( int i = g->firstIndex; i < g->lastIndex; i++ )
	{
		sys->viscousScale[i] = s;
	}
}

void lfParticleSystem_SetTuning( lfParticleSystem* sys, float dampingStrength, float pressureStrength,
								 float viscousStrength, float tensileStrength, float powderStrength,
								 float springStrength, float staticPressureStrength,
								 float staticPressureRelaxation, int staticPressureIterations )
{
	if ( sys == NULL )
	{
		return;
	}
	sys->def.dampingStrength = dampingStrength;
	sys->def.pressureStrength = pressureStrength;
	sys->def.viscousStrength = viscousStrength;
	sys->def.tensileStrength = tensileStrength;
	sys->def.powderStrength = powderStrength;
	sys->def.springStrength = springStrength;
	sys->def.staticPressureStrength = staticPressureStrength;
	sys->def.staticPressureRelaxation = staticPressureRelaxation;
	sys->def.staticPressureIterations = staticPressureIterations < 1 ? 1 : staticPressureIterations;
}

static void UpdateGroupStatistics( lfParticleSystem* sys )
{
	if ( sys->groupCount == 0 )
	{
		return;
	}

	sys->allGroupFlags = 0;
	sys->hasShapeGroups = false;

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive )
		{
			continue;
		}
		sys->allGroupFlags |= group->groupFlags;
		if ( IsShapeGroupFlags( group->flags ) )
		{
			sys->hasShapeGroups = true;
		}
		group->center = ( b2Vec2 ){ 0.0f, 0.0f };
		group->linearVelocity = ( b2Vec2 ){ 0.0f, 0.0f };
		group->count = group->lastIndex - group->firstIndex;
		if ( group->count <= 0 )
		{
			continue;
		}
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			group->center = b2Add( group->center, ( (b2Vec2){ sys->posX[i], sys->posY[i] } ) );
			group->linearVelocity = b2Add( group->linearVelocity, ( (b2Vec2){ sys->velX[i], sys->velY[i] } ) );
		}
		float inv = 1.0f / (float)group->count;
		group->center = b2MulSV( inv, group->center );
		group->linearVelocity = b2MulSV( inv, group->linearVelocity );
		group->mass = sys->particleMass * (float)group->count;
		group->invMass = group->mass > 0.0f ? 1.0f / group->mass : 0.0f;

		group->accDot = 0.0f;
		group->accCross = 0.0f;
		group->accSpin = 0.0f;
		group->accR2 = 0.0f;
		group->accI = 0.0f;
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			b2Vec2 p = b2Sub( ( (b2Vec2){ sys->posX[i], sys->posY[i] } ), group->center );
			b2Vec2 q = sys->restOffset[i];
			group->accDot += p.x * q.x + p.y * q.y;
			group->accCross += p.y * q.x - p.x * q.y;

			b2Vec2 relV = b2Sub( ( (b2Vec2){ sys->velX[i], sys->velY[i] } ), group->linearVelocity );
			group->accSpin += p.x * relV.y - p.y * relV.x;
			group->accR2 += b2LengthSquared( p );
			group->accI += sys->particleMass * b2LengthSquared( q );
		}
		group->angle = atan2f( group->accCross, group->accDot );
		group->angularVelocity = group->accR2 > 1e-12f ? group->accSpin / group->accR2 : 0.0f;
		group->invInertia = group->accI > 0.0f ? 1.0f / group->accI : 0.0f;
	}
}

static void SolveElastic( lfParticleSystem* sys, float dt )
{
	if ( dt <= 0.0f || sys->groupCount == 0 )
	{
		return;
	}

	float invDt = 1.0f / dt;
	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive || ( group->flags & lf_elasticParticle ) == 0 )
		{
			continue;
		}
		float c = cosf( group->angle );
		float s = sinf( group->angle );
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			if ( ( sys->flags[i] & lf_elasticParticle ) == 0 || ( sys->flags[i] & lf_wallParticle ) != 0 )
			{
				continue;
			}
			b2Vec2 target = b2Add( group->center, RotateOffset( c, s, sys->restOffset[i] ) );
			b2Vec2 delta = b2Sub( target, ( (b2Vec2){ sys->posX[i], sys->posY[i] } ) );
			{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[i], sys->velY[i] } ), b2MulSV( group->strength * invDt, delta ) ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
		}
	}
}

static void SolveSpring( lfParticleSystem* sys, float dt )
{
	if ( sys->pairCount == 0 || dt <= 0.0f )
	{
		return;
	}

	for ( int k = 0; k < sys->pairCount; k++ )
	{
		lfParticlePair* p = &sys->pairs[k];
		int a = p->a;
		int b = p->b;

		if ( a < 0 || a >= sys->count || b < 0 || b >= sys->count )
		{
			continue;
		}
		if ( ( p->flags & lf_springParticle ) == 0 )
		{
			continue;
		}

		b2Vec2 delta = b2Sub( ( (b2Vec2){ sys->posX[b], sys->posY[b] } ), ( (b2Vec2){ sys->posX[a], sys->posY[a] } ) );
		float distSqr = b2LengthSquared( delta );
		if ( distSqr < 1e-9f )
		{
			continue;
		}

		float dist = sqrtf( distSqr );
		b2Vec2 normal = b2MulSV( 1.0f / dist, delta );

		float deltaDist = dist - p->distance;
		float impulseMag = p->strength * deltaDist * dt * 0.5f;
		b2Vec2 impulse = b2MulSV( impulseMag, normal );

		if ( ( sys->flags[a] & lf_wallParticle ) == 0 )
		{
			{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[a], sys->velY[a] } ), impulse ); sys->velX[a] = __v.x; sys->velY[a] = __v.y; }
		}
		if ( ( sys->flags[b] & lf_wallParticle ) == 0 )
		{
			{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[b], sys->velY[b] } ), impulse ); sys->velX[b] = __v.x; sys->velY[b] = __v.y; }
		}
	}
}


// ------------------------------------------------------------------------
// Age-based destruction (Google LiquidFun's SetParticleDestructionByAge /
// SetParticleLifetime design - see lfParticleGroupDef.lifetimeMin/Max).
// Runs once per Step with the full dt (not subDt - a linear countdown has
// no sub-step accuracy to gain), right before SolveZombie so any particle
// that expires this step is swept up by that same batch compaction.
// ------------------------------------------------------------------------

static void SolveLifetime( lfParticleSystem* sys, float dt )
{
	for ( int i = 0; i < sys->count; i++ )
	{
		float total = sys->totalLife[i];
		if ( total == 0.0f )
		{
			continue; // not tracked
		}
		sys->remainingLife[i] -= dt;
		if ( sys->remainingLife[i] <= 0.0f )
		{
			sys->flags[i] |= lf_zombieParticle;
			// Fade particles go transparent on expiry; opaque-until-destroy stay at 1
			// (particle is about to be swept anyway).
			sys->renderAlpha[i] = total > 0.0f ? 0.0f : 1.0f;
			continue;
		}
		if ( total > 0.0f )
		{
			sys->renderAlpha[i] = sys->remainingLife[i] / total;
		}
		// else: fadeToAlpha0 was off - leave renderAlpha at 1.0
	}
}

// ------------------------------------------------------------------------
// Particle buffer rotate + Google-style order-preserving zombie compact
// ------------------------------------------------------------------------

static void CopyParticle( lfParticleSystem* sys, int dst, int src )
{
	if ( dst == src )
	{
		return;
	}
	sys->posX[dst] = sys->posX[src];
	sys->posY[dst] = sys->posY[src];
	sys->velX[dst] = sys->velX[src];
	sys->velY[dst] = sys->velY[src];
	sys->weight[dst] = sys->weight[src];
	sys->flags[dst] = sys->flags[src];
	sys->groupIndex[dst] = sys->groupIndex[src];
	sys->restOffset[dst] = sys->restOffset[src];
	sys->force[dst] = sys->force[src];
	sys->depth[dst] = sys->depth[src];
	sys->staticPressure[dst] = sys->staticPressure[src];
	sys->remainingLife[dst] = sys->remainingLife[src];
	sys->totalLife[dst] = sys->totalLife[src];
	sys->renderAlpha[dst] = sys->renderAlpha[src];
	sys->viscousScale[dst] = sys->viscousScale[src];
}

static int RotateIndex( int i, int start, int mid, int end )
{
	if ( i < start )
	{
		return i;
	}
	if ( i < mid )
	{
		return i + end - mid;
	}
	if ( i < end )
	{
		return i + start - mid;
	}
	return i;
}

static void RotateTyped( void* base, size_t elemSize, int start, int mid, int end )
{
	// std::rotate via temp+[memmove]: O(n) with big memcpy, not per-byte reverses.
	int nLeft = mid - start;
	int nRight = end - mid;
	if ( nLeft <= 0 || nRight <= 0 )
	{
		return;
	}
	unsigned char* p = (unsigned char*)base + (size_t)start * elemSize;
	size_t leftBytes = (size_t)nLeft * elemSize;
	size_t rightBytes = (size_t)nRight * elemSize;
	unsigned char* tmp = (unsigned char*)malloc( leftBytes );
	if ( tmp == NULL )
	{
		// Fallback: element swaps (still far cheaper than byte-wise reverse).
		unsigned char scratch[64];
		unsigned char* one = scratch;
		int heapOne = 0;
		if ( elemSize > sizeof( scratch ) )
		{
			one = (unsigned char*)malloc( elemSize );
			heapOne = 1;
			if ( one == NULL )
			{
				return;
			}
		}
		for ( int a = 0, b = nLeft - 1; a < b; a++, b-- )
		{
			memcpy( one, p + (size_t)a * elemSize, elemSize );
			memcpy( p + (size_t)a * elemSize, p + (size_t)b * elemSize, elemSize );
			memcpy( p + (size_t)b * elemSize, one, elemSize );
		}
		for ( int a = nLeft, b = nLeft + nRight - 1; a < b; a++, b-- )
		{
			memcpy( one, p + (size_t)a * elemSize, elemSize );
			memcpy( p + (size_t)a * elemSize, p + (size_t)b * elemSize, elemSize );
			memcpy( p + (size_t)b * elemSize, one, elemSize );
		}
		for ( int a = 0, b = nLeft + nRight - 1; a < b; a++, b-- )
		{
			memcpy( one, p + (size_t)a * elemSize, elemSize );
			memcpy( p + (size_t)a * elemSize, p + (size_t)b * elemSize, elemSize );
			memcpy( p + (size_t)b * elemSize, one, elemSize );
		}
		if ( heapOne )
		{
			free( one );
		}
		return;
	}
	memcpy( tmp, p, leftBytes );
	memmove( p, p + leftBytes, rightBytes );
	memcpy( p + rightBytes, tmp, leftBytes );
	free( tmp );
}

static void RemapPairs( lfParticleSystem* sys, const int* newIndices, int count )
{
	int w = 0;
	for ( int k = 0; k < sys->pairCount; k++ )
	{
		lfParticlePair p = sys->pairs[k];
		int a = p.a;
		int b = p.b;
		if ( a < 0 || a >= count || b < 0 || b >= count )
		{
			continue;
		}
		int na = newIndices[a];
		int nb = newIndices[b];
		if ( na < 0 || nb < 0 || na == nb )
		{
			continue;
		}
		p.a = (uint16_t)na;
		p.b = (uint16_t)nb;
		sys->pairs[w++] = p;
	}
	sys->pairCount = w;
}

// Google RotateBuffer: cycle [start,end) so mid becomes the new start of the
// rotated span. Updates pair indices and every group's first/last.
static void RotateBuffer( lfParticleSystem* sys, int start, int mid, int end )
{
	if ( start == mid || mid == end )
	{
		return;
	}
	RotateTyped( sys->posX, sizeof( float ), start, mid, end );
	RotateTyped( sys->posY, sizeof( float ), start, mid, end );
	RotateTyped( sys->velX, sizeof( float ), start, mid, end );
	RotateTyped( sys->velY, sizeof( float ), start, mid, end );
	RotateTyped( sys->weight, sizeof( float ), start, mid, end );
	RotateTyped( sys->flags, sizeof( uint32_t ), start, mid, end );
	RotateTyped( sys->groupIndex, sizeof( int ), start, mid, end );
	RotateTyped( sys->restOffset, sizeof( b2Vec2 ), start, mid, end );
	RotateTyped( sys->force, sizeof( b2Vec2 ), start, mid, end );
	RotateTyped( sys->depth, sizeof( float ), start, mid, end );
	RotateTyped( sys->staticPressure, sizeof( float ), start, mid, end );
	RotateTyped( sys->remainingLife, sizeof( float ), start, mid, end );
	RotateTyped( sys->totalLife, sizeof( float ), start, mid, end );
	RotateTyped( sys->renderAlpha, sizeof( float ), start, mid, end );
	RotateTyped( sys->viscousScale, sizeof( float ), start, mid, end );

	for ( int k = 0; k < sys->pairCount; k++ )
	{
		sys->pairs[k].a = (uint16_t)RotateIndex( sys->pairs[k].a, start, mid, end );
		sys->pairs[k].b = (uint16_t)RotateIndex( sys->pairs[k].b, start, mid, end );
	}

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive || group->lastIndex <= group->firstIndex )
		{
			continue;
		}
		group->firstIndex = RotateIndex( group->firstIndex, start, mid, end );
		group->lastIndex = RotateIndex( group->lastIndex - 1, start, mid, end ) + 1;
		group->count = group->lastIndex - group->firstIndex;
	}
}

static void SolveZombie( lfParticleSystem* sys )
{
	int count = sys->count;
	if ( count <= 0 )
	{
		return;
	}

	int* newIndices = (int*)malloc( (size_t)count * sizeof( int ) );
	if ( newIndices == NULL )
	{
		return;
	}

	int newCount = 0;
	uint32_t allParticleFlags = 0;
	for ( int i = 0; i < count; i++ )
	{
		uint32_t f = sys->flags[i];
		if ( ( f & lf_zombieParticle ) != 0 )
		{
			newIndices[i] = -1;
		}
		else
		{
			newIndices[i] = newCount;
			if ( i != newCount )
			{
				CopyParticle( sys, newCount, i );
			}
			newCount++;
			allParticleFlags |= f;
		}
	}

	RemapPairs( sys, newIndices, count );

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive )
		{
			continue;
		}
		int firstIndex = newCount;
		int lastIndex = 0;
		int modified = 0;
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			int j = ( i >= 0 && i < count ) ? newIndices[i] : -1;
			if ( j >= 0 )
			{
				if ( j < firstIndex )
				{
					firstIndex = j;
				}
				if ( j + 1 > lastIndex )
				{
					lastIndex = j + 1;
				}
			}
			else
			{
				modified = 1;
			}
		}
		if ( firstIndex < lastIndex )
		{
			group->firstIndex = firstIndex;
			group->lastIndex = lastIndex;
			group->count = lastIndex - firstIndex;
			if ( modified )
			{
				RebuildGroupRestFromPositions( sys, group );
				if ( ( group->groupFlags & lf_solidParticleGroup ) != 0 )
				{
					group->groupFlags |= lf_particleGroupNeedsUpdateDepth;
				}
			}
		}
		else
		{
			group->firstIndex = 0;
			group->lastIndex = 0;
			group->count = 0;
			if ( ( group->groupFlags & lf_particleGroupCanBeEmpty ) == 0 )
			{
				group->groupFlags |= lf_particleGroupWillBeDestroyed;
			}
		}
	}

	sys->count = newCount;
	sys->flagOr = allParticleFlags;
	free( newIndices );

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		if ( sys->groups[g].alive && ( sys->groups[g].groupFlags & lf_particleGroupWillBeDestroyed ) != 0 )
		{
			sys->groups[g].alive = false;
			sys->groups[g].groupFlags &= ~lf_particleGroupWillBeDestroyed;
		}
	}
}

// ------------------------------------------------------------------------
// Spatial hash grid + particle/particle contact detection
// ------------------------------------------------------------------------

static void EnsureHashForCount( lfParticleSystem* sys )
{
	int n = sys->count * 2;
	if ( n < 64 )
	{
		n = 64;
	}
	int desired = NextPow2( n );
	if ( desired == sys->hashSize && sys->cellHead != NULL )
	{
		return;
	}
	uint16_t* nextHead = (uint16_t*)realloc( sys->cellHead, (size_t)desired * sizeof( uint16_t ) );
	if ( nextHead == NULL )
	{
		return;
	}
	sys->cellHead = nextHead;
	sys->hashSize = desired;
}

static void BuildGrid( lfParticleSystem* sys )
{
	EnsureHashForCount( sys );
	if ( sys->cellHead == NULL || sys->hashSize < 1 )
	{
		return;
	}
	memset( sys->cellHead, 0xFF, (size_t)sys->hashSize * sizeof( uint16_t ) );
	for ( int i = 0; i < sys->count; i++ )
	{
		int ix, iy;
		GetCell( sys, ( (b2Vec2){ sys->posX[i], sys->posY[i] } ), &ix, &iy );
		sys->cellX[i] = ix;
		sys->cellY[i] = iy;
		uint32_t cell = HashCell( ix, iy, sys->hashSize );
		sys->next[i] = sys->cellHead[cell];
		sys->cellHead[cell] = (uint16_t)i;
	}
}

#define LF_CONTACT_PARALLEL_MIN 4096
#define LF_CONTACT_MIN_RANGE 256

static float lfInvSqrt( float x )
{
	float inv = _mm_cvtss_f32( _mm_rsqrt_ss( _mm_set_ss( x ) ) );
	return inv * ( 1.5f - 0.5f * x * inv * inv );
}

static void GrowParticleContactCap( lfParticleSystem* sys, int need )
{
	if ( need <= sys->particleContactCapacity )
	{
		return;
	}
	int cap = sys->particleContactCapacity < 1024 ? 1024 : sys->particleContactCapacity;
	while ( cap < need )
	{
		cap *= 2;
	}
	sys->particleContacts =
		(lfParticleContact*)realloc( sys->particleContacts, (size_t)cap * sizeof( lfParticleContact ) );
	sys->staticPressureContactIndices = (int*)realloc( sys->staticPressureContactIndices, (size_t)cap * sizeof( int ) );
	sys->particleContactCapacity = cap;
}

static void PushParticleContact( lfParticleSystem* sys, int a, int b, b2Vec2 normal, float weight )
{
	GrowParticleContactCap( sys, sys->particleContactCount + 1 );
	lfParticleContact* c = &sys->particleContacts[sys->particleContactCount++];
	c->a = (uint16_t)a;
	c->b = (uint16_t)b;
	c->normal = normal;
	c->weight = weight;
}

static void PushContactBucket( lfParticleSystem* sys, int workerIndex, int a, int b, b2Vec2 normal, float weight )
{
	int* count = &sys->contactBucketCount[workerIndex];
	int* cap = &sys->contactBucketCap[workerIndex];
	if ( *count == *cap )
	{
		int next = *cap < 256 ? 256 : *cap * 2;
		sys->contactBucket[workerIndex] =
			(lfParticleContact*)realloc( sys->contactBucket[workerIndex], (size_t)next * sizeof( lfParticleContact ) );
		*cap = next;
	}
	lfParticleContact* c = &sys->contactBucket[workerIndex][( *count )++];
	c->a = (uint16_t)a;
	c->b = (uint16_t)b;
	c->normal = normal;
	c->weight = weight;
}

static void FindContactsRange( int start, int end, int workerIndex, void* context )
{
	lfParticleSystem* sys = (lfParticleSystem*)context;
	const float squaredDiameter = sys->diameter * sys->diameter;
	const float invDiameter = sys->invDiameter;
	const float* posX = sys->posX;
	const float* posY = sys->posY;

	for ( int i = start; i < end; i++ )
	{
		int ix = sys->cellX[i];
		int iy = sys->cellY[i];
		for ( int dy = -1; dy <= 1; dy++ )
		{
			for ( int dx = -1; dx <= 1; dx++ )
			{
				uint32_t cell = HashCell( ix + dx, iy + dy, sys->hashSize );
				for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
				{
					if ( (int)j <= i )
					{
						continue;
					}
					float ddx = posX[j] - posX[i];
					float ddy = posY[j] - posY[i];
					float distSqr = ddx * ddx + ddy * ddy;
					if ( distSqr < squaredDiameter && distSqr > 1e-9f )
					{
						float invD = lfInvSqrt( distSqr );
						b2Vec2 normal = { ddx * invD, ddy * invD };
						float weight = 1.0f - distSqr * invD * invDiameter;
						PushContactBucket( sys, workerIndex, i, (int)j, normal, weight );
					}
				}
			}
		}
	}
}

static void MergeContactBuckets( lfParticleSystem* sys, int workerCount )
{
	int total = 0;
	for ( int w = 0; w < workerCount; w++ )
	{
		total += sys->contactBucketCount[w];
	}
	GrowParticleContactCap( sys, total );
	sys->particleContactCount = 0;
	for ( int w = 0; w < workerCount; w++ )
	{
		int n = sys->contactBucketCount[w];
		if ( n <= 0 )
		{
			continue;
		}
		memcpy( sys->particleContacts + sys->particleContactCount, sys->contactBucket[w],
				(size_t)n * sizeof( lfParticleContact ) );
		sys->particleContactCount += n;
	}
}

static void FindParticleContacts( lfParticleSystem* sys )
{
	sys->particleContactCount = 0;
	const int n = sys->count;
	if ( n < LF_CONTACT_PARALLEL_MIN || g_lfWorkerCount <= 1 || g_lfEnqueue == NULL )
	{
		const float squaredDiameter = sys->diameter * sys->diameter;
		const float invDiameter = sys->invDiameter;
		const float* posX = sys->posX;
		const float* posY = sys->posY;
		for ( int i = 0; i < n; i++ )
		{
			int ix = sys->cellX[i];
			int iy = sys->cellY[i];
			for ( int dy = -1; dy <= 1; dy++ )
			{
				for ( int dx = -1; dx <= 1; dx++ )
				{
					uint32_t cell = HashCell( ix + dx, iy + dy, sys->hashSize );
					for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
					{
						if ( (int)j <= i )
						{
							continue;
						}
						float ddx = posX[j] - posX[i];
						float ddy = posY[j] - posY[i];
						float distSqr = ddx * ddx + ddy * ddy;
						if ( distSqr < squaredDiameter && distSqr > 1e-9f )
						{
							float invD = lfInvSqrt( distSqr );
							b2Vec2 normal = { ddx * invD, ddy * invD };
							float weight = 1.0f - distSqr * invD * invDiameter;
							PushParticleContact( sys, i, j, normal, weight );
						}
					}
				}
			}
		}
		return;
	}

	const int workerCount = g_lfWorkerCount;
	for ( int w = 0; w < workerCount; w++ )
	{
		sys->contactBucketCount[w] = 0;
	}
	lfParallelFor( &FindContactsRange, n, LF_CONTACT_MIN_RANGE, sys );
	MergeContactBuckets( sys, workerCount );
}

// ------------------------------------------------------------------------
// Particle / rigid body contact detection, via Box2D's public query API
// ------------------------------------------------------------------------

static void PushBodyContact( lfParticleSystem* sys, int index, b2BodyId bodyId, b2ShapeId shapeId, b2Vec2 normal,
							 float weight, float invMassA, float mass )
{
	if ( sys->bodyContactCount == sys->bodyContactCapacity )
	{
		sys->bodyContactCapacity *= 2;
		sys->bodyContacts = (lfBodyContact*)realloc( sys->bodyContacts, (size_t)sys->bodyContactCapacity * sizeof( lfBodyContact ) );
	}
	lfBodyContact* c = &sys->bodyContacts[sys->bodyContactCount++];
	c->index = (uint16_t)index;
	c->bodyId = bodyId;
	c->shapeId = shapeId;
	c->normal = normal;
	c->weight = weight;
	c->invMassA = invMassA;
	c->mass = mass;
}

// Signed distance from a point to a shape. n is the direction distance
// increases (outward). Inside a convex hull, d < 0 and n is the closest face.
static void ShapeComputeDistance( b2ShapeId shapeId, b2Pos ap, b2Vec2* nOut, float* dOut )
{
	b2BodyId bodyId = b2Shape_GetBody( shapeId );
	b2ShapeType type = b2Shape_GetType( shapeId );

	if ( type == b2_polygonShape )
	{
		b2Polygon poly = b2Shape_GetPolygon( shapeId );
		b2Vec2 pLocal = b2Body_GetLocalPoint( bodyId, ap );
		float maxDistance = -FLT_MAX;
		b2Vec2 normalForMax = pLocal;
		for ( int i = 0; i < poly.count; i++ )
		{
			float dot = b2Dot( poly.normals[i], b2Sub( pLocal, poly.vertices[i] ) );
			if ( dot > maxDistance )
			{
				maxDistance = dot;
				normalForMax = poly.normals[i];
			}
		}

		if ( maxDistance > 0.0f )
		{
			b2Vec2 minVec = normalForMax;
			float minD2 = maxDistance * maxDistance;
			for ( int i = 0; i < poly.count; i++ )
			{
				b2Vec2 dv = b2Sub( pLocal, poly.vertices[i] );
				float d2 = b2LengthSquared( dv );
				if ( d2 < minD2 )
				{
					minVec = dv;
					minD2 = d2;
				}
			}
			float d = sqrtf( minD2 );
			b2Vec2 n = b2Body_GetWorldVector( bodyId, minVec );
			float nLen = b2Length( n );
			*nOut = nLen > 1e-6f ? b2MulSV( 1.0f / nLen, n ) : ( b2Vec2 ){ 1.0f, 0.0f };
			*dOut = d - poly.radius;
		}
		else
		{
			*nOut = b2Body_GetWorldVector( bodyId, normalForMax );
			*dOut = maxDistance - poly.radius;
		}
		return;
	}

	if ( type == b2_circleShape )
	{
		b2Circle circle = b2Shape_GetCircle( shapeId );
		b2Pos center = b2Body_GetWorldPoint( bodyId, circle.center );
		b2Vec2 delta = b2Sub( ap, center );
		float d1 = b2Length( delta );
		if ( d1 > 1e-6f )
		{
			*nOut = b2MulSV( 1.0f / d1, delta );
			*dOut = d1 - circle.radius;
		}
		else
		{
			*nOut = ( b2Vec2 ){ 1.0f, 0.0f };
			*dOut = -circle.radius;
		}
		return;
	}

	if ( type == b2_capsuleShape )
	{
		b2Capsule cap = b2Shape_GetCapsule( shapeId );
		b2Vec2 pLocal = b2Body_GetLocalPoint( bodyId, ap );
		b2Vec2 d = b2Sub( pLocal, cap.center1 );
		b2Vec2 s = b2Sub( cap.center2, cap.center1 );
		float ds = b2Dot( d, s );
		if ( ds > 0.0f )
		{
			float s2 = b2Dot( s, s );
			if ( ds > s2 )
			{
				d = b2Sub( pLocal, cap.center2 );
			}
			else if ( s2 > 1e-12f )
			{
				d = b2Sub( d, b2MulSV( ds / s2, s ) );
			}
		}
		float d1 = b2Length( d );
		b2Vec2 nLocal;
		float dist;
		if ( d1 > 1e-6f )
		{
			nLocal = b2MulSV( 1.0f / d1, d );
			dist = d1 - cap.radius;
		}
		else
		{
			float s2 = b2LengthSquared( s );
			nLocal = s2 > 1e-12f ? b2Normalize( ( b2Vec2 ){ -s.y, s.x } ) : ( b2Vec2 ){ 1.0f, 0.0f };
			dist = -cap.radius;
		}
		*nOut = b2Body_GetWorldVector( bodyId, nLocal );
		*dOut = dist;
		return;
	}

	if ( type == b2_segmentShape || type == b2_chainSegmentShape )
	{
		b2Vec2 v1;
		b2Vec2 v2;
		if ( type == b2_segmentShape )
		{
			b2Segment seg = b2Shape_GetSegment( shapeId );
			v1 = b2Body_GetWorldPoint( bodyId, seg.point1 );
			v2 = b2Body_GetWorldPoint( bodyId, seg.point2 );
		}
		else
		{
			b2ChainSegment chain = b2Shape_GetChainSegment( shapeId );
			v1 = b2Body_GetWorldPoint( bodyId, chain.segment.point1 );
			v2 = b2Body_GetWorldPoint( bodyId, chain.segment.point2 );
		}
		b2Vec2 d = b2Sub( ap, v1 );
		b2Vec2 s = b2Sub( v2, v1 );
		float ds = b2Dot( d, s );
		if ( ds > 0.0f )
		{
			float s2 = b2Dot( s, s );
			if ( ds > s2 )
			{
				d = b2Sub( ap, v2 );
			}
			else if ( s2 > 1e-12f )
			{
				d = b2Sub( d, b2MulSV( ds / s2, s ) );
			}
		}
		float d1 = b2Length( d );
		*dOut = d1;
		*nOut = d1 > 1e-6f ? b2MulSV( 1.0f / d1, d ) : ( b2Vec2 ){ 1.0f, 0.0f };
		return;
	}

	*nOut = ( b2Vec2 ){ 1.0f, 0.0f };
	*dOut = FLT_MAX;
}

static void ContactParticleWithShape( lfParticleSystem* sys, int i, b2ShapeId shapeId )
{
	b2Pos ap = ( (b2Vec2){ sys->posX[i], sys->posY[i] } );
	b2Vec2 nCompute;
	float d;
	ShapeComputeDistance( shapeId, ap, &nCompute, &d );

	if ( d >= sys->diameter )
	{
		return;
	}

	b2BodyId bodyId = b2Shape_GetBody( shapeId );
	float invMassA = ( sys->flags[i] & lf_wallParticle ) ? 0.0f : sys->particleInvMass;

	float invMassB = 0.0f;
	float invIB = 0.0f;
	b2Vec2 bodyCenter = ( b2Vec2 ){ 0.0f, 0.0f };
	if ( b2Body_GetType( bodyId ) == b2_dynamicBody )
	{
		float bm = b2Body_GetMass( bodyId );
		float bI = b2Body_GetRotationalInertia( bodyId );
		invMassB = bm > 0.0f ? 1.0f / bm : 0.0f;
		invIB = bI > 0.0f ? 1.0f / bI : 0.0f;
		bodyCenter = b2Body_GetWorldCenter( bodyId );
	}

	b2Vec2 r = b2Sub( ap, bodyCenter );
	float rn = b2Cross( r, nCompute );
	float invMassSum = invMassA + invMassB + invIB * rn * rn;
	float mass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;
	float weight = 1.0f - d * sys->invDiameter;
	PushBodyContact( sys, i, bodyId, shapeId, b2Neg( nCompute ), weight, invMassA, mass );
}

static bool CollectShapeCallback( b2ShapeId shapeId, void* context )
{
	lfParticleSystem* sys = (lfParticleSystem*)context;
	if ( sys->queryShapeCount == sys->queryShapeCapacity )
	{
		sys->queryShapeCapacity *= 2;
		sys->queryShapes = (b2ShapeId*)realloc( sys->queryShapes, (size_t)sys->queryShapeCapacity * sizeof( b2ShapeId ) );
	}
	sys->queryShapes[sys->queryShapeCount++] = shapeId;
	return true;
}

static void CollectOverlappingShapes( lfParticleSystem* sys, b2AABB aabb )
{
	sys->queryShapeCount = 0;
	b2QueryFilter filter = b2DefaultQueryFilter();
	b2World_OverlapAABB( sys->worldId, b2Pos_zero, aabb, filter, CollectShapeCallback, sys );
}

static void ForEachParticleNearShape( lfParticleSystem* sys, b2ShapeId shapeId, float pad,
									  void ( *fn )( lfParticleSystem*, int, b2ShapeId ) )
{
	b2AABB sa = b2Shape_GetAABB( shapeId );
	sa.lowerBound.x -= pad;
	sa.lowerBound.y -= pad;
	sa.upperBound.x += pad;
	sa.upperBound.y += pad;

	int ix0, iy0, ix1, iy1;
	GetCell( sys, sa.lowerBound, &ix0, &iy0 );
	GetCell( sys, sa.upperBound, &ix1, &iy1 );

	for ( int iy = iy0; iy <= iy1; iy++ )
	{
		for ( int ix = ix0; ix <= ix1; ix++ )
		{
			uint32_t cell = HashCell( ix, iy, sys->hashSize );
			for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
			{
				if ( sys->cellX[j] != ix || sys->cellY[j] != iy )
				{
					continue;
				}
				fn( sys, (int)j, shapeId );
			}
		}
	}
}

// Caller must have already populated sys->queryShapes for this sub-step (see
// lfParticleSystem_Step - shared with SolveCollision, one b2World_OverlapAABB
// instead of two: the swept-cloud AABB it queries against is always a
// superset of the static-cloud AABB this pass alone would need, same pad).
static void FindBodyContacts( lfParticleSystem* sys )
{
	sys->bodyContactCount = 0;
	if ( sys->count == 0 )
	{
		return;
	}

	for ( int s = 0; s < sys->queryShapeCount; s++ )
	{
		ForEachParticleNearShape( sys, sys->queryShapes[s], sys->diameter, ContactParticleWithShape );
	}
}

// LF 1.1.0 strictContactCheck: floor+wall corners invent a diagonal contact.
// Sort by particle then weight; project back along inverse (inward) normal;
// drop if that probe is not on/in the fixture. Keep at most 3 per particle.
static bool BodyContactSurvives( const lfParticleSystem* sys, const lfBodyContact* c )
{
	b2Vec2 p = ( (b2Vec2){ sys->posX[c->index], sys->posY[c->index] } );
	float travel = sys->diameter * ( 1.0f - c->weight );
	b2Vec2 probe = b2Add( p, b2MulSV( travel, c->normal ) );
	if ( b2Shape_TestPoint( c->shapeId, probe ) )
	{
		return true;
	}
	b2Vec2 n;
	float d;
	ShapeComputeDistance( c->shapeId, probe, &n, &d );
	return d < B2_LINEAR_SLOP;
}

static int BodyContactCompare( const void* a, const void* b )
{
	const lfBodyContact* ca = (const lfBodyContact*)a;
	const lfBodyContact* cb = (const lfBodyContact*)b;
	if ( ca->index != cb->index )
	{
		return ca->index - cb->index;
	}
	if ( cb->weight > ca->weight )
	{
		return 1;
	}
	if ( cb->weight < ca->weight )
	{
		return -1;
	}
	return 0;
}

static void RemoveSpuriousBodyContacts( lfParticleSystem* sys )
{
	if ( sys->bodyContactCount < 2 || sys->count == 0 )
	{
		return;
	}

	// H5 hypothesis (insertion sort instead of qsort) was tried and rejected -
	// see docs/LIQUIDFUN_HYPOTHESES.md. The "<=3 kept per particle" cap is
	// post-filter output; the array sorted here is every live body contact
	// pre-filter, which for a settled puddle resting on a wide floor is not
	// small - O(n^2) insertion sort measurably lost to qsort at that size.
	qsort( sys->bodyContacts, (size_t)sys->bodyContactCount, sizeof( lfBodyContact ), BodyContactCompare );

	int tail = 0;
	int lastIndex = -1;
	int kept = 0;
	for ( int k = 0; k < sys->bodyContactCount; k++ )
	{
		lfBodyContact c = sys->bodyContacts[k];
		if ( c.index != lastIndex )
		{
			lastIndex = c.index;
			kept = 0;
		}
		if ( kept >= 3 )
		{
			continue;
		}
		if ( !BodyContactSurvives( sys, &c ) )
		{
			continue;
		}
		sys->bodyContacts[tail++] = c;
		kept++;
	}
	sys->bodyContactCount = tail;
}

static b2AABB ComputeSweptCloudAABB( const lfParticleSystem* sys, float dt, float pad )
{
	b2AABB aabb = { { 1e9f, 1e9f }, { -1e9f, -1e9f } };
	for ( int i = 0; i < sys->count; i++ )
	{
		b2Vec2 p = ( (b2Vec2){ sys->posX[i], sys->posY[i] } );
		b2Vec2 p2 = b2Add( p, b2MulSV( dt, ( (b2Vec2){ sys->velX[i], sys->velY[i] } ) ) );
		float minx = p.x < p2.x ? p.x : p2.x;
		float miny = p.y < p2.y ? p.y : p2.y;
		float maxx = p.x > p2.x ? p.x : p2.x;
		float maxy = p.y > p2.y ? p.y : p2.y;
		if ( minx < aabb.lowerBound.x )
		{
			aabb.lowerBound.x = minx;
		}
		if ( miny < aabb.lowerBound.y )
		{
			aabb.lowerBound.y = miny;
		}
		if ( maxx > aabb.upperBound.x )
		{
			aabb.upperBound.x = maxx;
		}
		if ( maxy > aabb.upperBound.y )
		{
			aabb.upperBound.y = maxy;
		}
	}
	aabb.lowerBound.x -= pad;
	aabb.lowerBound.y -= pad;
	aabb.upperBound.x += pad;
	aabb.upperBound.y += pad;
	return aabb;
}

static void StopAtSurface( lfParticleSystem* sys, int i, b2Vec2 p, b2Vec2 p1, b2Vec2 p2, b2Vec2 n, float fraction )
{
	b2Vec2 hit = b2Add( b2MulSV( 1.0f - fraction, p1 ), b2MulSV( fraction, p2 ) );
	b2Vec2 target = b2Add( hit, b2MulSV( B2_LINEAR_SLOP, n ) );
	{ b2Vec2 __v = b2MulSV( s_collisionInvDt, b2Sub( target, p ) ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
}

static void RayCastParticleShape( lfParticleSystem* sys, int i, b2ShapeId shapeId )
{
	if ( sys->flags[i] & lf_wallParticle )
	{
		return;
	}

	b2Pos p = ( (b2Vec2){ sys->posX[i], sys->posY[i] } );
	b2Vec2 translation = b2MulSV( s_collisionDt, ( (b2Vec2){ sys->velX[i], sys->velY[i] } ) );
	b2WorldCastOutput hit = { 0 };
	if ( b2LengthSquared( translation ) >= 1e-12f )
	{
		hit = b2Shape_RayCast( shapeId, p, translation );
	}

	// Box2D 3 initial overlap: hit=true, fraction=0, normal=0. Google only
	// StopAtSurface on a real ray normal; interior particles keep pressure
	// velocity (closest-face ComputeDistance) and integrate out.
	if ( hit.hit )
	{
		float nLen = b2Length( hit.normal );
		if ( nLen > 1e-6f )
		{
			b2Vec2 n = b2MulSV( 1.0f / nLen, hit.normal );
			b2Vec2 p2 = b2Add( p, translation );
			StopAtSurface( sys, i, p, p, p2, n, hit.fraction );
		}
	}
}

// Reuses sys->queryShapes from the shared query in lfParticleSystem_Step
// (see FindBodyContacts) - it was already built from this same dt's swept AABB.
static void SolveCollision( lfParticleSystem* sys, float dt )
{
	if ( sys->count == 0 || dt <= 1e-12f )
	{
		return;
	}

	s_collisionDt = dt;
	s_collisionInvDt = 1.0f / dt;
	for ( int s = 0; s < sys->queryShapeCount; s++ )
	{
		ForEachParticleNearShape( sys, sys->queryShapes[s], sys->diameter, RayCastParticleShape );
	}
}

// ------------------------------------------------------------------------
// Density estimate (per-particle accumulated contact weight)
// ------------------------------------------------------------------------

static void ComputeWeight( lfParticleSystem* sys )
{
	memset( sys->weight, 0, (size_t)sys->count * sizeof( float ) ); // 0.0f is all-zero bits
	for ( int k = 0; k < sys->particleContactCount; k++ )
	{
		const lfParticleContact* c = &sys->particleContacts[k];
		sys->weight[c->a] += c->weight;
		sys->weight[c->b] += c->weight;
	}
	for ( int k = 0; k < sys->bodyContactCount; k++ )
	{
		const lfBodyContact* c = &sys->bodyContacts[k];
		sys->weight[c->index] += c->weight;
	}
}

// ------------------------------------------------------------------------
// Force / velocity passes
// ------------------------------------------------------------------------

// Poisson static pressure. LiquidFun 1.1.0: pressurePerWeight =
// staticPressureStrength * density * (diameter/dt)^2.
static void SolveStaticPressure( lfParticleSystem* sys, float dt )
{
	if ( ( sys->flagOr & lf_staticPressureParticle ) == 0 )
	{
		return;
	}

	float invDt = 1.0f / dt;
	float criticalVelocity = sys->diameter * invDt;
	float criticalPressure = sys->def.density * criticalVelocity * criticalVelocity;
	float pressurePerWeight = sys->def.staticPressureStrength * criticalPressure;
	float maxPressure = LF_MAX_PARTICLE_PRESSURE * criticalPressure;
	float relaxation = sys->def.staticPressureRelaxation;
	int iters = sys->def.staticPressureIterations;
	if ( iters < 1 )
	{
		iters = 1;
	}

	// Compact the qualifying contacts once instead of re-filtering the full
	// particleContacts array every Poisson iteration below.
	int qualifyingCount = 0;
	for ( int k = 0; k < sys->particleContactCount; k++ )
	{
		const lfParticleContact* c = &sys->particleContacts[k];
		if ( ( sys->flags[c->a] | sys->flags[c->b] ) & lf_staticPressureParticle )
		{
			sys->staticPressureContactIndices[qualifyingCount++] = k;
		}
	}

	for ( int t = 0; t < iters; t++ )
	{
		memset( sys->staticPressureAccum, 0, (size_t)sys->count * sizeof( float ) );
		for ( int q = 0; q < qualifyingCount; q++ )
		{
			const lfParticleContact* c = &sys->particleContacts[sys->staticPressureContactIndices[q]];
			float w = c->weight;
			sys->staticPressureAccum[c->a] += w * sys->staticPressure[c->b];
			sys->staticPressureAccum[c->b] += w * sys->staticPressure[c->a];
		}
		for ( int i = 0; i < sys->count; i++ )
		{
			if ( ( sys->flags[i] & lf_staticPressureParticle ) == 0 )
			{
				sys->staticPressure[i] = 0.0f;
				continue;
			}
			float w = sys->weight[i];
			float h = ( sys->staticPressureAccum[i] + pressurePerWeight * ( w - LF_MIN_PARTICLE_WEIGHT ) ) /
					  ( w + relaxation );
			if ( h < 0.0f )
			{
				h = 0.0f;
			}
			else if ( h > maxPressure )
			{
				h = maxPressure;
			}
			sys->staticPressure[i] = h;
		}
	}
}

static bool BarrierCrossing( float e0, float e1, float e2, float tmax, b2Vec2 pba, b2Vec2 vba,
							 b2Vec2 pca, b2Vec2 vca, float* sOut )
{
	float s, t;
	b2Vec2 qba, qca;
	if ( e2 == 0.0f )
	{
		if ( e1 == 0.0f )
		{
			return false;
		}
		t = -e0 / e1;
		if ( !( t >= 0.0f && t < tmax ) )
		{
			return false;
		}
		qba = b2Add( pba, b2MulSV( t, vba ) );
		qca = b2Add( pca, b2MulSV( t, vca ) );
		float qbaSqr = b2Dot( qba, qba );
		if ( qbaSqr <= 1e-12f )
		{
			return false;
		}
		s = b2Dot( qba, qca ) / qbaSqr;
		if ( !( s >= 0.0f && s <= 1.0f ) )
		{
			return false;
		}
		*sOut = s;
		return true;
	}

	float det = e1 * e1 - 4.0f * e0 * e2;
	if ( det < 0.0f )
	{
		return false;
	}
	float sqrtDet = sqrtf( det );
	float t1 = ( -e1 - sqrtDet ) / ( 2.0f * e2 );
	float t2 = ( -e1 + sqrtDet ) / ( 2.0f * e2 );
	if ( t1 > t2 )
	{
		float tmp = t1;
		t1 = t2;
		t2 = tmp;
	}
	t = t1;
	qba = b2Add( pba, b2MulSV( t, vba ) );
	qca = b2Add( pca, b2MulSV( t, vca ) );
	float qbaSqr = b2Dot( qba, qba );
	s = qbaSqr > 1e-12f ? b2Dot( qba, qca ) / qbaSqr : -1.0f;
	if ( !( t >= 0.0f && t < tmax && s >= 0.0f && s <= 1.0f ) )
	{
		t = t2;
		if ( !( t >= 0.0f && t < tmax ) )
		{
			return false;
		}
		qba = b2Add( pba, b2MulSV( t, vba ) );
		qca = b2Add( pca, b2MulSV( t, vca ) );
		qbaSqr = b2Dot( qba, qba );
		if ( qbaSqr <= 1e-12f )
		{
			return false;
		}
		s = b2Dot( qba, qca ) / qbaSqr;
		if ( !( s >= 0.0f && s <= 1.0f ) )
		{
			return false;
		}
	}
	*sOut = s;
	return true;
}

static void SolveBarrier( lfParticleSystem* sys, float dt )
{
	if ( ( sys->flagOr & lf_barrierParticle ) == 0 )
	{
		return;
	}

	const uint32_t wallBarrier = lf_barrierParticle | lf_wallParticle;
	for ( int i = 0; i < sys->count; i++ )
	{
		if ( ( sys->flags[i] & wallBarrier ) == wallBarrier )
		{
			{ b2Vec2 __v = ( b2Vec2 ){ 0.0f, 0.0f }; sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
		}
	}

	float tmax = LF_BARRIER_COLLISION_TIME * dt;
	float pad = sys->diameter;

	for ( int k = 0; k < sys->pairCount; k++ )
	{
		const lfParticlePair* pair = &sys->pairs[k];
		if ( ( pair->flags & lf_barrierParticle ) == 0 )
		{
			continue;
		}
		int a = (int)pair->a;
		int b = (int)pair->b;
		if ( a < 0 || a >= sys->count || b < 0 || b >= sys->count )
		{
			continue;
		}

		b2Vec2 pa = ( (b2Vec2){ sys->posX[a], sys->posY[a] } );
		b2Vec2 pb = ( (b2Vec2){ sys->posX[b], sys->posY[b] } );
		b2Vec2 va = ( (b2Vec2){ sys->velX[a], sys->velY[a] } );
		b2Vec2 vb = ( (b2Vec2){ sys->velX[b], sys->velY[b] } );
		b2Vec2 pba = b2Sub( pb, pa );
		b2Vec2 vba = b2Sub( vb, va );

		b2AABB aabb;
		aabb.lowerBound.x = fminf( pa.x, pb.x ) - pad;
		aabb.lowerBound.y = fminf( pa.y, pb.y ) - pad;
		aabb.upperBound.x = fmaxf( pa.x, pb.x ) + pad;
		aabb.upperBound.y = fmaxf( pa.y, pb.y ) + pad;

		int ix0, iy0, ix1, iy1;
		GetCell( sys, aabb.lowerBound, &ix0, &iy0 );
		GetCell( sys, aabb.upperBound, &ix1, &iy1 );

		for ( int iy = iy0; iy <= iy1; iy++ )
		{
			for ( int ix = ix0; ix <= ix1; ix++ )
			{
				uint32_t cell = HashCell( ix, iy, sys->hashSize );
				for ( uint16_t jc = sys->cellHead[cell]; jc != LF_EMPTY_PARTICLE; jc = sys->next[jc] )
				{
					int c = (int)jc;
					if ( sys->cellX[c] != ix || sys->cellY[c] != iy )
					{
						continue;
					}
					if ( c == a || c == b || ( sys->flags[c] & lf_wallParticle ) )
					{
						continue;
					}

					b2Vec2 pc = ( (b2Vec2){ sys->posX[c], sys->posY[c] } );
					b2Vec2 vc = ( (b2Vec2){ sys->velX[c], sys->velY[c] } );
					b2Vec2 pca = b2Sub( pc, pa );
					b2Vec2 vca = b2Sub( vc, va );
					float e2 = b2Cross( vba, vca );
					float e1 = b2Cross( pba, vca ) - b2Cross( pca, vba );
					float e0 = b2Cross( pba, pca );
					float s;
					if ( !BarrierCrossing( e0, e1, e2, tmax, pba, vba, pca, vca, &s ) )
					{
						continue;
					}
					b2Vec2 dv = b2Sub( b2Add( va, b2MulSV( s, vba ) ), vc );
					{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c], sys->velY[c] } ), dv ); sys->velX[c] = __v.x; sys->velY[c] = __v.y; }
				}
			}
		}
	}
}

typedef struct lfRangeJob
{
	lfParticleSystem* sys;
	float dt;
	float ax;
	float ay;
} lfRangeJob;

static void GravityRange( int start, int end, int workerIndex, void* context )
{
	(void)workerIndex;
	lfRangeJob* job = (lfRangeJob*)context;
	float* vx = job->sys->velX;
	float* vy = job->sys->velY;
	__m128 dvx = _mm_set1_ps( job->ax );
	__m128 dvy = _mm_set1_ps( job->ay );
	int i = start;
	int aligned = ( i + 3 ) & ~3;
	if ( aligned > end )
	{
		aligned = end;
	}
	for ( ; i < aligned; i++ )
	{
		vx[i] += job->ax;
		vy[i] += job->ay;
	}
	for ( ; i + 4 <= end; i += 4 )
	{
		_mm_storeu_ps( vx + i, _mm_add_ps( _mm_loadu_ps( vx + i ), dvx ) );
		_mm_storeu_ps( vy + i, _mm_add_ps( _mm_loadu_ps( vy + i ), dvy ) );
	}
	for ( ; i < end; i++ )
	{
		vx[i] += job->ax;
		vy[i] += job->ay;
	}
}

static void SolveGravity( lfParticleSystem* sys, float dt )
{
	b2Vec2 gravity = b2World_GetGravity( sys->worldId );
	b2Vec2 dv = b2MulSV( dt, gravity );
	lfRangeJob job = { sys, dt, dv.x, dv.y };
	GravityRange( 0, sys->count, 0, &job );
}

static void LimitVelocityRange( int start, int end, int workerIndex, void* context )
{
	(void)workerIndex;
	lfRangeJob* job = (lfRangeJob*)context;
	float* vx = job->sys->velX;
	float* vy = job->sys->velY;
	const float maxSpeed = job->ax;
	const float maxSpeedSqr = job->ay;
	__m128 maxSpeedSqrV = _mm_set1_ps( maxSpeedSqr );
	__m128 maxSpeedV = _mm_set1_ps( maxSpeed );
	int i = start;
	int aligned = ( i + 3 ) & ~3;
	if ( aligned > end )
	{
		aligned = end;
	}
	for ( ; i < aligned; i++ )
	{
		float speedSqr = vx[i] * vx[i] + vy[i] * vy[i];
		if ( speedSqr > maxSpeedSqr )
		{
			float s = maxSpeed / sqrtf( speedSqr );
			vx[i] *= s;
			vy[i] *= s;
		}
	}
	for ( ; i + 4 <= end; i += 4 )
	{
		__m128 x = _mm_loadu_ps( vx + i );
		__m128 y = _mm_loadu_ps( vy + i );
		__m128 speedSqr = _mm_add_ps( _mm_mul_ps( x, x ), _mm_mul_ps( y, y ) );
		__m128 mask = _mm_cmpgt_ps( speedSqr, maxSpeedSqrV );
		__m128 scale = _mm_div_ps( maxSpeedV, _mm_sqrt_ps( speedSqr ) );
		_mm_storeu_ps( vx + i, _mm_or_ps( _mm_and_ps( mask, _mm_mul_ps( x, scale ) ), _mm_andnot_ps( mask, x ) ) );
		_mm_storeu_ps( vy + i, _mm_or_ps( _mm_and_ps( mask, _mm_mul_ps( y, scale ) ), _mm_andnot_ps( mask, y ) ) );
	}
	for ( ; i < end; i++ )
	{
		float speedSqr = vx[i] * vx[i] + vy[i] * vy[i];
		if ( speedSqr > maxSpeedSqr )
		{
			float s = maxSpeed / sqrtf( speedSqr );
			vx[i] *= s;
			vy[i] *= s;
		}
	}
}

static void LimitVelocity( lfParticleSystem* sys, float dt )
{
	float maxSpeed = sys->diameter / dt;
	lfRangeJob job = { sys, dt, maxSpeed, maxSpeed * maxSpeed };
	LimitVelocityRange( 0, sys->count, 0, &job );
}

static void WallRange( int start, int end, int workerIndex, void* context )
{
	(void)workerIndex;
	lfParticleSystem* sys = (lfParticleSystem*)context;
	uint32_t* flags = sys->flags;
	float* vx = sys->velX;
	float* vy = sys->velY;
	for ( int i = start; i < end; i++ )
	{
		if ( flags[i] & lf_wallParticle )
		{
			vx[i] = 0.0f;
			vy[i] = 0.0f;
		}
	}
}

static void SolveWall( lfParticleSystem* sys )
{
	if ( ( sys->flagOr & lf_wallParticle ) == 0 )
	{
		return;
	}
	WallRange( 0, sys->count, 0, sys );
}

static void IntegrateRange( int start, int end, int workerIndex, void* context )
{
	(void)workerIndex;
	lfRangeJob* job = (lfRangeJob*)context;
	float* px = job->sys->posX;
	float* py = job->sys->posY;
	const float* vx = job->sys->velX;
	const float* vy = job->sys->velY;
	const float dt = job->dt;
	__m128 dtv = _mm_set1_ps( dt );
	int i = start;
	int aligned = ( i + 3 ) & ~3;
	if ( aligned > end )
	{
		aligned = end;
	}
	for ( ; i < aligned; i++ )
	{
		px[i] += dt * vx[i];
		py[i] += dt * vy[i];
	}
	for ( ; i + 4 <= end; i += 4 )
	{
		__m128 p = _mm_loadu_ps( px + i );
		__m128 v = _mm_loadu_ps( vx + i );
		_mm_storeu_ps( px + i, _mm_add_ps( p, _mm_mul_ps( dtv, v ) ) );
		p = _mm_loadu_ps( py + i );
		v = _mm_loadu_ps( vy + i );
		_mm_storeu_ps( py + i, _mm_add_ps( p, _mm_mul_ps( dtv, v ) ) );
	}
	for ( ; i < end; i++ )
	{
		px[i] += dt * vx[i];
		py[i] += dt * vy[i];
	}
}

static void Integrate( lfParticleSystem* sys, float dt )
{
	lfRangeJob job = { sys, dt, 0.0f, 0.0f };
	IntegrateRange( 0, sys->count, 0, &job );
}

static void SolvePressure( lfParticleSystem* sys, float dt )
{
	float invDt = 1.0f / dt;
	float criticalVelocity = sys->diameter * invDt;
	float criticalPressure = sys->def.density * criticalVelocity * criticalVelocity;
	float pressurePerWeight = sys->def.pressureStrength * criticalPressure;
	float maxPressure = LF_MAX_PARTICLE_PRESSURE * criticalPressure;
	float* h = sys->staticPressureAccum;

	for ( int i = 0; i < sys->count; i++ )
	{
		float w = sys->weight[i];
		float p = pressurePerWeight * fmaxf( 0.0f, w - LF_MIN_PARTICLE_WEIGHT );
		if ( p > maxPressure )
		{
			p = maxPressure;
		}
		if ( sys->flags[i] & LF_NO_PRESSURE_FLAGS )
		{
			p = 0.0f;
		}
		h[i] = p;
	}
	if ( sys->flagOr & lf_staticPressureParticle )
	{
		for ( int i = 0; i < sys->count; i++ )
		{
			if ( sys->flags[i] & lf_staticPressureParticle )
			{
				h[i] += sys->staticPressure[i];
			}
		}
	}

	float velocityPerPressure = dt / ( sys->def.density * sys->diameter );
	for ( int idx = 0; idx < sys->bodyContactCount; idx++ )
	{
		const lfBodyContact* c = &sys->bodyContacts[idx];
		int a = (int)c->index;
		float w = c->weight;
		float hp = h[a] + pressurePerWeight * w;
		b2Vec2 f = b2MulSV( velocityPerPressure * w * c->mass * hp, c->normal );
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[a], sys->velY[a] } ), b2MulSV( c->invMassA, f ) ); sys->velX[a] = __v.x; sys->velY[a] = __v.y; }
		b2Body_ApplyLinearImpulse( c->bodyId, f, ( (b2Vec2){ sys->posX[a], sys->posY[a] } ), true );
	}
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		float hp = h[c->a] + h[c->b];
		b2Vec2 f = b2MulSV( velocityPerPressure * c->weight * hp, c->normal );
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ), f ); sys->velX[c->a] = __v.x; sys->velY[c->a] = __v.y; }
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), f ); sys->velX[c->b] = __v.x; sys->velY[c->b] = __v.y; }
	}
}

static void SolveDamping( lfParticleSystem* sys, float dt )
{
	float linearDamping = sys->def.dampingStrength;
	float quadraticDamping = dt / sys->diameter;
	for ( int idx = 0; idx < sys->bodyContactCount; idx++ )
	{
		const lfBodyContact* c = &sys->bodyContacts[idx];
		int a = (int)c->index;
		b2Vec2 p = ( (b2Vec2){ sys->posX[a], sys->posY[a] } );
		b2Vec2 v = b2Sub( b2Body_GetWorldPointVelocity( c->bodyId, p ), ( (b2Vec2){ sys->velX[a], sys->velY[a] } ) );
		float vn = b2Dot( v, c->normal );
		if ( vn >= 0.0f )
		{
			continue;
		}
		float damping = fmaxf( linearDamping * c->weight, fminf( -quadraticDamping * vn, 0.5f ) );
		b2Vec2 f = b2MulSV( damping * c->mass * vn, c->normal );
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[a], sys->velY[a] } ), b2MulSV( c->invMassA, f ) ); sys->velX[a] = __v.x; sys->velY[a] = __v.y; }
		b2Body_ApplyLinearImpulse( c->bodyId, b2Neg( f ), p, true );
	}
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		b2Vec2 v = b2Sub( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ) );
		float vn = b2Dot( v, c->normal );
		if ( vn >= 0.0f )
		{
			continue;
		}
		float damping = fmaxf( linearDamping * c->weight, fminf( -quadraticDamping * vn, 0.5f ) );
		b2Vec2 f = b2MulSV( damping * vn, c->normal );
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ), f ); sys->velX[c->a] = __v.x; sys->velY[c->a] = __v.y; }
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), f ); sys->velX[c->b] = __v.x; sys->velY[c->b] = __v.y; }
	}
}

static void SolveViscous( lfParticleSystem* sys )
{
	if ( ( sys->flagOr & lf_viscousParticle ) == 0 )
	{
		return;
	}
	float viscousBase = sys->def.viscousStrength;
	for ( int idx = 0; idx < sys->bodyContactCount; idx++ )
	{
		const lfBodyContact* c = &sys->bodyContacts[idx];
		int a = (int)c->index;
		if ( ( sys->flags[a] & lf_viscousParticle ) == 0 )
		{
			continue;
		}
		float viscous = viscousBase * sys->viscousScale[a];
		b2Vec2 p = ( (b2Vec2){ sys->posX[a], sys->posY[a] } );
		b2Vec2 v = b2Sub( b2Body_GetWorldPointVelocity( c->bodyId, p ), ( (b2Vec2){ sys->velX[a], sys->velY[a] } ) );
		b2Vec2 f = b2MulSV( viscous * c->mass * c->weight, v );
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[a], sys->velY[a] } ), b2MulSV( c->invMassA, f ) ); sys->velX[a] = __v.x; sys->velY[a] = __v.y; }
		b2Body_ApplyLinearImpulse( c->bodyId, b2Neg( f ), p, true );
	}
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		if ( ( ( sys->flags[c->a] | sys->flags[c->b] ) & lf_viscousParticle ) == 0 )
		{
			continue;
		}
		float viscous = viscousBase * 0.5f * ( sys->viscousScale[c->a] + sys->viscousScale[c->b] );
		b2Vec2 v = b2Sub( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ) );
		b2Vec2 f = b2MulSV( viscous * c->weight, v );
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ), f ); sys->velX[c->a] = __v.x; sys->velY[c->a] = __v.y; }
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), f ); sys->velX[c->b] = __v.x; sys->velY[c->b] = __v.y; }
	}
}

static void SolveTensile( lfParticleSystem* sys, float dt )
{
	if ( ( sys->flagOr & lf_tensileParticle ) == 0 )
	{
		return;
	}
	memset( sys->accumulation2, 0, (size_t)sys->count * sizeof( b2Vec2 ) ); // (0,0) is all-zero bits
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		if ( ( ( sys->flags[c->a] | sys->flags[c->b] ) & lf_tensileParticle ) == 0 )
		{
			continue;
		}
		b2Vec2 weightedNormal = b2MulSV( ( 1.0f - c->weight ) * c->weight, c->normal );
		sys->accumulation2[c->a] = b2Sub( sys->accumulation2[c->a], weightedNormal );
		sys->accumulation2[c->b] = b2Add( sys->accumulation2[c->b], weightedNormal );
	}
	float criticalVelocity = sys->diameter / dt;
	float pressureStrength = sys->def.tensileStrength * criticalVelocity;
	float normalStrength = sys->def.tensileStrength * criticalVelocity;
	float maxVelocityVariation = LF_MAX_PARTICLE_FORCE * criticalVelocity;
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		if ( ( ( sys->flags[c->a] | sys->flags[c->b] ) & lf_tensileParticle ) == 0 )
		{
			continue;
		}
		float hw = sys->weight[c->a] + sys->weight[c->b];
		b2Vec2 s = b2Sub( sys->accumulation2[c->b], sys->accumulation2[c->a] );
		float fn = pressureStrength * ( hw - 2.0f ) + normalStrength * b2Dot( s, c->normal );
		if ( fn > maxVelocityVariation )
		{
			fn = maxVelocityVariation;
		}
		fn *= c->weight;
		b2Vec2 f = b2MulSV( fn, c->normal );
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ), f ); sys->velX[c->a] = __v.x; sys->velY[c->a] = __v.y; }
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), f ); sys->velX[c->b] = __v.x; sys->velY[c->b] = __v.y; }
	}
}

static void SolvePowder( lfParticleSystem* sys, float dt )
{
	if ( ( sys->flagOr & lf_powderParticle ) == 0 )
	{
		return;
	}
	float powder = sys->def.powderStrength * ( sys->diameter / dt );
	float minWeight = 1.0f - LF_PARTICLE_STRIDE;
	for ( int idx = 0; idx < sys->particleContactCount; idx++ )
	{
		const lfParticleContact* c = &sys->particleContacts[idx];
		if ( ( ( sys->flags[c->a] | sys->flags[c->b] ) & lf_powderParticle ) == 0 )
		{
			continue;
		}
		if ( c->weight <= minWeight )
		{
			continue;
		}
		b2Vec2 f = b2MulSV( powder * ( c->weight - minWeight ), c->normal );
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[c->a], sys->velY[c->a] } ), f ); sys->velX[c->a] = __v.x; sys->velY[c->a] = __v.y; }
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[c->b], sys->velY[c->b] } ), f ); sys->velX[c->b] = __v.x; sys->velY[c->b] = __v.y; }
	}
}

// ------------------------------------------------------------------------
// Public step function
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Join / Split / ApplyForce / Solid / Rigid / Query (Google 1.1 behavior)
// ------------------------------------------------------------------------

static void PrepareForceBuffer( lfParticleSystem* sys )
{
	if ( sys->hasForce )
	{
		return;
	}
	memset( sys->force, 0, (size_t)sys->count * sizeof( b2Vec2 ) );
	sys->hasForce = true;
}

static void SolveForce( lfParticleSystem* sys, float dt )
{
	if ( !sys->hasForce || dt <= 0.0f )
	{
		return;
	}
	float velocityPerForce = dt * sys->particleInvMass;
	for ( int i = 0; i < sys->count; i++ )
	{
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[i], sys->velY[i] } ), b2MulSV( velocityPerForce, sys->force[i] ) ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
	}
	sys->hasForce = false;
}

void lfParticleSystem_ApplyForce( lfParticleSystem* sys, int firstIndex, int lastIndex, b2Vec2 force )
{
	if ( sys == NULL || firstIndex < 0 || lastIndex > sys->count || firstIndex >= lastIndex )
	{
		return;
	}
	int n = lastIndex - firstIndex;
	b2Vec2 distributed = b2MulSV( 1.0f / (float)n, force );
	if ( b2LengthSquared( distributed ) < 1e-20f )
	{
		return;
	}
	PrepareForceBuffer( sys );
	for ( int i = firstIndex; i < lastIndex; i++ )
	{
		if ( ( sys->flags[i] & lf_wallParticle ) != 0 )
		{
			continue;
		}
		sys->force[i] = b2Add( sys->force[i], distributed );
	}
}

void lfParticleSystem_ParticleApplyForce( lfParticleSystem* sys, int index, b2Vec2 force )
{
	if ( sys == NULL || index < 0 || index >= sys->count )
	{
		return;
	}
	if ( ( sys->flags[index] & lf_wallParticle ) != 0 || b2LengthSquared( force ) < 1e-20f )
	{
		return;
	}
	PrepareForceBuffer( sys );
	sys->force[index] = b2Add( sys->force[index], force );
}

void lfParticleSystem_ApplyLinearImpulse( lfParticleSystem* sys, int firstIndex, int lastIndex, b2Vec2 impulse )
{
	if ( sys == NULL || firstIndex < 0 || lastIndex > sys->count || firstIndex >= lastIndex )
	{
		return;
	}
	float numParticles = (float)( lastIndex - firstIndex );
	float totalMass = numParticles * sys->particleMass;
	if ( totalMass <= 0.0f )
	{
		return;
	}
	b2Vec2 velocityDelta = b2MulSV( 1.0f / totalMass, impulse );
	for ( int i = firstIndex; i < lastIndex; i++ )
	{
		if ( ( sys->flags[i] & lf_wallParticle ) != 0 )
		{
			continue;
		}
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[i], sys->velY[i] } ), velocityDelta ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
	}
}

void lfParticleSystem_ParticleApplyLinearImpulse( lfParticleSystem* sys, int index, b2Vec2 impulse )
{
	if ( sys == NULL || index < 0 || index >= sys->count )
	{
		return;
	}
	if ( ( sys->flags[index] & lf_wallParticle ) != 0 || sys->particleMass <= 0.0f )
	{
		return;
	}
	{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[index], sys->velY[index] } ), b2MulSV( sys->particleInvMass, impulse ) ); sys->velX[index] = __v.x; sys->velY[index] = __v.y; }
}

void lfParticleSystem_GroupApplyForce( lfParticleSystem* sys, lfParticleGroupId groupId, b2Vec2 force )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	lfParticleGroup* g = &sys->groups[groupId];
	lfParticleSystem_ApplyForce( sys, g->firstIndex, g->lastIndex, force );
}

void lfParticleSystem_GroupApplyLinearImpulse( lfParticleSystem* sys, lfParticleGroupId groupId, b2Vec2 impulse )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	lfParticleGroup* g = &sys->groups[groupId];
	lfParticleSystem_ApplyLinearImpulse( sys, g->firstIndex, g->lastIndex, impulse );
}

void lfParticleSystem_JoinParticleGroups( lfParticleSystem* sys, lfParticleGroupId groupA, lfParticleGroupId groupB )
{
	if ( !GroupIsValid( sys, groupA ) || !GroupIsValid( sys, groupB ) || groupA == groupB )
	{
		return;
	}
	lfParticleGroup* a = &sys->groups[groupA];
	lfParticleGroup* b = &sys->groups[groupB];

	// Move B to the end of the particle buffer, then A immediately before B.
	RotateBuffer( sys, b->firstIndex, b->lastIndex, sys->count );
	RotateBuffer( sys, a->firstIndex, a->lastIndex, b->firstIndex );

	for ( int i = b->firstIndex; i < b->lastIndex; i++ )
	{
		sys->groupIndex[i] = groupA;
	}
	a->groupFlags |= b->groupFlags;
	if ( ( a->groupFlags & lf_solidParticleGroup ) != 0 )
	{
		a->groupFlags |= lf_particleGroupNeedsUpdateDepth;
	}
	a->lastIndex = b->lastIndex;
	a->count = a->lastIndex - a->firstIndex;
	a->flags |= b->flags;
	if ( IsShapeGroupFlags( a->flags ) )
	{
		sys->hasShapeGroups = true;
	}
	sys->allGroupFlags |= a->groupFlags;

	b->firstIndex = b->lastIndex;
	b->count = 0;
	b->alive = false;
}

static int UfFind( int* parent, int x )
{
	while ( parent[x] != x )
	{
		parent[x] = parent[parent[x]];
		x = parent[x];
	}
	return x;
}

static void UfUnion( int* parent, int* rank, int a, int b )
{
	a = UfFind( parent, a );
	b = UfFind( parent, b );
	if ( a == b )
	{
		return;
	}
	if ( rank[a] < rank[b] )
	{
		parent[a] = b;
	}
	else if ( rank[a] > rank[b] )
	{
		parent[b] = a;
	}
	else
	{
		parent[b] = a;
		rank[a]++;
	}
}

void lfParticleSystem_SplitParticleGroup( lfParticleSystem* sys, lfParticleGroupId groupId )
{
	if ( !GroupIsValid( sys, groupId ) )
	{
		return;
	}
	lfParticleGroup* group = &sys->groups[groupId];
	int first = group->firstIndex;
	int last = group->lastIndex;
	int n = last - first;
	if ( n <= 1 )
	{
		return;
	}

	BuildGrid( sys );
	FindParticleContacts( sys );

	int* parent = (int*)malloc( (size_t)n * sizeof( int ) );
	int* rank = (int*)calloc( (size_t)n, sizeof( int ) );
	int* sizes = (int*)calloc( (size_t)n, sizeof( int ) );
	if ( parent == NULL || rank == NULL || sizes == NULL )
	{
		free( parent );
		free( rank );
		free( sizes );
		return;
	}
	for ( int i = 0; i < n; i++ )
	{
		parent[i] = i;
	}
	for ( int k = 0; k < sys->particleContactCount; k++ )
	{
		int a = sys->particleContacts[k].a;
		int b = sys->particleContacts[k].b;
		if ( a < first || a >= last || b < first || b >= last )
		{
			continue;
		}
		UfUnion( parent, rank, a - first, b - first );
	}
	for ( int i = 0; i < n; i++ )
	{
		sizes[UfFind( parent, i )]++;
	}
	int longest = 0;
	for ( int i = 1; i < n; i++ )
	{
		if ( sizes[UfFind( parent, i )] > sizes[UfFind( parent, longest )] )
		{
			longest = i;
		}
	}
	int longestRoot = UfFind( parent, longest );

	// Clone secondary components to the end, zombie originals (Google pattern).
	uint32_t savedFlags = group->flags;
	uint32_t savedGroupFlags = group->groupFlags;
	float savedStrength = group->strength;
	float savedViscous = group->viscousScale;

	for ( int root = 0; root < n; root++ )
	{
		if ( UfFind( parent, root ) != root || root == longestRoot || sizes[root] == 0 )
		{
			continue;
		}
		int start = sys->count;
		int created = 0;
		for ( int i = 0; i < n; i++ )
		{
			if ( UfFind( parent, i ) != root )
			{
				continue;
			}
			int src = first + i;
			lfParticleDef def = lfDefaultParticleDef();
			def.flags = sys->flags[src] & ~lf_zombieParticle;
			def.position = ( (b2Vec2){ sys->posX[src], sys->posY[src] } );
			def.velocity = ( (b2Vec2){ sys->velX[src], sys->velY[src] } );
			int dst = lfParticleSystem_CreateParticle( sys, &def );
			if ( dst < 0 )
			{
				break;
			}
			sys->restOffset[dst] = sys->restOffset[src];
			sys->viscousScale[dst] = sys->viscousScale[src];
			sys->remainingLife[dst] = sys->remainingLife[src];
			sys->totalLife[dst] = sys->totalLife[src];
			sys->renderAlpha[dst] = sys->renderAlpha[src];
			sys->flags[src] |= lf_zombieParticle;
			created++;
		}
		if ( created <= 0 )
		{
			continue;
		}
		int gid = AllocGroup( sys );
		if ( gid < 0 )
		{
			continue;
		}
		lfParticleGroupDef gdef = lfDefaultParticleGroupDef();
		gdef.flags = savedFlags;
		gdef.groupFlags = savedGroupFlags & ~lf_particleGroupInternalMask;
		gdef.strength = savedStrength;
		gdef.viscousScale = savedViscous;
		InitGroupFromRange( sys, gid, start, created, &gdef );
	}

	free( parent );
	free( rank );
	free( sizes );
}

static void RefreshAllGroupFlags( lfParticleSystem* sys )
{
	sys->allGroupFlags = 0;
	sys->hasShapeGroups = false;
	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive )
		{
			continue;
		}
		sys->allGroupFlags |= group->groupFlags;
		if ( IsShapeGroupFlags( group->flags ) )
		{
			sys->hasShapeGroups = true;
		}
	}
}

static void ComputeDepth( lfParticleSystem* sys )
{
	if ( ( sys->allGroupFlags & lf_particleGroupNeedsUpdateDepth ) == 0 )
	{
		return;
	}
	if ( sys->accumulation == NULL || sys->depth == NULL )
	{
		return;
	}

	int dirtySolidCount = 0;
	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive || ( group->groupFlags & lf_particleGroupNeedsUpdateDepth ) == 0 )
		{
			continue;
		}
		if ( ( group->groupFlags & lf_solidParticleGroup ) == 0 )
		{
			continue;
		}
		dirtySolidCount += group->lastIndex - group->firstIndex;
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			sys->accumulation[i] = 0.0f;
		}
	}

	int qualifyingCount = 0;
	if ( sys->staticPressureContactIndices != NULL )
	{
		for ( int k = 0; k < sys->particleContactCount; k++ )
		{
			const lfParticleContact* c = &sys->particleContacts[k];
			int a = c->a;
			int b = c->b;
			int ga = sys->groupIndex[a];
			if ( ga < 0 || ga != sys->groupIndex[b] )
			{
				continue;
			}
			lfParticleGroup* group = &sys->groups[ga];
			if ( !group->alive || ( group->groupFlags & lf_solidParticleGroup ) == 0 ||
				 ( group->groupFlags & lf_particleGroupNeedsUpdateDepth ) == 0 )
			{
				continue;
			}
			sys->accumulation[a] += c->weight;
			sys->accumulation[b] += c->weight;
			sys->staticPressureContactIndices[qualifyingCount++] = k;
		}
	}

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive || ( group->groupFlags & lf_solidParticleGroup ) == 0 ||
			 ( group->groupFlags & lf_particleGroupNeedsUpdateDepth ) == 0 )
		{
			continue;
		}
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			float w = sys->accumulation[i];
			sys->depth[i] = w < 0.8f ? 0.0f : FLT_MAX;
		}
	}

	int iterationCount = (int)sqrtf( (float)( dirtySolidCount > 0 ? dirtySolidCount : 1 ) );
	if ( iterationCount < 1 )
	{
		iterationCount = 1;
	}
	for ( int t = 0; t < iterationCount; t++ )
	{
		for ( int q = 0; q < qualifyingCount; q++ )
		{
			const lfParticleContact* c = &sys->particleContacts[sys->staticPressureContactIndices[q]];
			int a = c->a;
			int b = c->b;
			float r = 1.0f - c->weight;
			float da = sys->depth[a];
			float db = sys->depth[b];
			float targetA = db + r;
			float targetB = da + r;
			if ( targetA < da )
			{
				sys->depth[a] = targetA;
			}
			if ( targetB < db )
			{
				sys->depth[b] = targetB;
			}
		}
	}

	for ( int g = 0; g < sys->groupCount; g++ )
	{
		if ( sys->groups[g].alive )
		{
			sys->groups[g].groupFlags &= ~lf_particleGroupNeedsUpdateDepth;
		}
	}

	// needsUpdateDepth cleared on groups; drop it from the OR-mask so we do not
	// re-run this every substep for solid-only scenes (no UpdateGroupStatistics).
	RefreshAllGroupFlags( sys );
}

static void SolveSolid( lfParticleSystem* sys, float dt )
{
	if ( ( sys->allGroupFlags & lf_solidParticleGroup ) == 0 || dt <= 0.0f )
	{
		return;
	}
	float ejectionStrength = ( 1.0f / dt ) * sys->def.ejectionStrength;
	for ( int k = 0; k < sys->particleContactCount; k++ )
	{
		const lfParticleContact* c = &sys->particleContacts[k];
		int a = c->a;
		int b = c->b;
		if ( sys->groupIndex[a] == sys->groupIndex[b] )
		{
			continue;
		}
		float h = sys->depth[a] + sys->depth[b];
		if ( !( h > 0.0f ) || h > 1e20f )
		{
			continue;
		}
		b2Vec2 f = b2MulSV( ejectionStrength * h * c->weight, c->normal );
		{ b2Vec2 __v = b2Sub( ( (b2Vec2){ sys->velX[a], sys->velY[a] } ), f ); sys->velX[a] = __v.x; sys->velY[a] = __v.y; }
		{ b2Vec2 __v = b2Add( ( (b2Vec2){ sys->velX[b], sys->velY[b] } ), f ); sys->velX[b] = __v.x; sys->velY[b] = __v.y; }
	}
}

static void SolveRigid( lfParticleSystem* sys, float dt )
{
	if ( ( sys->allGroupFlags & lf_rigidParticleGroup ) == 0 || dt <= 0.0f )
	{
		return;
	}
	float invDt = 1.0f / dt;
	for ( int g = 0; g < sys->groupCount; g++ )
	{
		lfParticleGroup* group = &sys->groups[g];
		if ( !group->alive || ( group->groupFlags & lf_rigidParticleGroup ) == 0 )
		{
			continue;
		}
		float ang = dt * group->angularVelocity;
		float c = cosf( ang );
		float s = sinf( ang );
		b2Vec2 center = group->center;
		b2Vec2 lin = group->linearVelocity;
		// transform.p = center + dt*lin - R*center; velocityTransform maps pos -> vel
		b2Vec2 rotatedCenter = RotateOffset( c, s, center );
		b2Vec2 tp = b2Sub( b2Add( center, b2MulSV( dt, lin ) ), rotatedCenter );
		for ( int i = group->firstIndex; i < group->lastIndex; i++ )
		{
			b2Vec2 p = ( (b2Vec2){ sys->posX[i], sys->posY[i] } );
			b2Vec2 rp = RotateOffset( c, s, p );
			b2Vec2 world = b2Add( tp, rp );
			{ b2Vec2 __v = b2MulSV( invDt, b2Sub( world, p ) ); sys->velX[i] = __v.x; sys->velY[i] = __v.y; }
		}
	}
}

int lfParticleSystem_QueryAABB( lfParticleSystem* sys, b2AABB aabb, int* outIndices, int maxOut )
{
	if ( sys == NULL || outIndices == NULL || maxOut <= 0 || sys->count <= 0 )
	{
		return 0;
	}
	BuildGrid( sys );
	if ( sys->cellHead == NULL || sys->hashSize < 1 )
	{
		return 0;
	}

	float r = sys->def.radius;
	float invD = sys->invDiameter;
	int ix0 = (int)floorf( ( aabb.lowerBound.x - r ) * invD );
	int iy0 = (int)floorf( ( aabb.lowerBound.y - r ) * invD );
	int ix1 = (int)floorf( ( aabb.upperBound.x + r ) * invD );
	int iy1 = (int)floorf( ( aabb.upperBound.y + r ) * invD );

	int hits = 0;
	for ( int iy = iy0; iy <= iy1; iy++ )
	{
		for ( int ix = ix0; ix <= ix1; ix++ )
		{
			uint32_t cell = HashCell( ix, iy, sys->hashSize );
			for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
			{
				// Hash collisions: only accept particles whose cached cell matches.
				if ( sys->cellX[j] != ix || sys->cellY[j] != iy )
				{
					continue;
				}
				b2Vec2 p = ( (b2Vec2){ sys->posX[j], sys->posY[j] } );
				if ( p.x + r < aabb.lowerBound.x || p.x - r > aabb.upperBound.x || p.y + r < aabb.lowerBound.y ||
					 p.y - r > aabb.upperBound.y )
				{
					continue;
				}
				outIndices[hits++] = (int)j;
				if ( hits >= maxOut )
				{
					return hits;
				}
			}
		}
	}
	return hits;
}

int lfParticleSystem_RayCast( lfParticleSystem* sys, b2Vec2 point1, b2Vec2 point2, int* outIndices, int maxOut )
{
	if ( sys == NULL || outIndices == NULL || maxOut <= 0 || sys->count <= 0 )
	{
		return 0;
	}
	b2Vec2 v = b2Sub( point2, point1 );
	float v2 = b2LengthSquared( v );
	if ( v2 < 1e-20f )
	{
		return 0;
	}

	BuildGrid( sys );
	if ( sys->cellHead == NULL || sys->hashSize < 1 )
	{
		return 0;
	}

	float r = sys->def.radius;
	float invD = sys->invDiameter;
	float squaredDiameter = sys->diameter * sys->diameter;
	float minX = point1.x < point2.x ? point1.x : point2.x;
	float maxX = point1.x > point2.x ? point1.x : point2.x;
	float minY = point1.y < point2.y ? point1.y : point2.y;
	float maxY = point1.y > point2.y ? point1.y : point2.y;
	int ix0 = (int)floorf( ( minX - r ) * invD );
	int iy0 = (int)floorf( ( minY - r ) * invD );
	int ix1 = (int)floorf( ( maxX + r ) * invD );
	int iy1 = (int)floorf( ( maxY + r ) * invD );

	float fraction = 1.0f;
	int hits = 0;
	for ( int iy = iy0; iy <= iy1; iy++ )
	{
		for ( int ix = ix0; ix <= ix1; ix++ )
		{
			uint32_t cell = HashCell( ix, iy, sys->hashSize );
			for ( uint16_t j = sys->cellHead[cell]; j != LF_EMPTY_PARTICLE; j = sys->next[j] )
			{
				if ( sys->cellX[j] != ix || sys->cellY[j] != iy )
				{
					continue;
				}
				b2Vec2 p = b2Sub( point1, ( (b2Vec2){ sys->posX[j], sys->posY[j] } ) );
				float p2 = b2LengthSquared( p );
				if ( p2 <= squaredDiameter )
				{
					continue; // particle contains ray origin — Google ignores
				}
				float pv = b2Dot( p, v );
				float determinant = pv * pv - v2 * ( p2 - squaredDiameter );
				if ( determinant < 0.0f )
				{
					continue;
				}
				float sqrtDet = sqrtf( determinant );
				float t = ( -pv - sqrtDet ) / v2;
				if ( t > fraction )
				{
					continue;
				}
				if ( t < 0.0f )
				{
					t = ( -pv + sqrtDet ) / v2;
					if ( t < 0.0f || t > fraction )
					{
						continue;
					}
				}
				outIndices[hits++] = (int)j;
				fraction = t;
				if ( hits >= maxOut )
				{
					return hits;
				}
			}
		}
	}
	return hits;
}

const float* lfParticleSystem_GetWeightBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->weight : NULL;
}

void lfParticleSystem_Step( lfParticleSystem* sys, float dt, int subStepCount )
{
	if ( sys->count == 0 )
	{
		return;
	}
	if ( subStepCount < 1 )
	{
		subStepCount = 1;
	}

	SolveLifetime( sys, dt );
	SolveZombie( sys );
	if ( sys->count == 0 )
	{
		return;
	}

	if ( g_lfResetTasks != NULL )
	{
		g_lfResetTasks( g_lfTaskUser );
	}

	float subDt = dt / (float)subStepCount;

	for ( int step = 0; step < subStepCount; step++ )
	{
		BuildGrid( sys );
		FindParticleContacts( sys );
		// One shared broad-phase query for both FindBodyContacts (below) and
		// SolveCollision (later this sub-step) - swept-cloud AABB is a proven
		// superset of the static-cloud AABB FindBodyContacts alone would need.
		CollectOverlappingShapes( sys, ComputeSweptCloudAABB( sys, subDt, sys->diameter ) );
		FindBodyContacts( sys );
		if ( sys->def.strictContactCheck )
		{
			RemoveSpuriousBodyContacts( sys );
		}
		ComputeWeight( sys );
		ComputeDepth( sys );

		SolveForce( sys, subDt );
		SolveViscous( sys );
		SolvePowder( sys, subDt );
		SolveTensile( sys, subDt );
		if ( ( sys->allGroupFlags & lf_solidParticleGroup ) != 0 )
		{
			SolveSolid( sys, subDt );
		}
		SolveGravity( sys, subDt );
		SolveStaticPressure( sys, subDt );
		SolvePressure( sys, subDt );
		SolveDamping( sys, subDt );
		if ( sys->hasShapeGroups ||
			 ( sys->allGroupFlags & ( lf_rigidParticleGroup | lf_solidParticleGroup ) ) != 0 )
		{
			UpdateGroupStatistics( sys );
			SolveElastic( sys, subDt );
			SolveSpring( sys, subDt );
		}
		LimitVelocity( sys, subDt );
		SolveBarrier( sys, subDt );
		SolveCollision( sys, subDt );
		if ( ( sys->allGroupFlags & lf_rigidParticleGroup ) != 0 )
		{
			SolveRigid( sys, subDt );
		}
		SolveWall( sys );
		Integrate( sys, subDt );
	}

}

// ------------------------------------------------------------------------
// Accessors
// ------------------------------------------------------------------------

int lfParticleSystem_GetParticleCount( const lfParticleSystem* sys )
{
	return sys->count;
}

int lfParticleSystem_GetCapacity( const lfParticleSystem* sys )
{
	return sys->capacity;
}

float lfParticleSystem_GetRadius( const lfParticleSystem* sys )
{
	return sys->def.radius;
}

const float* lfParticleSystem_GetPositionXBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->posX : NULL;
}

const float* lfParticleSystem_GetPositionYBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->posY : NULL;
}

const float* lfParticleSystem_GetVelocityXBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->velX : NULL;
}

const float* lfParticleSystem_GetVelocityYBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->velY : NULL;
}

const uint32_t* lfParticleSystem_GetFlagsBuffer( const lfParticleSystem* sys )
{
	return sys->flags;
}

const float* lfParticleSystem_GetAlphaBuffer( const lfParticleSystem* sys )
{
	return sys->renderAlpha;
}

const float* lfParticleSystem_GetViscousScaleBuffer( const lfParticleSystem* sys )
{
	return sys->viscousScale;
}

const int* lfParticleSystem_GetGroupIndexBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->groupIndex : NULL;
}

const b2Vec2* lfParticleSystem_GetRestOffsetBuffer( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->restOffset : NULL;
}

int lfParticleSystem_GetPairCount( const lfParticleSystem* sys )
{
	return sys != NULL ? sys->pairCount : 0;
}

int lfParticleSystem_SyncActiveGroups( const lfParticleSystem* sys, int* idOut, int* countOut, int* firstOut,
										int* lastOut, float* viscOut, float* xOut, float* yOut, float* vxOut,
										float* vyOut, float* angVelOut, float* angleOut, int maxGroups )
{
	if ( sys == NULL || maxGroups <= 0 )
	{
		return 0;
	}
	int w = 0;
	for ( int gid = 0; gid < sys->groupCount && w < maxGroups; gid++ )
	{
		const lfParticleGroup* g = &sys->groups[gid];
		if ( !g->alive )
		{
			continue;
		}
		if ( idOut )
		{
			idOut[w] = gid;
		}
		if ( countOut )
		{
			countOut[w] = g->count;
		}
		if ( firstOut )
		{
			firstOut[w] = g->firstIndex;
		}
		if ( lastOut )
		{
			lastOut[w] = g->lastIndex;
		}
		if ( viscOut )
		{
			viscOut[w] = g->viscousScale;
		}
		if ( xOut )
		{
			xOut[w] = g->center.x;
		}
		if ( yOut )
		{
			yOut[w] = g->center.y;
		}
		if ( vxOut )
		{
			vxOut[w] = g->linearVelocity.x;
		}
		if ( vyOut )
		{
			vyOut[w] = g->linearVelocity.y;
		}
		if ( angVelOut )
		{
			angVelOut[w] = g->angularVelocity;
		}
		if ( angleOut )
		{
			angleOut[w] = g->angle;
		}
		w++;
	}
	return w;
}

int lfParticleSystem_CopyGroupSlots( const lfParticleSystem* sys, uint8_t* aliveOut, uint32_t* flagsOut,
									 uint32_t* groupFlagsOut, float* strengthOut, float* viscousScaleOut,
									 int* firstIndexOut, int* lastIndexOut, int maxSlots )
{
	if ( sys == NULL )
	{
		return 0;
	}
	int n = sys->groupCount;
	int write = n < maxSlots ? n : maxSlots;
	for ( int i = 0; i < write; i++ )
	{
		const lfParticleGroup* g = &sys->groups[i];
		if ( aliveOut )
		{
			aliveOut[i] = g->alive ? 1 : 0;
		}
		if ( flagsOut )
		{
			flagsOut[i] = g->flags;
		}
		if ( groupFlagsOut )
		{
			groupFlagsOut[i] = g->groupFlags;
		}
		if ( strengthOut )
		{
			strengthOut[i] = g->strength;
		}
		if ( viscousScaleOut )
		{
			viscousScaleOut[i] = g->viscousScale;
		}
		if ( firstIndexOut )
		{
			firstIndexOut[i] = g->firstIndex;
		}
		if ( lastIndexOut )
		{
			lastIndexOut[i] = g->lastIndex;
		}
	}
	return n;
}

int lfParticleSystem_CopyPairs( const lfParticleSystem* sys, uint16_t* aOut, uint16_t* bOut, uint32_t* flagsOut,
								float* distanceOut, float* strengthOut, int maxPairs )
{
	if ( sys == NULL )
	{
		return 0;
	}
	int n = sys->pairCount;
	int write = n < maxPairs ? n : maxPairs;
	for ( int i = 0; i < write; i++ )
	{
		const lfParticlePair* p = &sys->pairs[i];
		if ( aOut )
		{
			aOut[i] = p->a;
		}
		if ( bOut )
		{
			bOut[i] = p->b;
		}
		if ( flagsOut )
		{
			flagsOut[i] = p->flags;
		}
		if ( distanceOut )
		{
			distanceOut[i] = p->distance;
		}
		if ( strengthOut )
		{
			strengthOut[i] = p->strength;
		}
	}
	return n;
}

static bool EnsurePairCapacity( lfParticleSystem* sys, int minCapacity )
{
	if ( minCapacity <= sys->pairCapacity )
	{
		return true;
	}
	int newCapacity = sys->pairCapacity > 0 ? sys->pairCapacity : 256;
	while ( newCapacity < minCapacity )
	{
		newCapacity *= 2;
	}
	lfParticlePair* next =
		(lfParticlePair*)realloc( sys->pairs, (size_t)newCapacity * sizeof( lfParticlePair ) );
	if ( next == NULL )
	{
		return false;
	}
	sys->pairs = next;
	sys->pairCapacity = newCapacity;
	return true;
}

int lfParticleSystem_RestoreGroupsAndPairs( lfParticleSystem* sys, const int* groupIndex,
											const float* restOffsetXY, int groupSlotCount, const uint8_t* alive,
											const uint32_t* flags, const uint32_t* groupFlags, const float* strength,
											const float* viscousScale, const int* firstIndex, const int* lastIndex,
											int pairCount, const uint16_t* pairA, const uint16_t* pairB,
											const uint32_t* pairFlags, const float* pairDistance,
											const float* pairStrength )
{
	if ( sys == NULL )
	{
		return -1;
	}
	if ( groupSlotCount < 0 || pairCount < 0 )
	{
		return -2;
	}
	if ( groupSlotCount > 0 &&
		 ( alive == NULL || flags == NULL || groupFlags == NULL || strength == NULL || viscousScale == NULL ||
		   firstIndex == NULL || lastIndex == NULL ) )
	{
		return -3;
	}
	if ( sys->count > 0 && ( groupIndex == NULL || restOffsetXY == NULL ) )
	{
		return -4;
	}
	if ( pairCount > 0 &&
		 ( pairA == NULL || pairB == NULL || pairFlags == NULL || pairDistance == NULL || pairStrength == NULL ) )
	{
		return -5;
	}

	sys->flagOr = 0;
	for ( int i = 0; i < sys->count; i++ )
	{
		int gid = groupIndex[i];
		if ( gid != LF_NULL_PARTICLE_GROUP && ( gid < 0 || gid >= groupSlotCount ) )
		{
			return -6;
		}
		sys->groupIndex[i] = gid;
		sys->restOffset[i] = ( b2Vec2 ){ restOffsetXY[i * 2], restOffsetXY[i * 2 + 1] };
		sys->flagOr |= sys->flags[i];
	}

	if ( groupSlotCount > 0 && !EnsureGroupCapacity( sys, groupSlotCount ) )
	{
		return -7;
	}
	sys->groupCount = groupSlotCount;
	sys->allGroupFlags = 0;
	sys->hasShapeGroups = false;
	for ( int g = 0; g < groupSlotCount; g++ )
	{
		lfParticleGroup* slot = &sys->groups[g];
		memset( slot, 0, sizeof( *slot ) );
		slot->alive = alive[g] != 0;
		slot->flags = flags[g];
		slot->groupFlags = groupFlags[g];
		slot->strength = strength[g];
		slot->viscousScale = viscousScale[g] > 0.0f ? viscousScale[g] : 1.0f;
		slot->firstIndex = firstIndex[g];
		slot->lastIndex = lastIndex[g];
		slot->count = slot->lastIndex - slot->firstIndex;
		if ( !slot->alive )
		{
			continue;
		}
		if ( slot->firstIndex < 0 || slot->lastIndex < slot->firstIndex || slot->lastIndex > sys->count )
		{
			return -8;
		}
		sys->allGroupFlags |= slot->groupFlags;
		if ( IsShapeGroupFlags( slot->flags ) )
		{
			sys->hasShapeGroups = true;
		}
	}

	if ( pairCount > 0 && !EnsurePairCapacity( sys, pairCount ) )
	{
		return -9;
	}
	sys->pairCount = 0;
	for ( int k = 0; k < pairCount; k++ )
	{
		uint16_t a = pairA[k];
		uint16_t b = pairB[k];
		if ( (int)a >= sys->count || (int)b >= sys->count )
		{
			return -10;
		}
		lfParticlePair* p = &sys->pairs[sys->pairCount++];
		p->a = a;
		p->b = b;
		p->flags = pairFlags[k];
		p->distance = pairDistance[k];
		p->strength = pairStrength[k];
	}

	if ( sys->groupCount > 0 && sys->hasShapeGroups )
	{
		UpdateGroupStatistics( sys );
	}
	return 0;
}
