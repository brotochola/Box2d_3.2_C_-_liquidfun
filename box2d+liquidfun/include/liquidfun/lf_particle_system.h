// liquidfun-c
// SPDX-License-Identifier: Zlib
//
// A from-scratch C reimplementation of the *core* ideas behind Google's
// LiquidFun particle system, built as an independent add-on module on top of
// Box2D 3.x's public C API (box2d/box2d.h). It does not use or depend on any
// LiquidFun source code - it only reproduces the same general simulation
// approach (weakly-compressible, grid-accelerated particle dynamics with
// two-way rigid body coupling), re-derived and re-implemented against Box2D
// 3's very different (data-oriented, id-based) architecture.
//
// Design summary (see ROADMAP.md for the long version):
//  - The particle system is *not* wired into Box2D's internal solver/graph.
//  - It is stepped once per frame, right after b2World_Step(), and it reads
//    rigid body/shape state exclusively through the public Box2D API
//    (b2Shape_GetPolygon/GetCircle/..., b2Body_GetMass, b2Body_ApplyLinearImpulse...).
//  - Coupling follows LiquidFun 1.1.0 Solve: signed body contacts,
//    pressure from criticalVelocity = diameter/dt, point SolveCollision
//    (velocity + linearSlop) then integrate. No PBD. Box2D 3 has no
//    ComputeDistance; liquidfun-c re-derives the per-shape signed distance
//    (closest face when inside) and uses b2Shape_RayCast for CCD.
//  - Indices in contacts/pairs/grid are uint16 (cap 65535, empty 0xFFFF).

#ifndef LF_PARTICLE_SYSTEM_H
#define LF_PARTICLE_SYSTEM_H

#include <box2d/box2d.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ----------------------------------------------------------------------
// Particle flags (subset of LiquidFun's b2ParticleFlag)
// ----------------------------------------------------------------------
typedef enum lfParticleFlag
{
	lf_waterParticle = 0,		  // default: normal fluid particle
	lf_zombieParticle = 1 << 0,	  // marked for removal at end of step
	lf_wallParticle = 1 << 1,	  // infinite mass, does not move (e.g. inlet/emitter dam)
	lf_viscousParticle = 1 << 2,  // extra tangential damping between neighbors
	lf_tensileParticle = 1 << 3,  // extra pull between neighbors near the free surface
	lf_elasticParticle = 1 << 4,  // shape-matching spring toward group rest pose
	lf_powderParticle = 1 << 5,	  // granular: extra repulsion when overlapping (sand)
	lf_springParticle = 1 << 6,	  // rest-length springs between neighbors at group create
	lf_barrierParticle = 1 << 7,  // with wall: zero vel; neighbor pairs form a segment dam
	lf_staticPressureParticle = 1 << 8, // extra Poisson pressure (does not vanish in a crack)
} lfParticleFlag;

// Group construction flags (Google b2ParticleGroupFlag). Separate from
// per-particle lfParticleFlag bits stamped onto members.
typedef enum lfParticleGroupFlag
{
	lf_solidParticleGroup = 1 << 0,			 // depth + inter-group ejection (SolveSolid)
	lf_rigidParticleGroup = 1 << 1,			 // SolveRigid: slab moves as a rigid body
	lf_particleGroupCanBeEmpty = 1 << 2,	 // keep group if emptied
	lf_particleGroupWillBeDestroyed = 1 << 3, // internal: destroy after zombie
	lf_particleGroupNeedsUpdateDepth = 1 << 4, // internal: recompute solid depth
	lf_particleGroupInternalMask = lf_particleGroupWillBeDestroyed | lf_particleGroupNeedsUpdateDepth,
} lfParticleGroupFlag;

// Hard cap: uint16 indices, empty sentinel 0xFFFF, live 0..65534.
#define LF_MAX_PARTICLES 65535
#define LF_EMPTY_PARTICLE ( (uint16_t)0xFFFF )

// Dense group slot. -1 means "no group".
typedef int32_t lfParticleGroupId;

#define LF_NULL_PARTICLE_GROUP ( (lfParticleGroupId) - 1 )

// ----------------------------------------------------------------------
// Tuning parameters. Defaults are reasonable for a world using meters and
// Box2D's usual gravity scale (~10 m/s^2) with a 1/60s step.
// ----------------------------------------------------------------------
typedef struct lfParticleSystemDef
{
	float radius;			 // particle radius (world units), default 0.05
	float density;			 // mass per unit area, default 1.0
	float dampingStrength;	 // normal-direction inter-particle damping [0,1], default 1.0
	float pressureStrength;	 // LF 1.1.0 default 0.05 (times critical pressure)
	float viscousStrength;	 // tangential damping for lf_viscousParticle, default 0.25
	float tensileStrength;	 // surface-tension pressure and normal, default 0.2
	float powderStrength;	 // extra overlap repulsion for lf_powderParticle, default 0.5
	float springStrength;	 // rest-length spring gain for lf_springParticle, default 0.25
	float staticPressureStrength;	 // Poisson static-pressure gain, default 0.2
	float staticPressureRelaxation; // Poisson diagonal, default 0.2
	int staticPressureIterations;	 // Poisson iters, default 8
	float ejectionStrength;	 // solid inter-group ejection (Google default 0.5)
	int maxParticles;		 // initial capacity (and hard cap when growable is false), default 2048
	bool growable;			 // if false, CreateParticle fails at maxParticles (required for WASM/SAB)
	bool strictContactCheck; // LF 1.1.0 default false; drop spurious floor+wall corners
} lfParticleSystemDef;

B2_API lfParticleSystemDef lfDefaultParticleSystemDef( void );

typedef struct lfParticleSystem lfParticleSystem;

// Create/destroy. The system does not own worldId; you must keep the Box2D
// world alive for the lifetime of the particle system and destroy the
// particle system before (or independently of) the world.
B2_API lfParticleSystem* lfParticleSystem_Create( b2WorldId worldId, const lfParticleSystemDef* def );
B2_API void lfParticleSystem_Destroy( lfParticleSystem* system );

// ----------------------------------------------------------------------
// Particle creation / destruction
// ----------------------------------------------------------------------
typedef struct lfParticleDef
{
	uint32_t flags;
	b2Vec2 position;
	b2Vec2 velocity;
} lfParticleDef;

B2_API lfParticleDef lfDefaultParticleDef( void );

// Returns the particle index, or -1 if at capacity and growth failed.
B2_API int lfParticleSystem_CreateParticle( lfParticleSystem* system, const lfParticleDef* def );

// Marks a particle for removal; it is actually removed at the end of the
// next lfParticleSystem_Step (indices you are holding become invalid then).
B2_API void lfParticleSystem_DestroyParticle( lfParticleSystem* system, int index );

// Immediately remove all particles and groups (no wait-for-step). Used by save restore.
// Does not destroy the system itself. Also clears elastic/spring pairs.
B2_API void lfParticleSystem_ClearParticles( lfParticleSystem* system );

// Convenience: fill an axis-aligned box with particles on a grid, spaced
// `spacing` apart (0 => 0.75 * diameter, Google b2_particleStride).
// Returns the number of particles created.
B2_API int lfParticleSystem_CreateParticleBox( lfParticleSystem* system, b2AABB box, float spacing, uint32_t flags,
											   b2Vec2 initialVelocity );

// ----------------------------------------------------------------------
// Particle groups (shape matching / elastic)
// ----------------------------------------------------------------------
typedef struct lfParticleGroupDef
{
	uint32_t flags;		  // stamped onto each member (include lf_elasticParticle for gelatin)
	uint32_t groupFlags;  // lfParticleGroupFlag (solid/rigid/…); not particle bits
	b2AABB box;			  // used by CreateParticleGroupBox
	b2Vec2 position;	  // circle center, used by CreateParticleGroupCircle
	float radius;		  // circle radius
	float spacing;		  // 0 => 0.75 * diameter (Google particle stride)
	b2Vec2 linearVelocity;
	float angularVelocity;
	float strength;		  // elastic stiffness in [0,1], default 0.5
	// Multiplier on system viscousStrength for members (default 1). Stamped
	// onto each particle at create. When != 1 (or trackGroup), a bookkeeping
	// group is kept without enabling shape-group stats/elastic.
	float viscousScale;
	// Force a bookkeeping group even when viscousScale == 1 (stable handle
	// for SetGroupViscousScale / group listing). No hasShapeGroups.
	int trackGroup;
	// Age-based destruction, matching Google LiquidFun's
	// SetParticleDestructionByAge/SetParticleLifetime design: each particle in
	// this group independently gets a random lifetime in [lifetimeMin,
	// lifetimeMax] seconds, counted down once per lfParticleSystem_Step (full
	// dt, not per-substep) and flagged lf_zombieParticle on expiry - swept up
	// by the same batch SolveZombie compaction as any other zombie.
	// lifetimeMax <= 0 (the zero-init default) disables tracking entirely.
	float lifetimeMin;
	float lifetimeMax;
	// When non-zero, render alpha lerps 1→0 over remaining life (opt-in).
	// Default 0: stay opaque until zombie destroy.
	int fadeToAlpha0;
} lfParticleGroupDef;

B2_API lfParticleGroupDef lfDefaultParticleGroupDef( void );

B2_API lfParticleGroupId lfParticleSystem_CreateParticleGroupBox( lfParticleSystem* system,
																  const lfParticleGroupDef* def );
B2_API lfParticleGroupId lfParticleSystem_CreateParticleGroupCircle( lfParticleSystem* system,
																	 const lfParticleGroupDef* def );
B2_API void lfParticleSystem_DestroyParticleGroup( lfParticleSystem* system, lfParticleGroupId groupId );

// Google Join: rotate slabs so B is contiguous after A, retarget B members to A, destroy B.
B2_API void lfParticleSystem_JoinParticleGroups( lfParticleSystem* system, lfParticleGroupId groupA,
												 lfParticleGroupId groupB );
// Google Split: connected components via current particle contacts; longest keeps groupA.
B2_API void lfParticleSystem_SplitParticleGroup( lfParticleSystem* system, lfParticleGroupId groupId );

B2_API int lfParticleSystem_GetGroupParticleCount( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API int lfParticleSystem_GetGroupFirstIndex( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API int lfParticleSystem_GetGroupLastIndex( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API uint32_t lfParticleSystem_GetGroupFlags( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API void lfParticleSystem_SetGroupFlags( lfParticleSystem* system, lfParticleGroupId groupId, uint32_t groupFlags );
B2_API b2Vec2 lfParticleSystem_GetGroupCenter( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API b2Vec2 lfParticleSystem_GetGroupLinearVelocity( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API float lfParticleSystem_GetGroupAngularVelocity( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API float lfParticleSystem_GetGroupAngle( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API float lfParticleSystem_GetGroupViscousScale( const lfParticleSystem* system, lfParticleGroupId groupId );
B2_API int lfParticleSystem_GetGroupSlotCount( const lfParticleSystem* system );
B2_API int lfParticleSystem_IsGroupAlive( const lfParticleSystem* system, lfParticleGroupId groupId );

// Stamp viscousScale onto every live member of the group (and the group field).
B2_API void lfParticleSystem_SetGroupViscousScale( lfParticleSystem* system, lfParticleGroupId groupId, float scale );

// Force / impulse (Google ApplyForce / ApplyLinearImpulse). Range is [first, last).
// Particle indices are invalid across SolveZombie / RotateBuffer / Split.
B2_API void lfParticleSystem_ApplyForce( lfParticleSystem* system, int firstIndex, int lastIndex, b2Vec2 force );
B2_API void lfParticleSystem_ParticleApplyForce( lfParticleSystem* system, int index, b2Vec2 force );
B2_API void lfParticleSystem_ApplyLinearImpulse( lfParticleSystem* system, int firstIndex, int lastIndex,
												 b2Vec2 impulse );
B2_API void lfParticleSystem_ParticleApplyLinearImpulse( lfParticleSystem* system, int index, b2Vec2 impulse );
B2_API void lfParticleSystem_GroupApplyForce( lfParticleSystem* system, lfParticleGroupId groupId, b2Vec2 force );
B2_API void lfParticleSystem_GroupApplyLinearImpulse( lfParticleSystem* system, lfParticleGroupId groupId,
													  b2Vec2 impulse );

// Gameplay queries (Google QueryAABB / RayCast). Rebuilds the particle grid
// (same as last-step contacts: document 1-frame staleness if called mid-step).
// Writes up to maxOut indices; returns hit count.
B2_API int lfParticleSystem_QueryAABB( lfParticleSystem* system, b2AABB aabb, int* outIndices, int maxOut );
B2_API int lfParticleSystem_RayCast( lfParticleSystem* system, b2Vec2 point1, b2Vec2 point2, int* outIndices,
									 int maxOut );

// Live-update system def strength coeffs (solvers read def each step).
B2_API void lfParticleSystem_SetTuning( lfParticleSystem* system, float dampingStrength, float pressureStrength,
										float viscousStrength, float tensileStrength, float powderStrength,
										float springStrength, float staticPressureStrength,
										float staticPressureRelaxation, int staticPressureIterations );

// ----------------------------------------------------------------------
// Simulation
// ----------------------------------------------------------------------
// Call this once per frame, AFTER b2World_Step() for the same dt, so that
// particle/body coupling reads settled rigid body transforms and applies
// impulses that will be picked up by the *next* b2World_Step. `subStepCount`
// controls the particle solver's own sub-stepping (independent of Box2D's),
// try 1-4.
B2_API void lfParticleSystem_Step( lfParticleSystem* system, float dt, int subStepCount );

// Optional: share Box2D's task callbacks so LF parallel-for uses the same worker
// threads. Call after b2CreateWorld (and after b2World_SetWorkerCount).
// resetTasks may be NULL; if set, it is invoked at the start of each LF parallel-for
// (Box2D built-in scheduler: pass a wrapper around b2ResetScheduler).
B2_API void lfSetTaskSystem( b2EnqueueTaskCallback* enqueue, b2FinishTaskCallback* finish, void* userContext,
							 int workerCount, void ( *resetTasks )( void* userContext ) );
B2_API int lfGetWorkerCount( void );
// Wire LF parallel-for to this world's Box2D scheduler (built-in or custom).
B2_API void lfBindBox2dWorld( b2WorldId worldId );

// ----------------------------------------------------------------------
// Accessors (read-only views into the internal SoA buffers).
// If the system was created with growable=false, these pointers are stable
// for the lifetime of the system (safe to bind as WASM/SAB views).
// If growable=true, they are invalidated by any allocation that grows capacity.
// ----------------------------------------------------------------------
B2_API int lfParticleSystem_GetParticleCount( const lfParticleSystem* system );
B2_API int lfParticleSystem_GetCapacity( const lfParticleSystem* system );
B2_API float lfParticleSystem_GetRadius( const lfParticleSystem* system );
B2_API const float* lfParticleSystem_GetPositionXBuffer( const lfParticleSystem* system );
B2_API const float* lfParticleSystem_GetPositionYBuffer( const lfParticleSystem* system );
B2_API const float* lfParticleSystem_GetVelocityXBuffer( const lfParticleSystem* system );
B2_API const float* lfParticleSystem_GetVelocityYBuffer( const lfParticleSystem* system );
B2_API const uint32_t* lfParticleSystem_GetFlagsBuffer( const lfParticleSystem* system );
// 1.0 for particles with no lifespan tracking, and for tracked particles
// created without fadeToAlpha0. When fadeToAlpha0 was set at create, ramps
// toward 0.0 as remaining life approaches zero (same idea as this engine's
// ParticleComponent.tweenToAlpha0). Recomputed every lfParticleSystem_Step
// by the same pass that ages/expires particles.
B2_API const float* lfParticleSystem_GetAlphaBuffer( const lfParticleSystem* system );
B2_API const float* lfParticleSystem_GetViscousScaleBuffer( const lfParticleSystem* system );
// Per-particle contact-weight density proxy (filled each sub-step in ComputeWeight).
B2_API const float* lfParticleSystem_GetWeightBuffer( const lfParticleSystem* system );

// Group membership / elastic rest pose (stable while growable=false).
B2_API const int* lfParticleSystem_GetGroupIndexBuffer( const lfParticleSystem* system );
B2_API const b2Vec2* lfParticleSystem_GetRestOffsetBuffer( const lfParticleSystem* system );
B2_API int lfParticleSystem_GetPairCount( const lfParticleSystem* system );

// Copy group slots (including dead holes so ids stay stable) into SoA outs.
// Writes min(slotCount, maxSlots); returns slot count available.
B2_API int lfParticleSystem_CopyGroupSlots( const lfParticleSystem* system, uint8_t* aliveOut, uint32_t* flagsOut,
											uint32_t* groupFlagsOut, float* strengthOut, float* viscousScaleOut,
											int* firstIndexOut, int* lastIndexOut, int maxSlots );

// Copy spring/barrier pairs into SoA outs. Writes min(pairCount, maxPairs); returns pair count.
B2_API int lfParticleSystem_CopyPairs( const lfParticleSystem* system, uint16_t* aOut, uint16_t* bOut,
									   uint32_t* flagsOut, float* distanceOut, float* strengthOut, int maxPairs );

// After particles exist (same indices as save), reinstall groups + restOffset + pairs.
// groupSlotCount may include dead slots. Returns 0 on success, negative on error.
B2_API int lfParticleSystem_RestoreGroupsAndPairs(
	lfParticleSystem* system, const int* groupIndex, const float* restOffsetXY, int groupSlotCount,
	const uint8_t* alive, const uint32_t* flags, const uint32_t* groupFlags, const float* strength,
	const float* viscousScale, const int* firstIndex, const int* lastIndex, int pairCount, const uint16_t* pairA,
	const uint16_t* pairB, const uint32_t* pairFlags, const float* pairDistance, const float* pairStrength );

#ifdef __cplusplus
}
#endif

#endif // LF_PARTICLE_SYSTEM_H
