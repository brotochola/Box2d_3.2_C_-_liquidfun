// WeedJS physics harness: a Predator-like crowd stepped by Box2D's own scheduler, in Node.
//
// N circle walkers (dynamic, no gravity, radius 8 px) on a field sized for a few contacts each, among static
// boxes (buildings), stepped at 60 Hz with 4 substeps and the WeedJS defaults (lengthUnitsPerMeter 100,
// contactHertz 30, damping ratio 0.7, contactSpeed 1000, maximumLinearSpeed 50000, sleeping on). Before each
// step every walker gets a velocity like logic sends: a heading that turns a little each step, from a seeded
// xorshift, so a run is the same sequence of inputs every time. The world uses Box2D's internal scheduler
// (no task callbacks), the one the WeedJS build runs, E-298's spin included.
//
// Output, one JSON line: the step times of the measured steps (mean, median, p95, after the warm-up), the mean
// of each b2Profile stage over those steps (so an A/B shows which stage moved), the counters of the last step, and a hash of every walker's transform bits after the last step. Two builds of
// the same inputs must print the same hash for a change that claims bit-identical results. The Weed build
// defines B2_COARSE_PROFILE (WeedJS E-235): only step, collide, solve (the finalize included), sleepIslands and
// sensors are timed there, and the finer stages read 0; a build without it fills them all.
//
// Args (all optional): bodies steps warmup workers seed
#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <emscripten.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t g_rng;

static uint32_t next_u32( void )
{
	g_rng ^= g_rng << 13;
	g_rng ^= g_rng >> 17;
	g_rng ^= g_rng << 5;
	return g_rng;
}

// [0, 1)
static float next_unit( void )
{
	return (float)( next_u32() >> 8 ) * ( 1.0f / 16777216.0f );
}

static int compare_double( const void* a, const void* b )
{
	double x = *(const double*)a, y = *(const double*)b;
	return ( x > y ) - ( x < y );
}

static uint64_t fnv1a( uint64_t h, const void* data, size_t n )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= p[i];
		h *= 1099511628211ull;
	}
	return h;
}

int main( int argc, char** argv )
{
	int bodyCount = argc > 1 ? atoi( argv[1] ) : 38000;
	int stepCount = argc > 2 ? atoi( argv[2] ) : 600;
	int warmup = argc > 3 ? atoi( argv[3] ) : 120;
	int workers = argc > 4 ? atoi( argv[4] ) : 4;
	g_rng = argc > 5 ? (uint32_t)strtoul( argv[5], NULL, 10 ) : 0x9e3779b9u;
	if ( g_rng == 0 )
		g_rng = 1;
	if ( warmup >= stepCount )
		warmup = stepCount / 2;

	const float radius = 8.0f;
	const float spacing = 40.0f; // field area per walker: (40 px)^2
	const float side = sqrtf( (float)bodyCount ) * spacing;
	const int buildingCount = bodyCount / 100;

	b2SetLengthUnitsPerMeter( 100.0f );
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, 0.0f };
	worldDef.contactHertz = 30.0f;
	worldDef.contactDampingRatio = 0.7f;
	worldDef.contactSpeed = 1000.0f;
	worldDef.maximumLinearSpeed = 50000.0f;
	worldDef.workerCount = workers;
	b2WorldId world = b2CreateWorld( &worldDef );
	b2World_EnableSleeping( world, true );

	// Buildings: static boxes 40 to 160 px a side.
	for ( int i = 0; i < buildingCount; ++i )
	{
		b2BodyDef def = b2DefaultBodyDef();
		def.type = b2_staticBody;
		def.position = (b2Pos){ next_unit() * side, next_unit() * side };
		b2BodyId body = b2CreateBody( world, &def );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 20.0f + 60.0f * next_unit(), 20.0f + 60.0f * next_unit() );
		b2CreatePolygonShape( body, &shapeDef, &box );
	}

	b2BodyId* walkers = (b2BodyId*)malloc( sizeof( b2BodyId ) * (size_t)bodyCount );
	float* heading = (float*)malloc( sizeof( float ) * (size_t)bodyCount );
	float* speed = (float*)malloc( sizeof( float ) * (size_t)bodyCount );
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2BodyDef def = b2DefaultBodyDef();
		def.type = b2_dynamicBody;
		def.position = (b2Pos){ next_unit() * side, next_unit() * side };
		def.motionLocks.angularZ = true; // walkers stay upright (fixedRotation in WeedJS)
		walkers[i] = b2CreateBody( world, &def );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		b2Circle circle = { { 0.0f, 0.0f }, radius };
		b2CreateCircleShape( walkers[i], &shapeDef, &circle );
		heading[i] = next_unit() * 6.2831853f;
		speed[i] = 30.0f + 30.0f * next_unit();
	}

	// b2Profile stages averaged over the measured steps (ms).
#define B2_HARNESS_STAGES( X )                                                                                         \
	X( pairs ) X( collide ) X( solve ) X( solverSetup ) X( prepareConstraints ) X( integrateVelocities ) X( warmStart )    \
		X( solveImpulses ) X( integratePositions ) X( relaxImpulses ) X( restitution ) X( storeImpulses ) X( splitIslands ) \
			X( transforms ) X( refit ) X( bullets ) X( sleepIslands )
#define B2_HARNESS_DECLARE( name ) double stage_##name = 0.0;
	B2_HARNESS_STAGES( B2_HARNESS_DECLARE )

	const float dt = 1.0f / 60.0f;
	int measured = stepCount - warmup;
	double* times = (double*)malloc( sizeof( double ) * (size_t)measured );
	for ( int step = 0; step < stepCount; ++step )
	{
		for ( int i = 0; i < bodyCount; ++i )
		{
			heading[i] += ( next_unit() - 0.5f ) * 0.2f;
			float h = heading[i];
			b2Body_SetLinearVelocity( walkers[i], (b2Vec2){ speed[i] * cosf( h ), speed[i] * sinf( h ) } );
		}
		double t0 = emscripten_get_now();
		b2World_Step( world, dt, 4 );
		double t1 = emscripten_get_now();
		if ( step >= warmup )
		{
			times[step - warmup] = t1 - t0;
			b2Profile profile = b2World_GetProfile( world );
#define B2_HARNESS_ADD( name ) stage_##name += profile.name;
			B2_HARNESS_STAGES( B2_HARNESS_ADD )
		}
	}

	double sum = 0.0;
	for ( int i = 0; i < measured; ++i )
		sum += times[i];
	qsort( times, (size_t)measured, sizeof( double ), compare_double );

	uint64_t hash = 1469598103934665603ull;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2WorldTransform t = b2Body_GetTransform( walkers[i] );
		b2Vec2 v = b2Body_GetLinearVelocity( walkers[i] );
		hash = fnv1a( hash, &t, sizeof( t ) );
		hash = fnv1a( hash, &v, sizeof( v ) );
	}

	printf( "{\"stages\":{" );
	{
		const char* separator = "";
#define B2_HARNESS_PRINT( name )                                                                                       \
	printf( "%s\"" #name "\":%.4f", separator, stage_##name / measured );                                               \
	separator = ",";
		B2_HARNESS_STAGES( B2_HARNESS_PRINT )
	}
	printf( "}," );

	b2Counters c = b2World_GetCounters( world );
	printf( "\"bodies\":%d,\"buildings\":%d,\"steps\":%d,\"warmup\":%d,\"workers\":%d,"
			"\"meanMs\":%.4f,\"medianMs\":%.4f,\"p95Ms\":%.4f,"
			"\"contacts\":%d,\"awakeContacts\":%d,\"recycledContacts\":%d,\"islands\":%d,"
			"\"poseHash\":\"%016llx\"}\n",
			bodyCount, buildingCount, stepCount, warmup, workers, sum / measured, times[measured / 2],
			times[( measured * 95 ) / 100], c.contactCount, c.awakeContactCount, c.recycledContactCount, c.islandCount,
			(unsigned long long)hash );

	b2DestroyWorld( world );
	free( walkers );
	free( heading );
	free( speed );
	free( times );
	return 0;
}
