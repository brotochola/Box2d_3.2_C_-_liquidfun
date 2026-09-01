// liquidfun-c - examples/demo_elastic.c
//
// Headless check: an elastic particle group (gelatin blob) falls onto a
// floor, bounces, and keeps its shape. Dumps CSV frames. Fails if the
// particle count changes or the COM leaves a sane range.

#include "liquidfun/lf_particle_system.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <direct.h>
#define LF_MKDIR( p ) _mkdir( p )
#else
#include <sys/stat.h>
#define LF_MKDIR( p ) mkdir( p, 0755 )
#endif

static const float kPi = 3.14159265f;

static void DumpFrame( int frameIndex, lfParticleSystem* ps, lfParticleGroupId group )
{
	char path[128];
	snprintf( path, sizeof( path ), "frames_elastic/frame_%03d.csv", frameIndex );
	FILE* f = fopen( path, "w" );
	if ( !f )
	{
		fprintf( stderr, "could not open %s\n", path );
		return;
	}

	b2Vec2 com = lfParticleSystem_GetGroupCenter( ps, group );
	float angle = lfParticleSystem_GetGroupAngle( ps, group );
	fprintf( f, "group,%f,%f,%f\n", com.x, com.y, angle );

	int count = lfParticleSystem_GetParticleCount( ps );
	const b2Vec2* pos = lfParticleSystem_GetPositionBuffer( ps );
	for ( int i = 0; i < count; i++ )
	{
		fprintf( f, "p,%f,%f\n", pos[i].x, pos[i].y );
	}
	fclose( f );
}

int main( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = ( b2Vec2 ){ 0.0f, -10.0f };
	b2WorldId worldId = b2CreateWorld( &worldDef );

	{
		b2BodyDef groundDef = b2DefaultBodyDef();
		groundDef.position = ( b2Vec2 ){ 0.0f, 0.0f };
		b2BodyId groundId = b2CreateBody( worldId, &groundDef );
		b2Polygon groundBox = b2MakeBox( 4.0f, 0.4f );
		b2ShapeDef groundShapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );
	}

	lfParticleSystemDef psDef = lfDefaultParticleSystemDef();
	psDef.radius = 0.05f;
	psDef.maxParticles = 2048;
	lfParticleSystem* ps = lfParticleSystem_Create( worldId, &psDef );

	lfParticleGroupDef gdef = lfDefaultParticleGroupDef();
	gdef.flags = lf_elasticParticle;
	gdef.position = ( b2Vec2 ){ 0.0f, 3.0f };
	gdef.radius = 0.6f;
	gdef.spacing = psDef.radius * 2.0f;
	gdef.strength = 0.4f;
	lfParticleGroupId group = lfParticleSystem_CreateParticleGroupCircle( ps, &gdef );
	int n = lfParticleSystem_GetParticleCount( ps );
	printf( "created elastic group %d with %d particles\n", (int)group, n );

	if ( n < 8 || group < 0 )
	{
		fprintf( stderr, "FAIL: expected a blob, got count=%d group=%d\n", n, (int)group );
		lfParticleSystem_Destroy( ps );
		b2DestroyWorld( worldId );
		return 1;
	}

	LF_MKDIR( "frames_elastic" );

	const float dt = 1.0f / 60.0f;
	const int totalSteps = 180;
	int dumpSteps[] = { 0, 20, 40, 80, 120, 179 };
	int dumpCount = (int)( sizeof( dumpSteps ) / sizeof( dumpSteps[0] ) );
	int frame = 0;

	for ( int step = 0; step < totalSteps; step++ )
	{
		b2World_Step( worldId, dt, 4 );
		lfParticleSystem_Step( ps, dt, 2 );

		for ( int d = 0; d < dumpCount; d++ )
		{
			if ( dumpSteps[d] == step )
			{
				DumpFrame( frame++, ps, group );
				b2Vec2 com = lfParticleSystem_GetGroupCenter( ps, group );
				float angleDeg = lfParticleSystem_GetGroupAngle( ps, group ) * 180.0f / kPi;
				int count = lfParticleSystem_GetParticleCount( ps );
				printf( "step %3d | particles=%4d com=(%.2f,%.2f) angle=%.1f deg\n", step, count, com.x, com.y,
						angleDeg );
			}
		}
	}

	int finalCount = lfParticleSystem_GetParticleCount( ps );
	b2Vec2 com = lfParticleSystem_GetGroupCenter( ps, group );
	lfParticleSystem_Destroy( ps );
	b2DestroyWorld( worldId );

	if ( finalCount != n )
	{
		fprintf( stderr, "FAIL: particle count %d != created %d\n", finalCount, n );
		return 1;
	}
	if ( !( com.y > 0.2f && com.y < 4.0f ) || !( com.x > -3.0f && com.x < 3.0f ) )
	{
		fprintf( stderr, "FAIL: COM exploded (%.2f, %.2f)\n", com.x, com.y );
		return 1;
	}
	if ( !isfinite( com.x ) || !isfinite( com.y ) )
	{
		fprintf( stderr, "FAIL: COM not finite\n" );
		return 1;
	}
	printf( "elastic demo ok\n" );
	return 0;
}
