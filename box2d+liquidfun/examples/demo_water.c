// liquidfun-c - examples/demo_water.c
//
// Headless demo: a Box2D 3 world with a ground, a ramp, and a light dynamic
// box, plus a block of "water" particles from liquidfun-c. Dumps a handful
// of CSV snapshots (particle positions + the dynamic box pose) so we can
// render them afterwards and see that:
//   1) the particle block falls and spreads out like a fluid,
//   2) particles collide with and settle against the ground/ramp,
//   3) the water measurably pushes the light floating box (two-way coupling).

#include "liquidfun/lf_particle_system.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <direct.h>
#define LF_MKDIR( p ) _mkdir( p )
#else
#include <sys/stat.h>
#define LF_MKDIR( p ) mkdir( p, 0755 )
#endif

static void DumpFrame( int frameIndex, lfParticleSystem* ps, b2BodyId boxId )
{
	char path[128];
	snprintf( path, sizeof( path ), "frames/frame_%03d.csv", frameIndex );
	FILE* f = fopen( path, "w" );
	if ( !f )
	{
		fprintf( stderr, "could not open %s\n", path );
		return;
	}

	b2Pos boxPos = b2Body_GetPosition( boxId );
	b2Rot boxRot = b2Body_GetRotation( boxId );
	float boxAngle = b2Rot_GetAngle( boxRot );
	fprintf( f, "box,%f,%f,%f\n", boxPos.x, boxPos.y, boxAngle );

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
		b2Polygon groundBox = b2MakeBox( 6.0f, 0.5f );
		b2ShapeDef groundShapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );
	}

	{
		b2BodyDef wallDef = b2DefaultBodyDef();
		b2ShapeDef wallShapeDef = b2DefaultShapeDef();
		b2Polygon wallBox = b2MakeBox( 0.2f, 5.0f );

		wallDef.position = ( b2Vec2 ){ -6.0f, 5.0f };
		b2BodyId leftWallId = b2CreateBody( worldId, &wallDef );
		b2CreatePolygonShape( leftWallId, &wallShapeDef, &wallBox );

		wallDef.position = ( b2Vec2 ){ 6.0f, 5.0f };
		b2BodyId rightWallId = b2CreateBody( worldId, &wallDef );
		b2CreatePolygonShape( rightWallId, &wallShapeDef, &wallBox );
	}

	{
		b2BodyDef rampDef = b2DefaultBodyDef();
		rampDef.position = ( b2Vec2 ){ -2.5f, 1.0f };
		rampDef.rotation = b2MakeRot( 0.35f );
		b2BodyId rampId = b2CreateBody( worldId, &rampDef );
		b2Polygon rampBox = b2MakeBox( 1.5f, 0.15f );
		b2ShapeDef rampShapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( rampId, &rampShapeDef, &rampBox );
	}

	b2BodyId boxId;
	{
		b2BodyDef boxDef = b2DefaultBodyDef();
		boxDef.type = b2_dynamicBody;
		boxDef.position = ( b2Vec2 ){ -1.8f, 0.8f };
		boxId = b2CreateBody( worldId, &boxDef );
		b2Polygon box = b2MakeBox( 0.3f, 0.3f );
		b2ShapeDef boxShapeDef = b2DefaultShapeDef();
		boxShapeDef.density = 0.3f;
		b2CreatePolygonShape( boxId, &boxShapeDef, &box );
	}

	lfParticleSystemDef psDef = lfDefaultParticleSystemDef();
	psDef.radius = 0.045f;
	psDef.maxParticles = 4096;
	lfParticleSystem* ps = lfParticleSystem_Create( worldId, &psDef );

	b2AABB block = { { -4.0f, 2.0f }, { -1.5f, 5.0f } };
	int n = lfParticleSystem_CreateParticleBox( ps, block, psDef.radius * 2.0f, lf_waterParticle, ( b2Vec2 ){ 0, 0 } );
	printf( "created %d water particles\n", n );

	LF_MKDIR( "frames" );

	const float dt = 1.0f / 60.0f;
	const int totalSteps = 360;
	int frame = 0;
	int dumpSteps[] = { 0, 15, 30, 60, 90, 150, 240, 359 };
	int dumpCount = (int)( sizeof( dumpSteps ) / sizeof( dumpSteps[0] ) );

	for ( int step = 0; step < totalSteps; step++ )
	{
		b2World_Step( worldId, dt, 4 );
		lfParticleSystem_Step( ps, dt, 2 );

		for ( int d = 0; d < dumpCount; d++ )
		{
			if ( dumpSteps[d] == step )
			{
				DumpFrame( frame++, ps, boxId );

				b2Pos bp = b2Body_GetPosition( boxId );
				int count = lfParticleSystem_GetParticleCount( ps );
				const b2Vec2* pos = lfParticleSystem_GetPositionBuffer( ps );
				float minY = 1e9f, maxY = -1e9f, avgY = 0.0f;
				float minX = 1e9f, maxX = -1e9f;
				for ( int i = 0; i < count; i++ )
				{
					if ( pos[i].y < minY )
						minY = pos[i].y;
					if ( pos[i].y > maxY )
						maxY = pos[i].y;
					if ( pos[i].x < minX )
						minX = pos[i].x;
					if ( pos[i].x > maxX )
						maxX = pos[i].x;
					avgY += pos[i].y;
				}
				avgY /= (float)count;

				printf( "step %3d | particles=%4d bounds x[%.2f,%.2f] y[%.2f,%.2f] avgY=%.2f | box=(%.2f,%.2f)\n",
						step, count, minX, maxX, minY, maxY, avgY, bp.x, bp.y );
			}
		}
	}

	int finalCount = lfParticleSystem_GetParticleCount( ps );
	lfParticleSystem_Destroy( ps );
	b2DestroyWorld( worldId );

	if ( finalCount != n )
	{
		fprintf( stderr, "FAIL: particle count %d != created %d\n", finalCount, n );
		return 1;
	}
	return 0;
}
