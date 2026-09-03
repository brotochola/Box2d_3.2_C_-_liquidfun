// Headless check: SoA particles + shared Box2D job pool on FindContacts.
// Fails if count changes, COM explodes, or workers did not bind.

#include "liquidfun/lf_particle_system.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = ( b2Vec2 ){ 0.0f, -10.0f };
	worldDef.workerCount = 2;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	lfBindBox2dWorld( worldId );
	if ( lfGetWorkerCount() != 2 )
	{
		fprintf( stderr, "FAIL: lfGetWorkerCount=%d want 2\n", lfGetWorkerCount() );
		b2DestroyWorld( worldId );
		return 1;
	}

	{
		b2BodyDef groundDef = b2DefaultBodyDef();
		groundDef.position = ( b2Vec2 ){ 0.0f, -0.5f };
		b2BodyId groundId = b2CreateBody( worldId, &groundDef );
		b2Polygon groundBox = b2MakeBox( 8.0f, 0.5f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &groundBox );

		b2Polygon wall = b2MakeBox( 0.4f, 6.0f );
		b2BodyDef wallDef = b2DefaultBodyDef();
		wallDef.position = ( b2Vec2 ){ -7.5f, 5.0f };
		b2CreatePolygonShape( b2CreateBody( worldId, &wallDef ), &shapeDef, &wall );
		wallDef.position = ( b2Vec2 ){ 7.5f, 5.0f };
		b2CreatePolygonShape( b2CreateBody( worldId, &wallDef ), &shapeDef, &wall );
	}

	lfParticleSystemDef psDef = lfDefaultParticleSystemDef();
	psDef.radius = 0.05f;
	psDef.maxParticles = 16384;
	psDef.growable = false;
	lfParticleSystem* ps = lfParticleSystem_Create( worldId, &psDef );
	b2AABB block = { { -5.0f, 1.0f }, { 5.0f, 6.0f } };
	int n = lfParticleSystem_CreateParticleBox( ps, block, psDef.radius * 2.0f, lf_waterParticle, ( b2Vec2 ){ 0, 0 } );
	printf( "parallel demo: n=%d workers=%d\n", n, lfGetWorkerCount() );
	if ( n < 4096 )
	{
		fprintf( stderr, "FAIL: need >=4096 particles for parallel contacts, got %d\n", n );
		lfParticleSystem_Destroy( ps );
		b2DestroyWorld( worldId );
		return 1;
	}

	const float dt = 1.0f / 60.0f;
	for ( int step = 0; step < 90; step++ )
	{
		b2World_Step( worldId, dt, 4 );
		lfParticleSystem_Step( ps, dt, 2 );
	}

	int finalCount = lfParticleSystem_GetParticleCount( ps );
	const float* posX = lfParticleSystem_GetPositionXBuffer( ps );
	const float* posY = lfParticleSystem_GetPositionYBuffer( ps );
	float comX = 0.0f;
	float comY = 0.0f;
	for ( int i = 0; i < finalCount; i++ )
	{
		comX += posX[i];
		comY += posY[i];
	}
	comX /= (float)finalCount;
	comY /= (float)finalCount;
	lfParticleSystem_Destroy( ps );
	b2DestroyWorld( worldId );

	if ( finalCount != n )
	{
		fprintf( stderr, "FAIL: count %d != %d\n", finalCount, n );
		return 1;
	}
	if ( !isfinite( comX ) || !isfinite( comY ) )
	{
		fprintf( stderr, "FAIL: COM not finite\n" );
		return 1;
	}
	if ( comY < 0.2f || comY > 8.0f || comX < -7.0f || comX > 7.0f )
	{
		fprintf( stderr, "FAIL: COM exploded (%.2f, %.2f)\n", comX, comY );
		return 1;
	}
	printf( "parallel demo ok  com=(%.2f, %.2f)\n", comX, comY );
	return 0;
}
