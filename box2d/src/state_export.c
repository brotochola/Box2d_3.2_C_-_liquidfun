// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "state_export.h"

#include <stdlib.h>
#include <string.h>

static float* g_channels[B2_STATE_EXPORT_CHANNELS];
static uint8_t* g_sleeping = NULL;
static int g_capacity;
/* E-181: per slot, 1 when the last finalize export found the pose unchanged. */
static uint8_t* g_still = NULL;
static int g_still_capacity = 0;

/* WeedJS E-288: the accelerations bound for the next step (b2BindWeedAccelerations), taken once by its solver. */
static b2WeedAccelColumns g_accel;
static int g_accel_bound = 0;

void b2BindWeedAccelerations( const b2WeedAccelColumns* columns )
{
	g_accel = *columns;
	g_accel_bound = 1;
}

const b2WeedAccelColumns* b2TakeWeedAccelerations( void )
{
	if ( g_accel_bound == 0 )
	{
		return NULL;
	}
	g_accel_bound = 0;
	return &g_accel;
}

int b2GetStateExportCapacity( void )
{
	return g_capacity;
}

void b2BindWeedExport( float* x, float* y, float* rotation, float* vx, float* vy, float* angularVelocity,
					   float* rotC, float* rotS, uint8_t* sleeping, int entityCapacity )
{
	g_capacity = entityCapacity;
	g_channels[b2_weed_x] = x;
	g_channels[b2_weed_y] = y;
	g_channels[b2_weed_rotation] = rotation;
	g_channels[b2_weed_vx] = vx;
	g_channels[b2_weed_vy] = vy;
	g_channels[b2_weed_angular_velocity] = angularVelocity;
	g_channels[b2_weed_rot_c] = rotC;
	g_channels[b2_weed_rot_s] = rotS;
	g_sleeping = sleeping;
	if ( entityCapacity > g_still_capacity )
	{
		free( g_still );
		g_still = (uint8_t*)calloc( (size_t)entityCapacity, 1 );
		g_still_capacity = g_still != NULL ? entityCapacity : 0;
	}
	else if ( g_still != NULL )
	{
		memset( g_still, 0, (size_t)g_still_capacity );
	}
	if ( entityCapacity <= 0 )
	{
		for ( int i = 0; i < B2_STATE_EXPORT_CHANNELS; ++i )
		{
			g_channels[i] = NULL;
		}
		g_sleeping = NULL;
		g_capacity = 0;
	}
}

void b2BindStateExportBuffer( float* buffer, int bodyCount )
{
	if ( buffer == NULL || bodyCount <= 0 )
	{
		b2BindWeedExport( NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0 );
		return;
	}

	b2BindWeedExport( buffer + 0 * bodyCount, buffer + 1 * bodyCount, buffer + 2 * bodyCount, buffer + 3 * bodyCount,
					  buffer + 4 * bodyCount, buffer + 5 * bodyCount, buffer + 6 * bodyCount, buffer + 7 * bodyCount, NULL,
					  bodyCount );
}

/* WeedJS E-308: the step's export of a body that moved. One that moved without turning (a fixed rotation: Predator's
   people) keeps the rotation the channels hold, so its angle (an atan2) is not computed again and the three columns
   WeedJS's other workers read are not written with the values they hold (each write takes the line from a reader's
   cache). Only from the step: there the channels hold what the last export wrote, while a command (setRotationCS) may
   have written rotC and rotS for the export that follows it to complete with the angle. The caller has checked the
   channels and the index. */
static void b2ExportMovedBodyState( int entityIndex, b2WorldTransform transform, b2Vec2 linearVelocity, float angularVelocity )
{
	g_channels[b2_weed_x][entityIndex] = transform.p.x;
	g_channels[b2_weed_y][entityIndex] = transform.p.y;
	g_channels[b2_weed_vx][entityIndex] = linearVelocity.x;
	g_channels[b2_weed_vy][entityIndex] = linearVelocity.y;
	g_channels[b2_weed_angular_velocity][entityIndex] = angularVelocity;
	float* rotC = g_channels[b2_weed_rot_c];
	float* rotS = g_channels[b2_weed_rot_s];
	if ( rotC[entityIndex] != transform.q.c || rotS[entityIndex] != transform.q.s )
	{
		if ( g_channels[b2_weed_rotation] != NULL )
		{
			g_channels[b2_weed_rotation][entityIndex] = b2Atan2( transform.q.s, transform.q.c );
		}
		rotC[entityIndex] = transform.q.c;
		rotS[entityIndex] = transform.q.s;
	}
	b2ExportBodySleeping( entityIndex, 0 );
}

void b2ExportBodyStateIfMoved( int entityIndex, b2WorldTransform transform, b2Vec2 linearVelocity, float angularVelocity )
{
	if ( g_channels[b2_weed_x] == NULL || g_channels[b2_weed_rot_c] == NULL || g_channels[b2_weed_rot_s] == NULL ||
		 g_still == NULL || entityIndex < 0 || entityIndex >= g_capacity || entityIndex >= g_still_capacity )
	{
		b2ExportBodyState( entityIndex, transform, linearVelocity, angularVelocity );
		return;
	}

	/* The channels hold what the last export wrote (WeedJS reads them as Transform.x, y, rotC and rotS), and Box2D
	   finalizes every awake body every step, moved or not. */
	if ( g_channels[b2_weed_x][entityIndex] == (float)transform.p.x && g_channels[b2_weed_y][entityIndex] == (float)transform.p.y &&
		 g_channels[b2_weed_rot_c][entityIndex] == transform.q.c && g_channels[b2_weed_rot_s][entityIndex] == transform.q.s )
	{
		g_channels[b2_weed_vx][entityIndex] = linearVelocity.x;
		g_channels[b2_weed_vy][entityIndex] = linearVelocity.y;
		g_channels[b2_weed_angular_velocity][entityIndex] = angularVelocity;
		b2ExportBodySleeping( entityIndex, 0 );
		g_still[entityIndex] = 1;
		return;
	}

	g_still[entityIndex] = 0;
	b2ExportMovedBodyState( entityIndex, transform, linearVelocity, angularVelocity );
}

void b2ExportBodyVelocity( int entityIndex, b2Vec2 linearVelocity, float angularVelocity )
{
	if ( g_channels[b2_weed_vx] == NULL || entityIndex < 0 || entityIndex >= g_capacity )
	{
		return;
	}
	g_channels[b2_weed_vx][entityIndex] = linearVelocity.x;
	g_channels[b2_weed_vy][entityIndex] = linearVelocity.y;
	g_channels[b2_weed_angular_velocity][entityIndex] = angularVelocity;
}

int b2BodyPoseDiffers( int entityIndex, b2WorldTransform transform )
{
	if ( g_channels[b2_weed_x] == NULL || g_channels[b2_weed_rot_c] == NULL || g_channels[b2_weed_rot_s] == NULL ||
		 entityIndex < 0 || entityIndex >= g_capacity )
	{
		return 1;
	}
	return g_channels[b2_weed_x][entityIndex] != (float)transform.p.x || g_channels[b2_weed_y][entityIndex] != (float)transform.p.y ||
		   g_channels[b2_weed_rot_c][entityIndex] != transform.q.c || g_channels[b2_weed_rot_s][entityIndex] != transform.q.s;
}

int b2BodyStateWasStill( int entityIndex )
{
	if ( g_still == NULL || entityIndex < 0 || entityIndex >= g_still_capacity )
	{
		return 0;
	}
	return g_still[entityIndex];
}

void b2ExportBodySleeping( int entityIndex, int sleeping )
{
	if ( g_sleeping == NULL || entityIndex < 0 || entityIndex >= g_capacity )
	{
		return;
	}
	g_sleeping[entityIndex] = sleeping ? 1 : 0;
}

void b2ExportBodyState( int entityIndex, b2WorldTransform transform, b2Vec2 linearVelocity, float angularVelocity )
{
	if ( g_channels[b2_weed_x] == NULL || entityIndex < 0 || entityIndex >= g_capacity )
	{
		return;
	}

	g_channels[b2_weed_x][entityIndex] = transform.p.x;
	g_channels[b2_weed_y][entityIndex] = transform.p.y;
	/* Angle for JS API; facing for render stays in rotC/rotS. */
	if ( g_channels[b2_weed_rotation] != NULL )
	{
		g_channels[b2_weed_rotation][entityIndex] = b2Atan2( transform.q.s, transform.q.c );
	}
	g_channels[b2_weed_vx][entityIndex] = linearVelocity.x;
	g_channels[b2_weed_vy][entityIndex] = linearVelocity.y;
	g_channels[b2_weed_angular_velocity][entityIndex] = angularVelocity;
	if ( g_channels[b2_weed_rot_c] != NULL )
	{
		g_channels[b2_weed_rot_c][entityIndex] = transform.q.c;
	}
	if ( g_channels[b2_weed_rot_s] != NULL )
	{
		g_channels[b2_weed_rot_s][entityIndex] = transform.q.s;
	}
	b2ExportBodySleeping( entityIndex, 0 );
}
