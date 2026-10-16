// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "state_export.h"

static float* g_channels[B2_STATE_EXPORT_CHANNELS];
static uint8_t* g_sleeping = NULL;
static int g_capacity;

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
