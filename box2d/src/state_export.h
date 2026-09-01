// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_functions.h"

#include <stdint.h>

/* WeedJS-shaped SoA: x, y, rotation(rad), vx, vy, angularVelocity, rotC, rotS */
#define B2_STATE_EXPORT_CHANNELS 8

enum b2WeedStateChannel
{
	b2_weed_x = 0,
	b2_weed_y = 1,
	b2_weed_rotation = 2,
	b2_weed_vx = 3,
	b2_weed_vy = 4,
	b2_weed_angular_velocity = 5,
	b2_weed_rot_c = 6,
	b2_weed_rot_s = 7,
};

void b2BindStateExportBuffer( float* buffer, int bodyCount );
void b2BindWeedExport( float* x, float* y, float* rotation, float* vx, float* vy, float* angularVelocity,
					   float* rotC, float* rotS, uint8_t* sleeping, int entityCapacity );
int b2GetStateExportCapacity( void );
void b2ExportBodyState( int entityIndex, b2WorldTransform transform, b2Vec2 linearVelocity, float angularVelocity );
void b2ExportBodySleeping( int entityIndex, int sleeping );
