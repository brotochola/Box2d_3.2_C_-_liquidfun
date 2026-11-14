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
/* WeedJS E-181: b2ExportBodyState from the solver's finalize, except that a pose equal to the one the channels hold
   writes the velocities only and notes the body still (b2BodyStateWasStill) until its next export. */
void b2ExportBodyStateIfMoved( int entityIndex, b2WorldTransform transform, b2Vec2 linearVelocity, float angularVelocity );
/* 1 when the body's last finalize export found its pose unchanged. */
int b2BodyStateWasStill( int entityIndex );
/* WeedJS E-182: 1 when the pose differs from what the channels hold (or they are not bound). */
int b2BodyPoseDiffers( int entityIndex, b2WorldTransform transform );
/* WeedJS E-181: the velocity channels alone. */
void b2ExportBodyVelocity( int entityIndex, b2Vec2 linearVelocity, float angularVelocity );
void b2ExportBodySleeping( int entityIndex, int sleeping );

/* WeedJS E-288: logic's accelerations, the RigidBody columns of the slots (entity index = body userData). The weed glue
   binds them before a step (bodies_apply_accelerations); that step's first substep takes them and, in its velocity
   integration, adds to each awake body's force ax, ay times its mass (1 when not positive) and to its torque
   angularAccel times its inertia (its mass when not positive) unless its rotation is fixed: what one
   b2Body_ApplyForceToCenter and b2Body_ApplyTorque per body did. inertia and fixedRotation may be NULL. */
typedef struct b2WeedAccelColumns
{
	const float* ax;
	const float* ay;
	const float* angularAccel;
	const float* mass;
	const float* inertia;
	const uint8_t* fixedRotation;
	int capacity;
} b2WeedAccelColumns;

void b2BindWeedAccelerations( const b2WeedAccelColumns* columns );
/* The columns bound for the step about to run, or NULL; clears the binding, so one step applies them once. */
const b2WeedAccelColumns* b2TakeWeedAccelerations( void );
