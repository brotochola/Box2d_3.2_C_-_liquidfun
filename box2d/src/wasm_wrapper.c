// box2d/src/wasm_wrapper.c — physics glue in physics_post.js (post-js)
#include "box2d/box2d.h"
#include "state_export.h"
#include "liquidfun/lf_particle_system.h"

#include <emscripten.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BODIES 65535
#define MAX_PARTICLES 65535
#define MAX_JOINTS 4096
#define META_FLOATS 4
#define JOINT_FLOATS 8
#define MAX_QUERY_SLOTS 512
#define MAX_RAY_HITS 64
#define QUERY_HIT_FLOATS 8
#define MAX_CONTACT_EVENTS 10000
#define MAX_SENSOR_EVENTS 10000
#define MAX_CONTACT_HIT_EVENTS 128
#define MAX_JOINT_EVENTS 256
/* Matches emcc -sPTHREAD_POOL_SIZE; create_world clamps workerCount to this.
 * CMake may override via -DWASM_PTHREAD_POOL_SIZE=N. */
#ifndef WASM_PTHREAD_POOL_SIZE
#define WASM_PTHREAD_POOL_SIZE 4
#endif
#define MAX_MOVER_PLANES 32
#define MOVER_PLANE_FLOATS 8
#define EVENT_HEADER_INTS 11
#define CONTACT_PAIR_INTS 2

enum b2GameBodyType
{
	b2_game_static = 0,
	b2_game_dynamic = 1,
	b2_game_kinematic = 2,
};

enum b2GameShapeType
{
	b2_game_shape_box = 0,
	b2_game_shape_circle = 1,
	b2_game_shape_polygon = 2,
};

enum b2GameMetaFlags
{
	b2_game_meta_static = 1,
	b2_game_meta_disabled = 2,
};

enum b2GameJointFlags
{
	b2_game_joint_active = 1,
	b2_game_joint_disabled = 2,
};

typedef struct BodySlot
{
	b2BodyId id;
	uint8_t active;
	uint8_t shapeType;
	uint8_t bodyType;
} BodySlot;

typedef struct JointSlot
{
	b2JointId id;
	uint8_t active;
} JointSlot;

static uint32_t pack_world_id( b2WorldId id )
{
	uint32_t packed;
	memcpy( &packed, &id, sizeof( id ) );
	return packed;
}

static b2WorldId unpack_world_id( uint32_t packed )
{
	b2WorldId id;
	memcpy( &id, &packed, sizeof( id ) );
	return id;
}

static float* g_state_buffer = NULL;
static uint8_t* g_sleeping_buffer = NULL;
static float* g_meta_buffer = NULL;
static float* g_joint_buffer = NULL;
static int32_t* g_query_slots = NULL;
static float* g_query_hits = NULL;
static int32_t* g_event_header = NULL;
static int32_t* g_contact_begin = NULL;
static int32_t* g_contact_end = NULL;
static float* g_contact_hit = NULL;
static int32_t* g_sensor_begin = NULL;
static int32_t* g_sensor_end = NULL;
static int32_t* g_joint_events = NULL;
static float* g_mover_planes = NULL;
static int32_t* g_body_moved = NULL;
static uint8_t* g_body_fell_asleep = NULL;
static int g_body_move_count = 0;
static float g_profile[5];
static int32_t g_counters[7];
static BodySlot g_slots[MAX_BODIES];
static JointSlot g_joints[MAX_JOINTS];
static int g_free_stack[MAX_BODIES];
static int g_free_top = 0;
static int g_capacity = 0;
static int g_slot_high_water = 0;
static int g_next_slot = 0;
static int g_joint_high_water = 0;
static int g_next_joint = 0;
static int g_joint_free_stack[MAX_JOINTS];
static int g_joint_free_top = 0;
static int g_query_ray_count = 0;
static int g_mover_plane_count = 0;

static lfParticleSystem* g_particles = NULL;
static int32_t g_particle_count_value = 0;
static int g_particle_capacity = 0;
static int g_particle_sub_steps = 2;
/* Deinterleaved copy of the particle position buffer (liquidfun-c stores
 * b2Vec2 x,y,x,y,... contiguously). Filled once per step_world from a tight C
 * loop so weedjs_post.js can do two bulk TypedArray .set() calls instead of a
 * scalar per-particle loop reading interleaved floats out of Module.HEAPF32. */
static float* g_particle_x = NULL;
static float* g_particle_y = NULL;
/* lfParticleSystem_GetAlphaBuffer, copied out the same way - fade-to-0 only
 * when lfParticleGroupDef.fadeToAlpha0 was set; otherwise always 1.0. */
static float* g_particle_alpha = NULL;
static float g_liquidfun_step_ms = 0.0f;

enum EventHeaderIndex
{
	b2_event_overlap_count = 0,
	b2_event_ray_hit_count = 1,
	b2_event_contact_begin_count = 2,
	b2_event_contact_end_count = 3,
	b2_event_contact_hit_count = 4,
	b2_event_sensor_begin_count = 5,
	b2_event_sensor_end_count = 6,
	b2_event_mover_plane_count = 7,
	b2_event_contact_dropped_count = 8,
	b2_event_sensor_dropped_count = 9,
	b2_event_joint_event_count = 10,
};

static uint64_t normalize_mask_bits( uint32_t maskBits )
{
	return maskBits == 0 ? B2_DEFAULT_MASK_BITS : (uint64_t)maskBits;
}

static uint64_t normalize_category_bits( uint32_t categoryBits )
{
	return categoryBits == 0 ? (uint64_t)B2_DEFAULT_CATEGORY_BITS : (uint64_t)categoryBits;
}

static b2QueryFilter make_query_filter( uint32_t categoryBits, uint32_t maskBits )
{
	b2QueryFilter filter;
	filter.categoryBits = normalize_category_bits( categoryBits );
	filter.maskBits = normalize_mask_bits( maskBits );
	return filter;
}

static int slot_from_body( b2BodyId bodyId )
{
	if ( !b2Body_IsValid( bodyId ) )
	{
		return -1;
	}

	intptr_t slot = (intptr_t)b2Body_GetUserData( bodyId );
	if ( slot < 0 || slot >= g_capacity )
	{
		return -1;
	}

	return (int)slot;
}

static int slot_from_shape( b2ShapeId shapeId )
{
	if ( !b2Shape_IsValid( shapeId ) )
	{
		return -1;
	}

	b2BodyId bodyId = b2Shape_GetBody( shapeId );
	if ( !b2Body_IsValid( bodyId ) )
	{
		return -1;
	}

	intptr_t slot = (intptr_t)b2Body_GetUserData( bodyId );
	if ( slot < 0 || slot >= g_capacity )
	{
		return -1;
	}

	return (int)slot;
}

static b2ShapeId first_shape( b2BodyId bodyId )
{
	b2ShapeId shapes[1];
	int count = b2Body_GetShapes( bodyId, shapes, 1 );
	if ( count <= 0 )
	{
		return b2_nullShapeId;
	}
	return shapes[0];
}

static void write_event_header_count( int index, int value )
{
	if ( g_event_header != NULL && index >= 0 && index < EVENT_HEADER_INTS )
	{
		g_event_header[index] = value;
	}
}

static void clear_event_counts( void )
{
	if ( g_event_header == NULL )
	{
		return;
	}

	memset( g_event_header, 0, EVENT_HEADER_INTS * sizeof( int32_t ) );
}

typedef struct OverlapCollectCtx
{
	int32_t* out;
	int capacity;
	int count;
} OverlapCollectCtx;

static bool overlap_collect_callback( b2ShapeId shapeId, void* context )
{
	OverlapCollectCtx* ctx = (OverlapCollectCtx*)context;
	if ( ctx == NULL || ctx->out == NULL || ctx->count >= ctx->capacity )
	{
		return true;
	}

	int slot = slot_from_shape( shapeId );
	if ( slot < 0 )
	{
		return true;
	}

	ctx->out[ctx->count++] = slot;
	return true;
}

static float ray_collect_callback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	(void)context;
	if ( g_query_hits == NULL || g_query_ray_count >= MAX_RAY_HITS )
	{
		return 1.0f;
	}

	float* hit = g_query_hits + g_query_ray_count * QUERY_HIT_FLOATS;
	hit[0] = (float)slot_from_shape( shapeId );
	hit[1] = fraction;
	hit[2] = (float)point.x;
	hit[3] = (float)point.y;
	hit[4] = normal.x;
	hit[5] = normal.y;
	hit[6] = 0.0f;
	hit[7] = 0.0f;
	g_query_ray_count++;
	return 1.0f;
}

static bool mover_plane_callback( b2ShapeId shapeId, const b2PlaneResult* plane, void* context )
{
	(void)shapeId;
	(void)context;
	if ( g_mover_planes == NULL || g_mover_plane_count >= MAX_MOVER_PLANES || plane == NULL || !plane->hit )
	{
		return true;
	}

	float* row = g_mover_planes + g_mover_plane_count * MOVER_PLANE_FLOATS;
	row[0] = plane->plane.normal.x;
	row[1] = plane->plane.normal.y;
	row[2] = plane->plane.offset;
	row[3] = plane->point.x;
	row[4] = plane->point.y;
	row[5] = 1.0f;
	row[6] = 0.0f;
	row[7] = 0.0f;
	g_mover_plane_count++;
	return true;
}

static void export_contact_events( b2WorldId worldId )
{
	if ( g_contact_begin == NULL || g_contact_end == NULL || g_contact_hit == NULL )
	{
		return;
	}

	b2ContactEvents events = b2World_GetContactEvents( worldId );
	int beginCount = 0;
	int endCount = 0;
	int hitCount = 0;
	int droppedCount = 0;

	for ( int i = 0; i < events.beginCount; ++i )
	{
		int slotA = slot_from_shape( events.beginEvents[i].shapeIdA );
		int slotB = slot_from_shape( events.beginEvents[i].shapeIdB );
		if ( slotA < 0 || slotB < 0 )
		{
			continue;
		}
		if ( beginCount >= MAX_CONTACT_EVENTS )
		{
			droppedCount++;
			continue;
		}
		g_contact_begin[beginCount * CONTACT_PAIR_INTS] = slotA;
		g_contact_begin[beginCount * CONTACT_PAIR_INTS + 1] = slotB;
		beginCount++;
	}

	for ( int i = 0; i < events.endCount; ++i )
	{
		b2ShapeId shapeA = events.endEvents[i].shapeIdA;
		b2ShapeId shapeB = events.endEvents[i].shapeIdB;
		if ( !b2Shape_IsValid( shapeA ) || !b2Shape_IsValid( shapeB ) )
		{
			continue;
		}
		int slotA = slot_from_shape( shapeA );
		int slotB = slot_from_shape( shapeB );
		if ( slotA < 0 || slotB < 0 )
		{
			continue;
		}
		if ( endCount >= MAX_CONTACT_EVENTS )
		{
			droppedCount++;
			continue;
		}
		g_contact_end[endCount * CONTACT_PAIR_INTS] = slotA;
		g_contact_end[endCount * CONTACT_PAIR_INTS + 1] = slotB;
		endCount++;
	}

	for ( int i = 0; i < events.hitCount && hitCount < MAX_CONTACT_HIT_EVENTS; ++i )
	{
		int slotA = slot_from_shape( events.hitEvents[i].shapeIdA );
		int slotB = slot_from_shape( events.hitEvents[i].shapeIdB );
		if ( slotA < 0 || slotB < 0 )
		{
			continue;
		}
		float* row = g_contact_hit + hitCount * QUERY_HIT_FLOATS;
		row[0] = (float)slotA;
		row[1] = (float)slotB;
		row[2] = (float)events.hitEvents[i].point.x;
		row[3] = (float)events.hitEvents[i].point.y;
		row[4] = events.hitEvents[i].normal.x;
		row[5] = events.hitEvents[i].normal.y;
		row[6] = events.hitEvents[i].approachSpeed;
		row[7] = 0.0f;
		hitCount++;
	}

	write_event_header_count( b2_event_contact_begin_count, beginCount );
	write_event_header_count( b2_event_contact_end_count, endCount );
	write_event_header_count( b2_event_contact_hit_count, hitCount );
	write_event_header_count( b2_event_contact_dropped_count, droppedCount );
}

static void export_sensor_events( b2WorldId worldId )
{
	if ( g_sensor_begin == NULL || g_sensor_end == NULL )
	{
		return;
	}

	b2SensorEvents events = b2World_GetSensorEvents( worldId );
	int beginCount = 0;
	int endCount = 0;
	int droppedCount = 0;

	for ( int i = 0; i < events.beginCount; ++i )
	{
		int sensorSlot = slot_from_shape( events.beginEvents[i].sensorShapeId );
		int visitorSlot = slot_from_shape( events.beginEvents[i].visitorShapeId );
		if ( sensorSlot < 0 || visitorSlot < 0 )
		{
			continue;
		}
		if ( beginCount >= MAX_SENSOR_EVENTS )
		{
			droppedCount++;
			continue;
		}
		g_sensor_begin[beginCount * CONTACT_PAIR_INTS] = sensorSlot;
		g_sensor_begin[beginCount * CONTACT_PAIR_INTS + 1] = visitorSlot;
		beginCount++;
	}

	for ( int i = 0; i < events.endCount; ++i )
	{
		b2ShapeId sensorShape = events.endEvents[i].sensorShapeId;
		b2ShapeId visitorShape = events.endEvents[i].visitorShapeId;
		if ( !b2Shape_IsValid( sensorShape ) || !b2Shape_IsValid( visitorShape ) )
		{
			continue;
		}
		int sensorSlot = slot_from_shape( sensorShape );
		int visitorSlot = slot_from_shape( visitorShape );
		if ( sensorSlot < 0 || visitorSlot < 0 )
		{
			continue;
		}
		if ( endCount >= MAX_SENSOR_EVENTS )
		{
			droppedCount++;
			continue;
		}
		g_sensor_end[endCount * CONTACT_PAIR_INTS] = sensorSlot;
		g_sensor_end[endCount * CONTACT_PAIR_INTS + 1] = visitorSlot;
		endCount++;
	}

	write_event_header_count( b2_event_sensor_begin_count, beginCount );
	write_event_header_count( b2_event_sensor_end_count, endCount );
	write_event_header_count( b2_event_sensor_dropped_count, droppedCount );
}

/* userData on joints is Weed joint index (set via joint_configure). */
static void export_joint_events( b2WorldId worldId )
{
	if ( g_joint_events == NULL )
	{
		return;
	}

	b2JointEvents events = b2World_GetJointEvents( worldId );
	int count = 0;
	for ( int i = 0; i < events.count && count < MAX_JOINT_EVENTS; ++i )
	{
		intptr_t weedIdx = (intptr_t)events.jointEvents[i].userData;
		if ( weedIdx < 0 )
		{
			continue;
		}
		g_joint_events[count++] = (int32_t)weedIdx;
	}
	write_event_header_count( b2_event_joint_event_count, count );
}

static void export_body_move_events( b2WorldId worldId )
{
	g_body_move_count = 0;
	if ( g_body_moved == NULL || g_body_fell_asleep == NULL || g_capacity <= 0 )
	{
		return;
	}

	b2BodyEvents events = b2World_GetBodyEvents( worldId );
	int count = 0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		if ( count >= g_capacity )
		{
			break;
		}

		int slot = -1;
		intptr_t fromEvent = (intptr_t)events.moveEvents[i].userData;
		if ( fromEvent >= 0 && fromEvent < g_capacity )
		{
			slot = (int)fromEvent;
		}
		else
		{
			slot = slot_from_body( events.moveEvents[i].bodyId );
		}
		if ( slot < 0 )
		{
			continue;
		}

		g_body_moved[count] = slot;
		g_body_fell_asleep[count] = events.moveEvents[i].fellAsleep ? 1 : 0;
		count++;
	}
	g_body_move_count = count;
}

static void export_profile_and_counters( b2WorldId worldId )
{
	b2Profile profile = b2World_GetProfile( worldId );
	g_profile[0] = profile.step;
	g_profile[1] = profile.collide;
	g_profile[2] = profile.solve;
	g_profile[3] = profile.sleepIslands;
	g_profile[4] = profile.sensors;

	b2Counters counters = b2World_GetCounters( worldId );
	g_counters[0] = counters.bodyCount;
	g_counters[1] = counters.shapeCount;
	g_counters[2] = counters.contactCount;
	g_counters[3] = counters.jointCount;
	g_counters[4] = counters.islandCount;
	g_counters[5] = counters.awakeContactCount;
	g_counters[6] = counters.treeHeight;
}

static b2BodyType to_b2_body_type( int type )
{
	switch ( type )
	{
		case b2_game_static:
			return b2_staticBody;
		case b2_game_kinematic:
			return b2_kinematicBody;
		default:
			return b2_dynamicBody;
	}
}

static void write_meta( int slot, int shapeType, float halfW, float halfH, int flags )
{
	if ( g_meta_buffer == NULL || slot < 0 || slot >= g_capacity )
	{
		return;
	}

	float* meta = g_meta_buffer + slot * META_FLOATS;
	meta[0] = (float)shapeType;
	meta[1] = halfW;
	meta[2] = halfH;
	meta[3] = (float)flags;
}

static int claim_entity_slot( int entity )
{
	if ( entity < 0 || entity >= g_capacity || g_slots[entity].active )
	{
		return -1;
	}
	for ( int i = 0; i < g_free_top; ++i )
	{
		if ( g_free_stack[i] == entity )
		{
			g_free_stack[i] = g_free_stack[--g_free_top];
			break;
		}
	}
	if ( entity >= g_next_slot )
	{
		g_next_slot = entity + 1;
	}
	return entity;
}

static int alloc_slot( void )
{
	if ( g_free_top > 0 )
	{
		return g_free_stack[--g_free_top];
	}
	if ( g_next_slot >= g_capacity )
	{
		return -1;
	}
	return g_next_slot++;
}

static void free_slot( int slot )
{
	if ( slot < 0 || slot >= g_capacity || g_free_top >= MAX_BODIES )
	{
		return;
	}
	g_free_stack[g_free_top++] = slot;
}

static void mark_active_dynamics_sleeping( void )
{
	if ( g_sleeping_buffer == NULL )
	{
		return;
	}
	for ( int slot = 0; slot < g_slot_high_water; ++slot )
	{
		if ( g_slots[slot].active && g_slots[slot].bodyType == b2_game_dynamic )
		{
			g_sleeping_buffer[slot] = 1;
		}
	}
}

static int alloc_joint_slot( void )
{
	if ( g_joint_free_top > 0 )
	{
		return g_joint_free_stack[--g_joint_free_top];
	}
	if ( g_next_joint >= MAX_JOINTS )
	{
		return -1;
	}
	return g_next_joint++;
}

static void free_joint_slot( int handle )
{
	if ( handle < 0 || handle >= MAX_JOINTS || g_joint_free_top >= MAX_JOINTS )
	{
		return;
	}
	g_joint_free_stack[g_joint_free_top++] = handle;
}

static void reset_joint_table( void )
{
	memset( g_joints, 0, sizeof( g_joints ) );
	g_joint_high_water = 0;
	g_next_joint = 0;
	g_joint_free_top = 0;
}

static void write_joint_meta( int handle, int jointType, int flags, float ax, float ay, float bx, float by, float rotC,
							  float rotS )
{
	if ( g_joint_buffer == NULL || handle < 0 || handle >= MAX_JOINTS )
	{
		return;
	}

	float* joint = g_joint_buffer + handle * JOINT_FLOATS;
	joint[0] = (float)jointType;
	joint[1] = (float)flags;
	joint[2] = ax;
	joint[3] = ay;
	joint[4] = bx;
	joint[5] = by;
	joint[6] = rotC;
	joint[7] = rotS;
}

static void export_joint_state( int handle )
{
	if ( handle < 0 || handle >= MAX_JOINTS || !g_joints[handle].active )
	{
		write_joint_meta( handle, 0, b2_game_joint_disabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
		return;
	}

	b2JointId jointId = g_joints[handle].id;
	b2BodyId bodyA = b2Joint_GetBodyA( jointId );
	b2BodyId bodyB = b2Joint_GetBodyB( jointId );
	b2WorldTransform xfA = b2Body_GetTransform( bodyA );
	b2WorldTransform xfB = b2Body_GetTransform( bodyB );
	b2WorldTransform frameA = b2OffsetWorldTransform( xfA, b2Joint_GetLocalFrameA( jointId ) );
	b2WorldTransform frameB = b2OffsetWorldTransform( xfB, b2Joint_GetLocalFrameB( jointId ) );

	write_joint_meta( handle, (int)b2Joint_GetType( jointId ), b2_game_joint_active, (float)frameA.p.x, (float)frameA.p.y,
					  (float)frameB.p.x, (float)frameB.p.y, frameA.q.c, frameA.q.s );
}

static void export_all_joints( void )
{
	for ( int handle = 0; handle < g_joint_high_water; ++handle )
	{
		export_joint_state( handle );
	}
}

static void set_joint_anchors( b2JointDef* jointDef, b2BodyId bodyA, b2BodyId bodyB, float anchorX, float anchorY )
{
	b2Pos pivot = { anchorX, anchorY };
	jointDef->bodyIdA = bodyA;
	jointDef->bodyIdB = bodyB;
	jointDef->localFrameA.p = b2Body_GetLocalPoint( bodyA, pivot );
	jointDef->localFrameB.p = b2Body_GetLocalPoint( bodyB, pivot );
	jointDef->localFrameA.q = b2Rot_identity;
	jointDef->localFrameB.q = b2Rot_identity;
}

static void set_joint_local_anchors( b2JointDef* jointDef, b2BodyId bodyA, b2BodyId bodyB, float lax, float lay,
									 float lbx, float lby )
{
	jointDef->bodyIdA = bodyA;
	jointDef->bodyIdB = bodyB;
	jointDef->localFrameA.p = (b2Vec2){ lax, lay };
	jointDef->localFrameB.p = (b2Vec2){ lbx, lby };
	jointDef->localFrameA.q = b2Rot_identity;
	jointDef->localFrameB.q = b2Rot_identity;
}

static void capture_weld_relative_rotation( b2JointDef* jointDef, b2BodyId bodyA, b2BodyId bodyB )
{
	b2Rot qA = b2Body_GetRotation( bodyA );
	b2Rot qB = b2Body_GetRotation( bodyB );
	jointDef->localFrameA.q = b2Rot_identity;
	jointDef->localFrameB.q = b2InvMulRot( qB, qA );
}

static void set_joint_anchors_axis( b2JointDef* jointDef, b2BodyId bodyA, b2BodyId bodyB, float anchorX, float anchorY,
									float axisAngle )
{
	b2Pos pivot = { anchorX, anchorY };
	b2Rot axisRot = b2MakeRot( axisAngle );
	jointDef->bodyIdA = bodyA;
	jointDef->bodyIdB = bodyB;
	jointDef->localFrameA.p = b2Body_GetLocalPoint( bodyA, pivot );
	jointDef->localFrameB.p = b2Body_GetLocalPoint( bodyB, pivot );
	jointDef->localFrameA.q = axisRot;
	jointDef->localFrameB.q = axisRot;
}

static int store_joint_handle( b2JointId jointId )
{
	if ( !B2_IS_NON_NULL( jointId ) )
	{
		return -1;
	}

	int handle = alloc_joint_slot();
	if ( handle < 0 )
	{
		/* Box2D joint already lives in world — destroy so retries don't leak. */
		b2DestroyJoint( jointId, true );
		return -2;
	}

	g_joints[handle].id = jointId;
	g_joints[handle].active = 1;
	if ( handle >= g_joint_high_water )
	{
		g_joint_high_water = handle + 1;
	}
	return handle;
}

static b2BodyId slot_body( int slot )
{
	if ( slot < 0 || slot >= g_capacity || !g_slots[slot].active )
	{
		return b2_nullBodyId;
	}
	return g_slots[slot].id;
}

static void export_slot_state( int slot )
{
	if ( slot < 0 || slot >= g_capacity || !g_slots[slot].active )
	{
		return;
	}

	b2BodyId bodyId = g_slots[slot].id;
	b2ExportBodyState( slot, b2Body_GetTransform( bodyId ), b2Body_GetLinearVelocity( bodyId ),
					   b2Body_GetAngularVelocity( bodyId ) );
	if ( g_slots[slot].bodyType != b2_game_dynamic )
	{
		b2ExportBodySleeping( slot, 0 );
	}
}

static int create_body_internal( uint32_t worldPacked, int type, float x, float y, float angle, int shapeType, float halfW,
								 float halfH, float offsetX, float offsetY, float density, float friction, float restitution,
								 float linearDamp, float angularDamp, float gravityScale, float vx, float vy,
								 float angularVel, int isSensor, int enableHitEvents, uint32_t categoryBits,
								 uint32_t maskBits, int groupIndex, int fixedRotation, int entityIndex )
{
	if ( g_capacity <= 0 )
	{
		return -1;
	}

	int slot = entityIndex >= 0 ? claim_entity_slot( entityIndex ) : alloc_slot();
	if ( slot < 0 )
	{
		return -1;
	}

	b2WorldId worldId = unpack_world_id( worldPacked );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = to_b2_body_type( type );
	bodyDef.position = (b2Vec2){ x, y };
	bodyDef.rotation = b2MakeRot( angle );
	bodyDef.linearVelocity = (b2Vec2){ vx, vy };
	bodyDef.angularVelocity = angularVel;
	bodyDef.linearDamping = linearDamp;
	bodyDef.angularDamping = angularDamp;
	bodyDef.gravityScale = gravityScale;
	bodyDef.motionLocks.angularZ = fixedRotation != 0;

	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2Body_SetUserData( bodyId, (void*)(intptr_t)slot );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = density;
	shapeDef.material.friction = friction;
	shapeDef.material.restitution = restitution;
	shapeDef.filter.categoryBits = normalize_category_bits( categoryBits );
	shapeDef.filter.maskBits = normalize_mask_bits( maskBits );
	shapeDef.filter.groupIndex = groupIndex;
  shapeDef.isSensor = isSensor != 0;
  /* Visitors need sensor events too or Box2D omits sensor begin/end. */
  shapeDef.enableSensorEvents = true;
  /* Must set on shapeDef — b2Body_EnableContactEvents before Create*Shape is a no-op. */
  shapeDef.enableContactEvents = ( type != b2_game_static );
	shapeDef.enableHitEvents = enableHitEvents != 0;

	if ( shapeType == b2_game_shape_circle )
	{
		b2Circle circle = { { offsetX, offsetY }, halfW };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
	else
	{
		b2Polygon box = b2MakeOffsetBox( halfW, halfH, (b2Vec2){ offsetX, offsetY }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}

	g_slots[slot].id = bodyId;
	g_slots[slot].active = 1;
	g_slots[slot].shapeType = (uint8_t)shapeType;
	g_slots[slot].bodyType = (uint8_t)type;

	if ( slot >= g_slot_high_water )
	{
		g_slot_high_water = slot + 1;
	}

	int flags = 0;
	if ( type == b2_game_static )
	{
		flags |= b2_game_meta_static;
	}
	write_meta( slot, shapeType, halfW, halfH, flags );
	export_slot_state( slot );
	return slot;
}

EMSCRIPTEN_KEEPALIVE
uint32_t create_world(
	float gx,
	float gy,
	float lengthUnits,
	float contactHertz,
	float contactDampingRatio,
	float contactSpeed,
	float maximumLinearSpeed,
	int workerCount )
{
	float units = lengthUnits > 0.0f ? lengthUnits : 1.0f;
	b2SetLengthUnitsPerMeter( units );
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ gx, gy };
	worldDef.contactHertz = contactHertz;
	worldDef.contactDampingRatio = contactDampingRatio;
	worldDef.contactSpeed = contactSpeed;
	worldDef.maximumLinearSpeed = maximumLinearSpeed;
	if ( workerCount < 1 )
	{
		workerCount = 1;
	}
	if ( workerCount > WASM_PTHREAD_POOL_SIZE )
	{
		workerCount = WASM_PTHREAD_POOL_SIZE;
	}
	worldDef.workerCount = workerCount;
	return pack_world_id( b2CreateWorld( &worldDef ) );
}

EMSCRIPTEN_KEEPALIVE
void world_enable_sleeping( uint32_t worldPacked, int enable )
{
	b2World_EnableSleeping( unpack_world_id( worldPacked ), enable != 0 );
}

EMSCRIPTEN_KEEPALIVE
int bind_game_buffers( int maxBodies )
{
	if ( maxBodies <= 0 || maxBodies > MAX_BODIES )
	{
		return 0;
	}

	if ( g_state_buffer != NULL )
	{
		free( g_state_buffer );
	}
	if ( g_sleeping_buffer != NULL )
	{
		free( g_sleeping_buffer );
	}
	if ( g_meta_buffer != NULL )
	{
		free( g_meta_buffer );
	}
	if ( g_joint_buffer != NULL )
	{
		free( g_joint_buffer );
	}
	if ( g_query_slots != NULL )
	{
		free( g_query_slots );
	}
	if ( g_query_hits != NULL )
	{
		free( g_query_hits );
	}
	if ( g_event_header != NULL )
	{
		free( g_event_header );
	}
	if ( g_contact_begin != NULL )
	{
		free( g_contact_begin );
	}
	if ( g_contact_end != NULL )
	{
		free( g_contact_end );
	}
	if ( g_contact_hit != NULL )
	{
		free( g_contact_hit );
	}
	if ( g_sensor_begin != NULL )
	{
		free( g_sensor_begin );
	}
	if ( g_sensor_end != NULL )
	{
		free( g_sensor_end );
	}
	if ( g_joint_events != NULL )
	{
		free( g_joint_events );
	}
	if ( g_mover_planes != NULL )
	{
		free( g_mover_planes );
	}
	if ( g_body_moved != NULL )
	{
		free( g_body_moved );
	}
	if ( g_body_fell_asleep != NULL )
	{
		free( g_body_fell_asleep );
	}

	size_t stateFloatCount = (size_t)maxBodies * B2_STATE_EXPORT_CHANNELS;
	size_t jointFloatCount = (size_t)MAX_JOINTS * JOINT_FLOATS;
	g_state_buffer = (float*)malloc( stateFloatCount * sizeof( float ) );
	g_sleeping_buffer = (uint8_t*)malloc( (size_t)maxBodies * sizeof( uint8_t ) );
	g_meta_buffer = (float*)malloc( (size_t)maxBodies * META_FLOATS * sizeof( float ) );
	g_joint_buffer = (float*)malloc( jointFloatCount * sizeof( float ) );
	g_query_slots = (int32_t*)malloc( MAX_QUERY_SLOTS * sizeof( int32_t ) );
	g_query_hits = (float*)malloc( MAX_RAY_HITS * QUERY_HIT_FLOATS * sizeof( float ) );
	g_event_header = (int32_t*)malloc( EVENT_HEADER_INTS * sizeof( int32_t ) );
	g_contact_begin = (int32_t*)malloc( MAX_CONTACT_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	g_contact_end = (int32_t*)malloc( MAX_CONTACT_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	g_contact_hit = (float*)malloc( MAX_CONTACT_HIT_EVENTS * QUERY_HIT_FLOATS * sizeof( float ) );
	g_sensor_begin = (int32_t*)malloc( MAX_SENSOR_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	g_sensor_end = (int32_t*)malloc( MAX_SENSOR_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	g_joint_events = (int32_t*)malloc( MAX_JOINT_EVENTS * sizeof( int32_t ) );
	g_mover_planes = (float*)malloc( MAX_MOVER_PLANES * MOVER_PLANE_FLOATS * sizeof( float ) );
	g_body_moved = (int32_t*)malloc( (size_t)maxBodies * sizeof( int32_t ) );
	g_body_fell_asleep = (uint8_t*)malloc( (size_t)maxBodies * sizeof( uint8_t ) );
	if ( g_state_buffer == NULL || g_sleeping_buffer == NULL || g_meta_buffer == NULL || g_joint_buffer == NULL ||
		 g_query_slots == NULL || g_query_hits == NULL || g_event_header == NULL || g_contact_begin == NULL ||
		 g_contact_end == NULL || g_contact_hit == NULL || g_sensor_begin == NULL || g_sensor_end == NULL ||
		 g_joint_events == NULL || g_mover_planes == NULL || g_body_moved == NULL || g_body_fell_asleep == NULL )
	{
		free( g_state_buffer );
		free( g_sleeping_buffer );
		free( g_meta_buffer );
		free( g_joint_buffer );
		free( g_query_slots );
		free( g_query_hits );
		free( g_event_header );
		free( g_contact_begin );
		free( g_contact_end );
		free( g_contact_hit );
		free( g_sensor_begin );
		free( g_sensor_end );
		free( g_joint_events );
		free( g_mover_planes );
		free( g_body_moved );
		free( g_body_fell_asleep );
		g_state_buffer = NULL;
		g_sleeping_buffer = NULL;
		g_meta_buffer = NULL;
		g_joint_buffer = NULL;
		g_query_slots = NULL;
		g_query_hits = NULL;
		g_event_header = NULL;
		g_contact_begin = NULL;
		g_contact_end = NULL;
		g_contact_hit = NULL;
		g_sensor_begin = NULL;
		g_sensor_end = NULL;
		g_joint_events = NULL;
		g_mover_planes = NULL;
		g_body_moved = NULL;
		g_body_fell_asleep = NULL;
		return 0;
	}

	memset( g_state_buffer, 0, stateFloatCount * sizeof( float ) );
	/* Identity rotation: cos=1, sin=0 until first export */
	{
		float* rotC = g_state_buffer + 6 * maxBodies;
		for ( int i = 0; i < maxBodies; ++i )
		{
			rotC[i] = 1.0f;
		}
	}
	memset( g_sleeping_buffer, 0, (size_t)maxBodies * sizeof( uint8_t ) );
	memset( g_meta_buffer, 0, (size_t)maxBodies * META_FLOATS * sizeof( float ) );
	memset( g_joint_buffer, 0, jointFloatCount * sizeof( float ) );
	memset( g_query_slots, 0, MAX_QUERY_SLOTS * sizeof( int32_t ) );
	memset( g_query_hits, 0, MAX_RAY_HITS * QUERY_HIT_FLOATS * sizeof( float ) );
	memset( g_event_header, 0, EVENT_HEADER_INTS * sizeof( int32_t ) );
	memset( g_contact_begin, 0, MAX_CONTACT_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	memset( g_contact_end, 0, MAX_CONTACT_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	memset( g_contact_hit, 0, MAX_CONTACT_HIT_EVENTS * QUERY_HIT_FLOATS * sizeof( float ) );
	memset( g_sensor_begin, 0, MAX_SENSOR_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	memset( g_sensor_end, 0, MAX_SENSOR_EVENTS * CONTACT_PAIR_INTS * sizeof( int32_t ) );
	memset( g_joint_events, 0, MAX_JOINT_EVENTS * sizeof( int32_t ) );
	memset( g_mover_planes, 0, MAX_MOVER_PLANES * MOVER_PLANE_FLOATS * sizeof( float ) );
	memset( g_body_moved, 0, (size_t)maxBodies * sizeof( int32_t ) );
	memset( g_body_fell_asleep, 0, (size_t)maxBodies * sizeof( uint8_t ) );
	memset( g_slots, 0, sizeof( g_slots ) );

	g_capacity = maxBodies;
	g_slot_high_water = 0;
	g_next_slot = 0;
	g_free_top = 0;
	g_query_ray_count = 0;
	g_mover_plane_count = 0;
	g_body_move_count = 0;
	reset_joint_table();
	b2BindWeedExport( g_state_buffer + 0 * maxBodies, g_state_buffer + 1 * maxBodies, g_state_buffer + 2 * maxBodies,
					  g_state_buffer + 3 * maxBodies, g_state_buffer + 4 * maxBodies, g_state_buffer + 5 * maxBodies,
					  g_state_buffer + 6 * maxBodies, g_state_buffer + 7 * maxBodies, g_sleeping_buffer, maxBodies );
	return 1;
}

EMSCRIPTEN_KEEPALIVE
int create_body_box( uint32_t worldPacked, int type, float x, float y, float angle, float hx, float hy, float offsetX,
					 float offsetY, float density, float friction, float restitution, float linearDamp, float angularDamp,
					 float gravityScale, float vx, float vy, float angularVel, int isSensor, int enableHitEvents,
					 uint32_t categoryBits, uint32_t maskBits, int groupIndex, int fixedRotation, int entityIndex )
{
	return create_body_internal( worldPacked, type, x, y, angle, b2_game_shape_box, hx, hy, offsetX, offsetY, density,
								 friction, restitution, linearDamp, angularDamp, gravityScale, vx, vy, angularVel,
								 isSensor, enableHitEvents, categoryBits, maskBits, groupIndex, fixedRotation, entityIndex );
}

EMSCRIPTEN_KEEPALIVE
int create_body_circle( uint32_t worldPacked, int type, float x, float y, float angle, float radius, float offsetX,
						float offsetY, float density, float friction, float restitution, float linearDamp,
						float angularDamp, float gravityScale, float vx, float vy, float angularVel, int isSensor,
						int enableHitEvents, uint32_t categoryBits, uint32_t maskBits, int groupIndex, int fixedRotation,
						int entityIndex )
{
	return create_body_internal( worldPacked, type, x, y, angle, b2_game_shape_circle, radius, radius, offsetX, offsetY,
								 density, friction, restitution, linearDamp, angularDamp, gravityScale, vx, vy,
								 angularVel, isSensor, enableHitEvents, categoryBits, maskBits, groupIndex, fixedRotation,
								 entityIndex );
}

EMSCRIPTEN_KEEPALIVE
int create_body_polygon( uint32_t worldPacked, int type, float x, float y, float angle, float* vertsXY, int vertCount,
						 float offsetX, float offsetY, float density, float friction, float restitution, float linearDamp,
						 float angularDamp, float gravityScale, float vx, float vy, float angularVel, int isSensor,
						 int enableHitEvents, uint32_t categoryBits, uint32_t maskBits, int groupIndex, int fixedRotation,
						 int entityIndex )
{
	if ( g_capacity <= 0 || vertsXY == NULL || vertCount < 3 || vertCount > B2_MAX_POLYGON_VERTICES )
	{
		return -1;
	}

	int slot = entityIndex >= 0 ? claim_entity_slot( entityIndex ) : alloc_slot();
	if ( slot < 0 )
	{
		return -1;
	}

	b2WorldId worldId = unpack_world_id( worldPacked );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = to_b2_body_type( type );
	bodyDef.position = (b2Vec2){ x, y };
	bodyDef.rotation = b2MakeRot( angle );
	bodyDef.linearVelocity = (b2Vec2){ vx, vy };
	bodyDef.angularVelocity = angularVel;
	bodyDef.linearDamping = linearDamp;
	bodyDef.angularDamping = angularDamp;
	bodyDef.gravityScale = gravityScale;
	bodyDef.motionLocks.angularZ = fixedRotation != 0;

	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2Body_SetUserData( bodyId, (void*)(intptr_t)slot );

	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	for ( int i = 0; i < vertCount; ++i )
	{
		points[i].x = vertsXY[i * 2] + offsetX;
		points[i].y = vertsXY[i * 2 + 1] + offsetY;
	}

	b2Hull hull = b2ComputeHull( points, vertCount );
	if ( hull.count < 3 )
	{
		b2DestroyBody( bodyId );
		return -1;
	}

	b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = density;
	shapeDef.material.friction = friction;
	shapeDef.material.restitution = restitution;
	shapeDef.filter.categoryBits = normalize_category_bits( categoryBits );
	shapeDef.filter.maskBits = normalize_mask_bits( maskBits );
	shapeDef.filter.groupIndex = groupIndex;
  shapeDef.isSensor = isSensor != 0;
  shapeDef.enableSensorEvents = true;
  shapeDef.enableContactEvents = ( type != b2_game_static );
	shapeDef.enableHitEvents = enableHitEvents != 0;
	b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

	g_slots[slot].id = bodyId;
	g_slots[slot].active = 1;
	g_slots[slot].shapeType = (uint8_t)b2_game_shape_polygon;
	g_slots[slot].bodyType = (uint8_t)type;

	if ( slot >= g_slot_high_water )
	{
		g_slot_high_water = slot + 1;
	}

	int flags = 0;
	if ( type == b2_game_static )
	{
		flags |= b2_game_meta_static;
	}
	write_meta( slot, b2_game_shape_polygon, 0.0f, 0.0f, flags );
	export_slot_state( slot );
	return slot;
}

EMSCRIPTEN_KEEPALIVE
void destroy_body( int slot )
{
	if ( slot < 0 || slot >= g_capacity || !g_slots[slot].active )
	{
		return;
	}

	b2DestroyBody( g_slots[slot].id );
	g_slots[slot].active = 0;
	g_slots[slot].id = b2_nullBodyId;
	write_meta( slot, g_slots[slot].shapeType, 0.0f, 0.0f, b2_game_meta_disabled );
	b2ExportBodySleeping( slot, 0 );
	free_slot( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_set_transform( int slot, float x, float y, float c, float s )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Rot q = { c, s };
	b2Body_SetTransform( bodyId, (b2Vec2){ x, y }, b2NormalizeRot( q ) );
	export_slot_state( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_set_linear_velocity( int slot, float vx, float vy )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetLinearVelocity( bodyId, (b2Vec2){ vx, vy } );
	export_slot_state( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_set_angular_velocity( int slot, float angularVel )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetAngularVelocity( bodyId, angularVel );
	export_slot_state( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_set_fixed_rotation( int slot, int locked )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	if ( locked != 0 )
	{
		/* Zero spin before lock — SetAngularVelocity no-ops once angularZ is set. */
		b2Body_SetAngularVelocity( bodyId, 0.0f );
	}

	b2MotionLocks locks = b2Body_GetMotionLocks( bodyId );
	locks.angularZ = locked != 0;
	b2Body_SetMotionLocks( bodyId, locks );
	export_slot_state( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_set_type( int slot, int type )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetType( bodyId, to_b2_body_type( type ) );
	g_slots[slot].bodyType = (uint8_t)type;
	if ( g_meta_buffer != NULL )
	{
		float* meta = g_meta_buffer + slot * META_FLOATS;
		meta[3] = type == b2_game_static ? (float)b2_game_meta_static : 0.0f;
	}
	export_slot_state( slot );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_force( int slot, float fx, float fy, float px, float py, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyForce( bodyId, (b2Vec2){ fx, fy }, (b2Vec2){ px, py }, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_force_center( int slot, float fx, float fy, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ fx, fy }, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_set_linear_damping( int slot, float damping )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetLinearDamping( bodyId, damping );
}

EMSCRIPTEN_KEEPALIVE
void body_set_angular_damping( int slot, float damping )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetAngularDamping( bodyId, damping );
}

EMSCRIPTEN_KEEPALIVE
void body_set_gravity_scale( int slot, float scale )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetGravityScale( bodyId, scale );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_linear_impulse( int slot, float ix, float iy, float px, float py, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyLinearImpulse( bodyId, (b2Vec2){ ix, iy }, (b2Vec2){ px, py }, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_linear_impulse_center( int slot, float ix, float iy, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyLinearImpulseToCenter( bodyId, (b2Vec2){ ix, iy }, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_angular_impulse( int slot, float impulse, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyAngularImpulse( bodyId, impulse, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_apply_torque( int slot, float torque, int wake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_ApplyTorque( bodyId, torque, wake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_set_awake( int slot, int awake )
{
	b2BodyId bodyId = slot_body( slot );
	if ( !B2_IS_NON_NULL( bodyId ) )
	{
		return;
	}

	b2Body_SetAwake( bodyId, awake != 0 );
}

EMSCRIPTEN_KEEPALIVE
void body_set_filter( int slot, uint32_t categoryBits, uint32_t maskBits, int groupIndex )
{
	b2BodyId bodyId = slot_body( slot );
	b2ShapeId shapeId = first_shape( bodyId );
	if ( !B2_IS_NON_NULL( shapeId ) )
	{
		return;
	}

	b2Filter filter = b2Shape_GetFilter( shapeId );
	filter.categoryBits = normalize_category_bits( categoryBits );
	filter.maskBits = normalize_mask_bits( maskBits );
	filter.groupIndex = groupIndex;
	b2Shape_SetFilter( shapeId, filter );
}

EMSCRIPTEN_KEEPALIVE
void body_set_friction( int slot, float friction )
{
	b2ShapeId shapeId = first_shape( slot_body( slot ) );
	if ( B2_IS_NON_NULL( shapeId ) )
	{
		b2Shape_SetFriction( shapeId, friction );
	}
}

EMSCRIPTEN_KEEPALIVE
void body_set_restitution( int slot, float restitution )
{
	b2ShapeId shapeId = first_shape( slot_body( slot ) );
	if ( B2_IS_NON_NULL( shapeId ) )
	{
		b2Shape_SetRestitution( shapeId, restitution );
	}
}

EMSCRIPTEN_KEEPALIVE
void body_set_sleep_threshold( int slot, float sleepThreshold )
{
	b2BodyId bodyId = slot_body( slot );
	if ( B2_IS_NON_NULL( bodyId ) )
	{
		b2Body_SetSleepThreshold( bodyId, sleepThreshold );
	}
}

EMSCRIPTEN_KEEPALIVE
void world_set_hit_event_threshold( uint32_t worldPacked, float value )
{
	b2World_SetHitEventThreshold( unpack_world_id( worldPacked ), value );
}

EMSCRIPTEN_KEEPALIVE
void world_explode( uint32_t worldPacked, float x, float y, float radius, float falloff, float impulsePerLength,
					uint32_t maskBitsLo, uint32_t maskBitsHi )
{
	b2ExplosionDef def = b2DefaultExplosionDef();
	def.position = (b2Pos){ x, y };
	def.radius = radius;
	def.falloff = falloff;
	def.impulsePerLength = impulsePerLength;
	uint64_t mask = ( (uint64_t)maskBitsHi << 32 ) | (uint64_t)maskBitsLo;
	if ( mask == 0 )
	{
		mask = B2_DEFAULT_MASK_BITS;
	}
	def.maskBits = mask;
	b2World_Explode( unpack_world_id( worldPacked ), &def );
}

EMSCRIPTEN_KEEPALIVE
void joint_configure( int handle, int userDataInt, float forceThreshold, float torqueThreshold )
{
	if ( handle < 0 || handle >= MAX_JOINTS || !g_joints[handle].active )
	{
		return;
	}
	b2JointId jointId = g_joints[handle].id;
	b2Joint_SetUserData( jointId, (void*)(intptr_t)userDataInt );
	if ( forceThreshold > 0.0f && forceThreshold == forceThreshold )
	{
		b2Joint_SetForceThreshold( jointId, forceThreshold );
	}
	if ( torqueThreshold > 0.0f && torqueThreshold == torqueThreshold )
	{
		b2Joint_SetTorqueThreshold( jointId, torqueThreshold );
	}
}

EMSCRIPTEN_KEEPALIVE
void body_set_density( int slot, float density )
{
	b2ShapeId shapeId = first_shape( slot_body( slot ) );
	if ( B2_IS_NON_NULL( shapeId ) )
	{
		b2Shape_SetDensity( shapeId, density, true );
	}
}

EMSCRIPTEN_KEEPALIVE
void body_set_shape_box( int slot, float hx, float hy, float offsetX, float offsetY )
{
	b2BodyId bodyId = slot_body( slot );
	b2ShapeId shapeId = first_shape( bodyId );
	if ( !B2_IS_NON_NULL( shapeId ) || !( hx > 0.0f ) || !( hy > 0.0f ) )
	{
		return;
	}

	b2Polygon box = b2MakeOffsetBox( hx, hy, (b2Vec2){ offsetX, offsetY }, b2Rot_identity );
	b2Shape_SetPolygon( shapeId, &box );
	b2Body_ApplyMassFromShapes( bodyId );
	g_slots[slot].shapeType = (uint8_t)b2_game_shape_box;
	int flags = g_slots[slot].bodyType == b2_game_static ? b2_game_meta_static : 0;
	write_meta( slot, b2_game_shape_box, hx, hy, flags );
}

EMSCRIPTEN_KEEPALIVE
void body_set_shape_circle( int slot, float radius, float offsetX, float offsetY )
{
	b2BodyId bodyId = slot_body( slot );
	b2ShapeId shapeId = first_shape( bodyId );
	if ( !B2_IS_NON_NULL( shapeId ) || !( radius > 0.0f ) )
	{
		return;
	}

	b2Circle circle = { { offsetX, offsetY }, radius };
	b2Shape_SetCircle( shapeId, &circle );
	b2Body_ApplyMassFromShapes( bodyId );
	g_slots[slot].shapeType = (uint8_t)b2_game_shape_circle;
	int flags = g_slots[slot].bodyType == b2_game_static ? b2_game_meta_static : 0;
	write_meta( slot, b2_game_shape_circle, radius, radius, flags );
}

EMSCRIPTEN_KEEPALIVE
void body_set_shape_polygon( int slot, float* vertsXY, int vertCount, float offsetX, float offsetY )
{
	b2BodyId bodyId = slot_body( slot );
	b2ShapeId shapeId = first_shape( bodyId );
	if ( !B2_IS_NON_NULL( shapeId ) || vertsXY == NULL || vertCount < 3 ||
		 vertCount > B2_MAX_POLYGON_VERTICES )
	{
		return;
	}

	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	for ( int i = 0; i < vertCount; ++i )
	{
		points[i].x = vertsXY[i * 2] + offsetX;
		points[i].y = vertsXY[i * 2 + 1] + offsetY;
	}
	b2Hull hull = b2ComputeHull( points, vertCount );
	if ( hull.count < 3 )
	{
		return;
	}

	b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
	b2Shape_SetPolygon( shapeId, &polygon );
	b2Body_ApplyMassFromShapes( bodyId );
	g_slots[slot].shapeType = (uint8_t)b2_game_shape_polygon;
	int flags = g_slots[slot].bodyType == b2_game_static ? b2_game_meta_static : 0;
	write_meta( slot, b2_game_shape_polygon, 0.0f, 0.0f, flags );
}

EMSCRIPTEN_KEEPALIVE
int overlap_aabb_into( uint32_t worldPacked, float x0, float y0, float x1, float y1, uint32_t categoryBits,
					   uint32_t maskBits, int outByteOffset, int outCapacity )
{
	if ( outCapacity <= 0 || outByteOffset == 0 )
	{
		return 0;
	}

	b2WorldId worldId = unpack_world_id( worldPacked );
	int32_t* out = (int32_t*)(intptr_t)outByteOffset;
	OverlapCollectCtx ctx = { out, outCapacity, 0 };

	b2AABB aabb = { { x0, y0 }, { x1, y1 } };
	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2World_OverlapAABB( worldId, b2Pos_zero, aabb, filter, overlap_collect_callback, &ctx );
	write_event_header_count( b2_event_overlap_count, ctx.count );
	return ctx.count;
}

EMSCRIPTEN_KEEPALIVE
int overlap_aabb( uint32_t worldPacked, float x0, float y0, float x1, float y1, uint32_t categoryBits,
				  uint32_t maskBits )
{
	if ( g_query_slots == NULL )
	{
		return 0;
	}

	return overlap_aabb_into( worldPacked, x0, y0, x1, y1, categoryBits, maskBits,
							  (int)(uintptr_t)g_query_slots, MAX_QUERY_SLOTS );
}

EMSCRIPTEN_KEEPALIVE
int overlap_circle( uint32_t worldPacked, float cx, float cy, float radius, uint32_t categoryBits, uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	if ( g_query_slots == NULL )
	{
		return 0;
	}

	OverlapCollectCtx ctx = { g_query_slots, MAX_QUERY_SLOTS, 0 };
	b2Vec2 center = { cx, cy };
	b2ShapeProxy proxy = b2MakeProxy( &center, 1, radius );
	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2World_OverlapShape( worldId, b2Pos_zero, &proxy, filter, overlap_collect_callback, &ctx );
	write_event_header_count( b2_event_overlap_count, ctx.count );
	return ctx.count;
}

EMSCRIPTEN_KEEPALIVE
int overlap_box( uint32_t worldPacked, float cx, float cy, float hx, float hy, float angle, uint32_t categoryBits,
				 uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	if ( g_query_slots == NULL )
	{
		return 0;
	}

	OverlapCollectCtx ctx = { g_query_slots, MAX_QUERY_SLOTS, 0 };
	b2Vec2 corners[4];
	b2Polygon box = b2MakeBox( hx, hy );
	b2Rot rot = b2MakeRot( angle );
	for ( int i = 0; i < 4; ++i )
	{
		corners[i] = b2RotateVector( rot, box.vertices[i] );
		corners[i].x += cx;
		corners[i].y += cy;
	}
	b2ShapeProxy proxy = b2MakeProxy( corners, 4, 0.0f );
	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2World_OverlapShape( worldId, b2Pos_zero, &proxy, filter, overlap_collect_callback, &ctx );
	write_event_header_count( b2_event_overlap_count, ctx.count );
	return ctx.count;
}

EMSCRIPTEN_KEEPALIVE
int cast_ray_closest( uint32_t worldPacked, float ox, float oy, float dx, float dy, uint32_t categoryBits,
					  uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	g_query_ray_count = 0;
	if ( g_query_hits == NULL )
	{
		write_event_header_count( b2_event_ray_hit_count, 0 );
		return 0;
	}

	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2RayResult result = b2World_CastRayClosest( worldId, (b2Pos){ ox, oy }, (b2Vec2){ dx, dy }, filter );
	if ( !result.hit )
	{
		write_event_header_count( b2_event_ray_hit_count, 0 );
		return 0;
	}

	float* hit = g_query_hits;
	hit[0] = (float)slot_from_shape( result.shapeId );
	hit[1] = result.fraction;
	hit[2] = (float)result.point.x;
	hit[3] = (float)result.point.y;
	hit[4] = result.normal.x;
	hit[5] = result.normal.y;
	hit[6] = 0.0f;
	hit[7] = 0.0f;
	g_query_ray_count = 1;
	write_event_header_count( b2_event_ray_hit_count, 1 );
	return 1;
}

EMSCRIPTEN_KEEPALIVE
int cast_ray_all( uint32_t worldPacked, float ox, float oy, float dx, float dy, uint32_t categoryBits,
				  uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	g_query_ray_count = 0;
	if ( g_query_hits == NULL )
	{
		write_event_header_count( b2_event_ray_hit_count, 0 );
		return 0;
	}

	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2World_CastRay( worldId, (b2Pos){ ox, oy }, (b2Vec2){ dx, dy }, filter, ray_collect_callback, NULL );
	write_event_header_count( b2_event_ray_hit_count, g_query_ray_count );
	return g_query_ray_count;
}

EMSCRIPTEN_KEEPALIVE
float cast_mover( uint32_t worldPacked, float cx, float cy, float halfHeight, float radius, float dx, float dy,
				  uint32_t categoryBits, uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	b2Capsule mover = { { cx, cy - halfHeight }, { cx, cy + halfHeight }, radius };
	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	return b2World_CastMover( worldId, b2Pos_zero, &mover, (b2Vec2){ dx, dy }, filter );
}

EMSCRIPTEN_KEEPALIVE
int collide_mover( uint32_t worldPacked, float cx, float cy, float halfHeight, float radius, uint32_t categoryBits,
				   uint32_t maskBits )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	g_mover_plane_count = 0;
	if ( g_mover_planes == NULL )
	{
		write_event_header_count( b2_event_mover_plane_count, 0 );
		return 0;
	}

	b2Capsule mover = { { cx, cy - halfHeight }, { cx, cy + halfHeight }, radius };
	b2QueryFilter filter = make_query_filter( categoryBits, maskBits );
	b2World_CollideMover( worldId, b2Pos_zero, &mover, filter, mover_plane_callback, NULL );
	write_event_header_count( b2_event_mover_plane_count, g_mover_plane_count );
	return g_mover_plane_count;
}

EMSCRIPTEN_KEEPALIVE
int create_revolute_joint( uint32_t worldPacked, int slotA, int slotB, float anchorX, float anchorY, int enableLimit,
						   float lowerAngle, float upperAngle, int enableMotor, float motorSpeed, float maxMotorTorque )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	set_joint_anchors( &def.base, bodyA, bodyB, anchorX, anchorY );
	def.enableLimit = enableLimit != 0;
	def.lowerAngle = lowerAngle;
	def.upperAngle = upperAngle;
	def.enableMotor = enableMotor != 0;
	def.motorSpeed = motorSpeed;
	def.maxMotorTorque = maxMotorTorque;

	return store_joint_handle( b2CreateRevoluteJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_distance_joint( uint32_t worldPacked, int slotA, int slotB, float anchorX, float anchorY, float length,
						   int enableSpring, float hertz, float dampingRatio )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2DistanceJointDef def = b2DefaultDistanceJointDef();
	set_joint_anchors( &def.base, bodyA, bodyB, anchorX, anchorY );
	def.length = length;
	def.enableSpring = enableSpring != 0;
	def.hertz = hertz;
	def.dampingRatio = dampingRatio;

	return store_joint_handle( b2CreateDistanceJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_prismatic_joint( uint32_t worldPacked, int slotA, int slotB, float anchorX, float anchorY, float axisAngle,
							int enableLimit, float lowerTranslation, float upperTranslation, int enableMotor,
							float motorSpeed, float maxMotorForce )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
	set_joint_anchors_axis( &def.base, bodyA, bodyB, anchorX, anchorY, axisAngle );
	def.enableLimit = enableLimit != 0;
	def.lowerTranslation = lowerTranslation;
	def.upperTranslation = upperTranslation;
	def.enableMotor = enableMotor != 0;
	def.motorSpeed = motorSpeed;
	def.maxMotorForce = maxMotorForce;

	return store_joint_handle( b2CreatePrismaticJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_weld_joint( uint32_t worldPacked, int slotA, int slotB, float anchorX, float anchorY, float linearHertz,
						 float angularHertz, float linearDampingRatio, float angularDampingRatio )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2WeldJointDef def = b2DefaultWeldJointDef();
	set_joint_anchors( &def.base, bodyA, bodyB, anchorX, anchorY );
	capture_weld_relative_rotation( &def.base, bodyA, bodyB );
	def.linearHertz = linearHertz;
	def.angularHertz = angularHertz;
	def.linearDampingRatio = linearDampingRatio;
	def.angularDampingRatio = angularDampingRatio;

	return store_joint_handle( b2CreateWeldJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_distance_joint_local( uint32_t worldPacked, int slotA, int slotB, float lax, float lay, float lbx,
								 float lby, float length, int enableSpring, float hertz, float dampingRatio )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}
	if ( B2_ID_EQUALS( bodyA, bodyB ) )
	{
		return -1;
	}
	if ( !( length > 0.0f ) || length != length )
	{
		length = 1.0f;
	}

	b2DistanceJointDef def = b2DefaultDistanceJointDef();
	set_joint_local_anchors( &def.base, bodyA, bodyB, lax, lay, lbx, lby );
	def.length = length;
	def.enableSpring = enableSpring != 0;
	def.hertz = hertz;
	def.dampingRatio = dampingRatio;

	return store_joint_handle( b2CreateDistanceJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_revolute_joint_local( uint32_t worldPacked, int slotA, int slotB, float lax, float lay, float lbx,
								 float lby, int enableLimit, float lowerAngle, float upperAngle, int enableMotor,
								 float motorSpeed, float maxMotorTorque )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	set_joint_local_anchors( &def.base, bodyA, bodyB, lax, lay, lbx, lby );
	def.enableLimit = enableLimit != 0;
	def.lowerAngle = lowerAngle;
	def.upperAngle = upperAngle;
	def.enableMotor = enableMotor != 0;
	def.motorSpeed = motorSpeed;
	def.maxMotorTorque = maxMotorTorque;

	return store_joint_handle( b2CreateRevoluteJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
int create_weld_joint_local( uint32_t worldPacked, int slotA, int slotB, float lax, float lay, float lbx, float lby,
							 float linearHertz, float angularHertz, float linearDampingRatio,
							 float angularDampingRatio )
{
	b2BodyId bodyA = slot_body( slotA );
	b2BodyId bodyB = slot_body( slotB );
	if ( !B2_IS_NON_NULL( bodyA ) || !B2_IS_NON_NULL( bodyB ) )
	{
		return -1;
	}

	b2WeldJointDef def = b2DefaultWeldJointDef();
	set_joint_local_anchors( &def.base, bodyA, bodyB, lax, lay, lbx, lby );
	capture_weld_relative_rotation( &def.base, bodyA, bodyB );
	def.linearHertz = linearHertz;
	def.angularHertz = angularHertz;
	def.linearDampingRatio = linearDampingRatio;
	def.angularDampingRatio = angularDampingRatio;

	return store_joint_handle( b2CreateWeldJoint( unpack_world_id( worldPacked ), &def ) );
}

EMSCRIPTEN_KEEPALIVE
void destroy_joint( int handle )
{
	if ( handle < 0 || handle >= MAX_JOINTS || !g_joints[handle].active )
	{
		return;
	}

	b2DestroyJoint( g_joints[handle].id, true );
	g_joints[handle].active = 0;
	write_joint_meta( handle, 0, b2_game_joint_disabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	free_joint_slot( handle );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_count( void )
{
	return g_joint_high_water;
}

EMSCRIPTEN_KEEPALIVE
void step_world( uint32_t worldPacked, float timeStep, int subStepCount )
{
	b2WorldId worldId = unpack_world_id( worldPacked );
	clear_event_counts();
	// Weed SoA sleep paint: assume asleep, clear on export. Skip when Box2D sleep is off
	// so RigidBody.sleeping matches b2World_EnableSleeping(false).
	if ( b2World_IsSleepingEnabled( worldId ) )
	{
		mark_active_dynamics_sleeping();
	}
	b2World_Step( worldId, timeStep, subStepCount );
	g_liquidfun_step_ms = 0.0f;
	if ( g_particles != NULL )
	{
		double t0 = emscripten_get_now();
		lfParticleSystem_Step( g_particles, timeStep, g_particle_sub_steps );
		g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
		if ( g_particle_x != NULL && g_particle_y != NULL )
		{
			const b2Vec2* pos = lfParticleSystem_GetPositionBuffer( g_particles );
			const float* alpha = g_particle_alpha != NULL ? lfParticleSystem_GetAlphaBuffer( g_particles ) : NULL;
			for ( int32_t i = 0; i < g_particle_count_value; i++ )
			{
				g_particle_x[i] = pos[i].x;
				g_particle_y[i] = pos[i].y;
				if ( alpha != NULL )
				{
					g_particle_alpha[i] = alpha[i];
				}
			}
		}
		g_liquidfun_step_ms = (float)( emscripten_get_now() - t0 );
	}
	export_contact_events( worldId );
	export_sensor_events( worldId );
	export_joint_events( worldId );
	export_body_move_events( worldId );
	export_profile_and_counters( worldId );
	export_all_joints();
}

EMSCRIPTEN_KEEPALIVE
int get_state_byte_offset( void )
{
	return g_state_buffer == NULL ? 0 : (int)( (uintptr_t)g_state_buffer );
}

EMSCRIPTEN_KEEPALIVE
int get_sleeping_byte_offset( void )
{
	return g_sleeping_buffer == NULL ? 0 : (int)( (uintptr_t)g_sleeping_buffer );
}

EMSCRIPTEN_KEEPALIVE
int get_meta_byte_offset( void )
{
	return g_meta_buffer == NULL ? 0 : (int)( (uintptr_t)g_meta_buffer );
}

EMSCRIPTEN_KEEPALIVE
int get_body_capacity( void )
{
	return g_capacity;
}

EMSCRIPTEN_KEEPALIVE
int get_max_body_slots( void )
{
	return MAX_BODIES;
}

EMSCRIPTEN_KEEPALIVE
int get_slot_count( void )
{
	return g_slot_high_water;
}

EMSCRIPTEN_KEEPALIVE
int get_state_channel_offset( int channel )
{
	if ( g_state_buffer == NULL || channel < 0 || channel >= B2_STATE_EXPORT_CHANNELS )
	{
		return 0;
	}

	return channel * g_capacity;
}

EMSCRIPTEN_KEEPALIVE
int get_meta_float_stride( void )
{
	return META_FLOATS;
}

EMSCRIPTEN_KEEPALIVE
int get_state_region_bytes( void )
{
	return g_capacity * B2_STATE_EXPORT_CHANNELS * (int)sizeof( float );
}

EMSCRIPTEN_KEEPALIVE
int get_meta_region_bytes( void )
{
	return g_capacity * META_FLOATS * (int)sizeof( float );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_byte_offset( void )
{
	return g_joint_buffer == NULL ? 0 : (int)( (uintptr_t)g_joint_buffer );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_float_stride( void )
{
	return JOINT_FLOATS;
}

EMSCRIPTEN_KEEPALIVE
int get_joint_region_bytes( void )
{
	return MAX_JOINTS * JOINT_FLOATS * (int)sizeof( float );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_capacity( void )
{
	return MAX_JOINTS;
}

EMSCRIPTEN_KEEPALIVE
int get_query_slots_byte_offset( void )
{
	return g_query_slots == NULL ? 0 : (int)( (uintptr_t)g_query_slots );
}

EMSCRIPTEN_KEEPALIVE
int get_query_hits_byte_offset( void )
{
	return g_query_hits == NULL ? 0 : (int)( (uintptr_t)g_query_hits );
}

EMSCRIPTEN_KEEPALIVE
int get_event_header_byte_offset( void )
{
	return g_event_header == NULL ? 0 : (int)( (uintptr_t)g_event_header );
}

EMSCRIPTEN_KEEPALIVE
int get_contact_begin_byte_offset( void )
{
	return g_contact_begin == NULL ? 0 : (int)( (uintptr_t)g_contact_begin );
}

EMSCRIPTEN_KEEPALIVE
int get_contact_end_byte_offset( void )
{
	return g_contact_end == NULL ? 0 : (int)( (uintptr_t)g_contact_end );
}

EMSCRIPTEN_KEEPALIVE
int get_contact_hit_byte_offset( void )
{
	return g_contact_hit == NULL ? 0 : (int)( (uintptr_t)g_contact_hit );
}

EMSCRIPTEN_KEEPALIVE
int get_sensor_begin_byte_offset( void )
{
	return g_sensor_begin == NULL ? 0 : (int)( (uintptr_t)g_sensor_begin );
}

EMSCRIPTEN_KEEPALIVE
int get_sensor_end_byte_offset( void )
{
	return g_sensor_end == NULL ? 0 : (int)( (uintptr_t)g_sensor_end );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_events_byte_offset( void )
{
	return g_joint_events == NULL ? 0 : (int)( (uintptr_t)g_joint_events );
}

EMSCRIPTEN_KEEPALIVE
int get_joint_event_capacity( void )
{
	return MAX_JOINT_EVENTS;
}

EMSCRIPTEN_KEEPALIVE
int get_mover_planes_byte_offset( void )
{
	return g_mover_planes == NULL ? 0 : (int)( (uintptr_t)g_mover_planes );
}

EMSCRIPTEN_KEEPALIVE
int get_query_capacity( void )
{
	return MAX_QUERY_SLOTS;
}

EMSCRIPTEN_KEEPALIVE
int get_ray_hit_capacity( void )
{
	return MAX_RAY_HITS;
}

EMSCRIPTEN_KEEPALIVE
int get_query_hit_float_stride( void )
{
	return QUERY_HIT_FLOATS;
}

EMSCRIPTEN_KEEPALIVE
int get_contact_event_capacity( void )
{
	return MAX_CONTACT_EVENTS;
}

EMSCRIPTEN_KEEPALIVE
int get_sensor_event_capacity( void )
{
	return MAX_SENSOR_EVENTS;
}

EMSCRIPTEN_KEEPALIVE
int get_contact_hit_capacity( void )
{
	return MAX_CONTACT_HIT_EVENTS;
}

EMSCRIPTEN_KEEPALIVE
int get_mover_plane_capacity( void )
{
	return MAX_MOVER_PLANES;
}

EMSCRIPTEN_KEEPALIVE
int get_mover_plane_float_stride( void )
{
	return MOVER_PLANE_FLOATS;
}

EMSCRIPTEN_KEEPALIVE
int get_event_header_int_count( void )
{
	return EVENT_HEADER_INTS;
}

EMSCRIPTEN_KEEPALIVE
int get_contact_pair_int_stride( void )
{
	return CONTACT_PAIR_INTS;
}

EMSCRIPTEN_KEEPALIVE
int get_body_move_count( void )
{
	return g_body_move_count;
}

EMSCRIPTEN_KEEPALIVE
int get_body_move_byte_offset( void )
{
	return g_body_moved == NULL ? 0 : (int)( (uintptr_t)g_body_moved );
}

EMSCRIPTEN_KEEPALIVE
int get_body_fell_asleep_byte_offset( void )
{
	return g_body_fell_asleep == NULL ? 0 : (int)( (uintptr_t)g_body_fell_asleep );
}

EMSCRIPTEN_KEEPALIVE
int get_body_move_capacity( void )
{
	return g_capacity;
}

EMSCRIPTEN_KEEPALIVE
int get_awake_body_count( uint32_t worldPacked )
{
	return b2World_GetAwakeBodyCount( unpack_world_id( worldPacked ) );
}

EMSCRIPTEN_KEEPALIVE
int get_profile_byte_offset( void )
{
	return (int)( (uintptr_t)g_profile );
}

EMSCRIPTEN_KEEPALIVE
int get_profile_float_count( void )
{
	return 5;
}

EMSCRIPTEN_KEEPALIVE
int get_counters_byte_offset( void )
{
	return (int)( (uintptr_t)g_counters );
}

EMSCRIPTEN_KEEPALIVE
int get_counters_int_count( void )
{
	return 7;
}

EMSCRIPTEN_KEEPALIVE
int create_particle_system( uint32_t worldPacked, float radius, float density, int maxParticles, int strictContactCheck )
{
	if ( g_particles != NULL )
	{
		lfParticleSystem_Destroy( g_particles );
		g_particles = NULL;
		g_particle_capacity = 0;
		g_particle_count_value = 0;
	}
	free( g_particle_x );
	free( g_particle_y );
	g_particle_x = NULL;
	g_particle_y = NULL;
	free( g_particle_alpha );
	g_particle_alpha = NULL;
	if ( maxParticles <= 0 || maxParticles > MAX_PARTICLES )
	{
		return 0;
	}
	lfParticleSystemDef def = lfDefaultParticleSystemDef();
	def.radius = radius;
	def.density = density;
	def.maxParticles = maxParticles;
	def.growable = false;
	def.strictContactCheck = strictContactCheck != 0;
	g_particles = lfParticleSystem_Create( unpack_world_id( worldPacked ), &def );
	if ( g_particles == NULL )
	{
		return 0;
	}
	g_particle_capacity = lfParticleSystem_GetCapacity( g_particles );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
	g_particle_x = (float*)malloc( (size_t)g_particle_capacity * sizeof( float ) );
	g_particle_y = (float*)malloc( (size_t)g_particle_capacity * sizeof( float ) );
	g_particle_alpha = (float*)malloc( (size_t)g_particle_capacity * sizeof( float ) );
	return 1;
}

EMSCRIPTEN_KEEPALIVE
void destroy_particle_system( void )
{
	if ( g_particles != NULL )
	{
		lfParticleSystem_Destroy( g_particles );
		g_particles = NULL;
	}
	g_particle_capacity = 0;
	g_particle_count_value = 0;
	free( g_particle_x );
	free( g_particle_y );
	g_particle_x = NULL;
	g_particle_y = NULL;
	free( g_particle_alpha );
	g_particle_alpha = NULL;
	g_liquidfun_step_ms = 0.0f;
}

EMSCRIPTEN_KEEPALIVE
int create_particle_box( float x0, float y0, float x1, float y1, float spacing, uint32_t flags )
{
	if ( g_particles == NULL )
	{
		return 0;
	}
	b2AABB box = { { x0, y0 }, { x1, y1 } };
	int n = lfParticleSystem_CreateParticleBox( g_particles, box, spacing, flags, ( b2Vec2 ){ 0.0f, 0.0f } );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
	return n;
}

EMSCRIPTEN_KEEPALIVE
int create_particle_group_box( float x0, float y0, float x1, float y1, float spacing, uint32_t flags, float strength,
								float lifetimeMinSec, float lifetimeMaxSec, int fadeToAlpha0, float viscousScale,
								int trackGroup, uint32_t groupFlags )
{
	if ( g_particles == NULL )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	lfParticleGroupDef def = lfDefaultParticleGroupDef();
	def.flags = flags;
	def.groupFlags = groupFlags;
	def.box = ( b2AABB ){ { x0, y0 }, { x1, y1 } };
	def.spacing = spacing;
	def.strength = strength;
	def.lifetimeMin = lifetimeMinSec;
	def.lifetimeMax = lifetimeMaxSec;
	def.fadeToAlpha0 = fadeToAlpha0;
	def.viscousScale = viscousScale > 0.0f ? viscousScale : 1.0f;
	def.trackGroup = trackGroup;
	lfParticleGroupId id = lfParticleSystem_CreateParticleGroupBox( g_particles, &def );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
	return (int)id;
}

EMSCRIPTEN_KEEPALIVE
int create_particle_group_circle( float cx, float cy, float radius, float spacing, uint32_t flags, float strength,
								   float lifetimeMinSec, float lifetimeMaxSec, int fadeToAlpha0, float viscousScale,
								   int trackGroup, uint32_t groupFlags )
{
	if ( g_particles == NULL )
	{
		return LF_NULL_PARTICLE_GROUP;
	}
	lfParticleGroupDef def = lfDefaultParticleGroupDef();
	def.flags = flags;
	def.groupFlags = groupFlags;
	def.position = ( b2Vec2 ){ cx, cy };
	def.radius = radius;
	def.spacing = spacing;
	def.strength = strength;
	def.lifetimeMin = lifetimeMinSec;
	def.lifetimeMax = lifetimeMaxSec;
	def.fadeToAlpha0 = fadeToAlpha0;
	def.viscousScale = viscousScale > 0.0f ? viscousScale : 1.0f;
	def.trackGroup = trackGroup;
	lfParticleGroupId id = lfParticleSystem_CreateParticleGroupCircle( g_particles, &def );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
	return (int)id;
}

EMSCRIPTEN_KEEPALIVE
void destroy_particle_group( int groupId )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_DestroyParticleGroup( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
void set_particle_sub_steps( int n )
{
	g_particle_sub_steps = n < 1 ? 1 : n;
}

EMSCRIPTEN_KEEPALIVE
void set_particle_tuning( float dampingStrength, float pressureStrength, float viscousStrength, float tensileStrength,
						  float powderStrength, float springStrength, float staticPressureStrength,
						  float staticPressureRelaxation, int staticPressureIterations )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_SetTuning( g_particles, dampingStrength, pressureStrength, viscousStrength, tensileStrength,
								powderStrength, springStrength, staticPressureStrength, staticPressureRelaxation,
								staticPressureIterations );
}

EMSCRIPTEN_KEEPALIVE
void set_group_viscous_scale( int groupId, float scale )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_SetGroupViscousScale( g_particles, (lfParticleGroupId)groupId, scale );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_group_slot_count( void )
{
	return g_particles == NULL ? 0 : lfParticleSystem_GetGroupSlotCount( g_particles );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_group_alive( int groupId )
{
	return g_particles == NULL ? 0 : lfParticleSystem_IsGroupAlive( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_group_particle_count( int groupId )
{
	return g_particles == NULL ? 0 : lfParticleSystem_GetGroupParticleCount( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_center_x( int groupId )
{
	if ( g_particles == NULL )
	{
		return 0.0f;
	}
	return lfParticleSystem_GetGroupCenter( g_particles, (lfParticleGroupId)groupId ).x;
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_center_y( int groupId )
{
	if ( g_particles == NULL )
	{
		return 0.0f;
	}
	return lfParticleSystem_GetGroupCenter( g_particles, (lfParticleGroupId)groupId ).y;
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_vx( int groupId )
{
	if ( g_particles == NULL )
	{
		return 0.0f;
	}
	return lfParticleSystem_GetGroupLinearVelocity( g_particles, (lfParticleGroupId)groupId ).x;
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_vy( int groupId )
{
	if ( g_particles == NULL )
	{
		return 0.0f;
	}
	return lfParticleSystem_GetGroupLinearVelocity( g_particles, (lfParticleGroupId)groupId ).y;
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_angular_velocity( int groupId )
{
	return g_particles == NULL
			   ? 0.0f
			   : lfParticleSystem_GetGroupAngularVelocity( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_angle( int groupId )
{
	return g_particles == NULL ? 0.0f : lfParticleSystem_GetGroupAngle( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
float get_particle_group_viscous_scale( int groupId )
{
	return g_particles == NULL ? 1.0f
							  : lfParticleSystem_GetGroupViscousScale( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_group_first_index( int groupId )
{
	return g_particles == NULL ? 0 : lfParticleSystem_GetGroupFirstIndex( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_group_last_index( int groupId )
{
	return g_particles == NULL ? 0 : lfParticleSystem_GetGroupLastIndex( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
uint32_t get_particle_group_flags( int groupId )
{
	return g_particles == NULL ? 0u : lfParticleSystem_GetGroupFlags( g_particles, (lfParticleGroupId)groupId );
}

EMSCRIPTEN_KEEPALIVE
void join_particle_groups( int groupA, int groupB )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_JoinParticleGroups( g_particles, (lfParticleGroupId)groupA, (lfParticleGroupId)groupB );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
}

EMSCRIPTEN_KEEPALIVE
void split_particle_group( int groupId )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_SplitParticleGroup( g_particles, (lfParticleGroupId)groupId );
	g_particle_count_value = lfParticleSystem_GetParticleCount( g_particles );
}

EMSCRIPTEN_KEEPALIVE
void particle_apply_force( int index, float fx, float fy )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_ParticleApplyForce( g_particles, index, ( b2Vec2 ){ fx, fy } );
}

EMSCRIPTEN_KEEPALIVE
void particle_apply_linear_impulse( int index, float ix, float iy )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_ParticleApplyLinearImpulse( g_particles, index, ( b2Vec2 ){ ix, iy } );
}

EMSCRIPTEN_KEEPALIVE
void particle_group_apply_force( int groupId, float fx, float fy )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_GroupApplyForce( g_particles, (lfParticleGroupId)groupId, ( b2Vec2 ){ fx, fy } );
}

EMSCRIPTEN_KEEPALIVE
void particle_group_apply_linear_impulse( int groupId, float ix, float iy )
{
	if ( g_particles == NULL )
	{
		return;
	}
	lfParticleSystem_GroupApplyLinearImpulse( g_particles, (lfParticleGroupId)groupId, ( b2Vec2 ){ ix, iy } );
}

#define MAX_PARTICLE_QUERY_HITS 512
static int g_particle_query_hits[MAX_PARTICLE_QUERY_HITS];
static int g_particle_query_hit_count = 0;

EMSCRIPTEN_KEEPALIVE
int particle_query_aabb( float x0, float y0, float x1, float y1 )
{
	if ( g_particles == NULL )
	{
		g_particle_query_hit_count = 0;
		return 0;
	}
	b2AABB aabb = { { x0, y0 }, { x1, y1 } };
	g_particle_query_hit_count =
		lfParticleSystem_QueryAABB( g_particles, aabb, g_particle_query_hits, MAX_PARTICLE_QUERY_HITS );
	return g_particle_query_hit_count;
}

EMSCRIPTEN_KEEPALIVE
int particle_ray_cast( float x1, float y1, float x2, float y2 )
{
	if ( g_particles == NULL )
	{
		g_particle_query_hit_count = 0;
		return 0;
	}
	g_particle_query_hit_count = lfParticleSystem_RayCast( g_particles, ( b2Vec2 ){ x1, y1 }, ( b2Vec2 ){ x2, y2 },
														   g_particle_query_hits, MAX_PARTICLE_QUERY_HITS );
	return g_particle_query_hit_count;
}

EMSCRIPTEN_KEEPALIVE
int get_particle_query_hit( int i )
{
	if ( i < 0 || i >= g_particle_query_hit_count )
	{
		return -1;
	}
	return g_particle_query_hits[i];
}

EMSCRIPTEN_KEEPALIVE
int get_particle_weight_byte_offset( void )
{
	if ( g_particles == NULL )
	{
		return 0;
	}
	const float* w = lfParticleSystem_GetWeightBuffer( g_particles );
	return w == NULL ? 0 : (int)( (uintptr_t)w );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_count( void )
{
	return g_particle_count_value;
}

EMSCRIPTEN_KEEPALIVE
float get_liquidfun_step_ms( void )
{
	return g_liquidfun_step_ms;
}

EMSCRIPTEN_KEEPALIVE
int get_particle_capacity( void )
{
	return g_particle_capacity;
}

EMSCRIPTEN_KEEPALIVE
float get_particle_radius( void )
{
	return g_particles == NULL ? 0.0f : lfParticleSystem_GetRadius( g_particles );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_count_byte_offset( void )
{
	return (int)( (uintptr_t)&g_particle_count_value );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_pos_byte_offset( void )
{
	if ( g_particles == NULL )
	{
		return 0;
	}
	const b2Vec2* pos = lfParticleSystem_GetPositionBuffer( g_particles );
	return pos == NULL ? 0 : (int)( (uintptr_t)pos );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_vel_byte_offset( void )
{
	if ( g_particles == NULL )
	{
		return 0;
	}
	const b2Vec2* vel = lfParticleSystem_GetVelocityBuffer( g_particles );
	return vel == NULL ? 0 : (int)( (uintptr_t)vel );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_flags_byte_offset( void )
{
	if ( g_particles == NULL )
	{
		return 0;
	}
	const uint32_t* flags = lfParticleSystem_GetFlagsBuffer( g_particles );
	return flags == NULL ? 0 : (int)( (uintptr_t)flags );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_x_byte_offset( void )
{
	return g_particle_x == NULL ? 0 : (int)( (uintptr_t)g_particle_x );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_alpha_byte_offset( void )
{
	return g_particle_alpha == NULL ? 0 : (int)( (uintptr_t)g_particle_alpha );
}

EMSCRIPTEN_KEEPALIVE
int get_particle_y_byte_offset( void )
{
	return g_particle_y == NULL ? 0 : (int)( (uintptr_t)g_particle_y );
}


/* dlmalloc occupancy — for Weed MemoryPanel (used vs INITIAL_MEMORY reserved). */
#include <malloc.h>

EMSCRIPTEN_KEEPALIVE
size_t weedjs_heap_bytes_used( void )
{
	struct mallinfo info = mallinfo();
	return (size_t)info.uordblks;
}
