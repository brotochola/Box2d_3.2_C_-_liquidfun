// liquidfun-c - src/lf_sse2_scalar.h
// SPDX-License-Identifier: Zlib
//
// Plain C stand-ins for the SSE2 intrinsics lf_particle_system.c uses, for the no-SIMD WebAssembly
// build (BOX2D_WASM_NO_SIMD in wasm/CMakeLists.txt, which defines BOX2D_DISABLE_SIMD). Engines
// without wasm SIMD, such as Firefox/GeckoView on 32-bit ARM, cannot compile a module that holds a
// single SIMD instruction, so this build must not emit any.
//
// Each function follows Emscripten's own SSE emulation (system/include/compat/xmmintrin.h), which is
// what the SIMD build runs, so both builds compute the same values:
//   _mm_min_ps(a, b) = a < b ? a : b and _mm_max_ps(a, b) = a > b ? a : b, lane by lane;
//   _mm_rsqrt_ss is 1 / sqrt(x), not x86's 12-bit estimate;
//   _mm_shuffle_ps(a, b, m) takes lanes 0 and 1 from a and lanes 2 and 3 from b.
// Only the intrinsics lf_particle_system.c calls are here. A new one needs a case in
// tests/sse2_scalar, which checks every function here against Emscripten's SSE2 on real wasm SIMD.
// See NO_SIMD_BUILD.md.

#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

typedef union
{
	float f[4];
	uint32_t u[4];
} __m128;

static inline __m128 _mm_loadu_ps( const float* p )
{
	__m128 r;
	memcpy( r.f, p, sizeof( r.f ) );
	return r;
}

static inline void _mm_storeu_ps( float* p, __m128 a )
{
	memcpy( p, a.f, sizeof( a.f ) );
}

static inline __m128 _mm_set1_ps( float x )
{
	__m128 r = { { x, x, x, x } };
	return r;
}

static inline __m128 _mm_set_ss( float x )
{
	__m128 r = { { x, 0.0f, 0.0f, 0.0f } };
	return r;
}

static inline float _mm_cvtss_f32( __m128 a )
{
	return a.f[0];
}

static inline __m128 _mm_add_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] += b.f[i];
	return a;
}

static inline __m128 _mm_mul_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] *= b.f[i];
	return a;
}

static inline __m128 _mm_div_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] /= b.f[i];
	return a;
}

static inline __m128 _mm_sqrt_ps( __m128 a )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] = sqrtf( a.f[i] );
	return a;
}

static inline __m128 _mm_min_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] = a.f[i] < b.f[i] ? a.f[i] : b.f[i];
	return a;
}

static inline __m128 _mm_max_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.f[i] = a.f[i] > b.f[i] ? a.f[i] : b.f[i];
	return a;
}

// Lane 0 from the operation, lanes 1..3 from a.
static inline __m128 _mm_min_ss( __m128 a, __m128 b )
{
	a.f[0] = a.f[0] < b.f[0] ? a.f[0] : b.f[0];
	return a;
}

static inline __m128 _mm_max_ss( __m128 a, __m128 b )
{
	a.f[0] = a.f[0] > b.f[0] ? a.f[0] : b.f[0];
	return a;
}

static inline __m128 _mm_rsqrt_ss( __m128 a )
{
	a.f[0] = 1.0f / sqrtf( a.f[0] );
	return a;
}

// All bits set in the lanes where a > b.
static inline __m128 _mm_cmpgt_ps( __m128 a, __m128 b )
{
	__m128 r;
	for ( int i = 0; i < 4; ++i )
		r.u[i] = a.f[i] > b.f[i] ? 0xffffffffu : 0u;
	return r;
}

static inline __m128 _mm_and_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.u[i] &= b.u[i];
	return a;
}

static inline __m128 _mm_or_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.u[i] |= b.u[i];
	return a;
}

// ~a & b
static inline __m128 _mm_andnot_ps( __m128 a, __m128 b )
{
	for ( int i = 0; i < 4; ++i )
		a.u[i] = ~a.u[i] & b.u[i];
	return a;
}

static inline __m128 _mm_shuffle_ps( __m128 a, __m128 b, int mask )
{
	__m128 r = { { a.f[mask & 3], a.f[( mask >> 2 ) & 3], b.f[( mask >> 4 ) & 3], b.f[( mask >> 6 ) & 3] } };
	return r;
}

// Lanes 2 and 3 of b, then lanes 2 and 3 of a.
static inline __m128 _mm_movehl_ps( __m128 a, __m128 b )
{
	__m128 r = { { b.f[2], b.f[3], a.f[2], a.f[3] } };
	return r;
}
