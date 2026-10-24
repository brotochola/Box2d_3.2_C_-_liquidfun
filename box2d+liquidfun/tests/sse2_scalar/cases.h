// liquidfun-c - tests/sse2_scalar/cases.h
// SPDX-License-Identifier: Zlib
//
// The cases that check lf_sse2_scalar.h (the no-SIMD build's plain C intrinsics) against
// Emscripten's SSE2 on real wasm SIMD, which is what the SIMD build runs. See NO_SIMD_BUILD.md.
//
// X( name, input domain, how lanes compare, body ). A body reads the lanes a[4] and b[4] (A and B
// load them) and writes four lanes to out[4]. simd.c compiles every body against <emmintrin.h> with
// -msimd128 -msse2, scalar.c against lf_sse2_scalar.h, and main.c compares the two on the same inputs.
// Every intrinsic lf_particle_system.c uses must appear in a body here: check.mjs fails otherwise.
//
// Domains: ANY, NONNEG (a >= 0), POS (a > 0), NONZERO_B (b far from 0).
// FLOAT lanes compare as floats, with +0 equal to -0 (-ffast-math lets either build drop the sign of
// a zero); BITS lanes (masks, bitwise results) compare bit for bit.

#pragma once

#define A _mm_loadu_ps( a )
#define B _mm_loadu_ps( b )
#define STORE( v ) _mm_storeu_ps( out, ( v ) )

#define LF_SSE2_CASES( X ) \
	X( loadu_storeu, ANY, BITS, STORE( A ) ) \
	X( set1_ps, ANY, FLOAT, STORE( _mm_set1_ps( a[1] ) ) ) \
	X( set_ss, ANY, FLOAT, STORE( _mm_set_ss( a[2] ) ) ) \
	X( cvtss_f32, ANY, FLOAT, out[0] = _mm_cvtss_f32( A ); out[1] = out[2] = out[3] = 0.0f ) \
	X( add_ps, ANY, FLOAT, STORE( _mm_add_ps( A, B ) ) ) \
	X( mul_ps, ANY, FLOAT, STORE( _mm_mul_ps( A, B ) ) ) \
	X( div_ps, NONZERO_B, FLOAT, STORE( _mm_div_ps( A, B ) ) ) \
	X( sqrt_ps, NONNEG, FLOAT, STORE( _mm_sqrt_ps( A ) ) ) \
	X( min_ps, ANY, FLOAT, STORE( _mm_min_ps( A, B ) ) ) \
	X( max_ps, ANY, FLOAT, STORE( _mm_max_ps( A, B ) ) ) \
	X( min_ss, ANY, FLOAT, STORE( _mm_min_ss( A, B ) ) ) \
	X( max_ss, ANY, FLOAT, STORE( _mm_max_ss( A, B ) ) ) \
	X( rsqrt_ss, POS, FLOAT, STORE( _mm_rsqrt_ss( A ) ) ) \
	X( cmpgt_ps, ANY, BITS, STORE( _mm_cmpgt_ps( A, B ) ) ) \
	X( and_ps, ANY, BITS, STORE( _mm_and_ps( A, B ) ) ) \
	X( or_ps, ANY, BITS, STORE( _mm_or_ps( A, B ) ) ) \
	X( andnot_ps, ANY, BITS, STORE( _mm_andnot_ps( A, B ) ) ) \
	X( select, ANY, FLOAT, STORE( _mm_or_ps( _mm_and_ps( _mm_cmpgt_ps( A, B ), A ), _mm_andnot_ps( _mm_cmpgt_ps( A, B ), B ) ) ) ) \
	X( shuffle_ps_01, ANY, FLOAT, STORE( _mm_shuffle_ps( A, A, 0x01 ) ) ) \
	X( shuffle_ps_1b, ANY, FLOAT, STORE( _mm_shuffle_ps( A, B, 0x1b ) ) ) \
	X( shuffle_ps_4e, ANY, FLOAT, STORE( _mm_shuffle_ps( A, B, 0x4e ) ) ) \
	X( shuffle_ps_e4, ANY, FLOAT, STORE( _mm_shuffle_ps( A, B, 0xe4 ) ) ) \
	X( movehl_ps, ANY, FLOAT, STORE( _mm_movehl_ps( A, B ) ) )
