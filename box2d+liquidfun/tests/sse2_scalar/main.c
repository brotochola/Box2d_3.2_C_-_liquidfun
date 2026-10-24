// liquidfun-c - tests/sse2_scalar/main.c
// SPDX-License-Identifier: Zlib
//
// Runs every case of cases.h on the same inputs through simd.c (Emscripten's SSE2 on wasm SIMD) and
// scalar.c (lf_sse2_scalar.h) and compares the four result lanes. Exits with 1 on any difference.
// The inputs are finite floats from a fixed seed: small and large values, normal values near zero,
// subnormals, signed zeros, and b lanes equal to a's or to -a's (the ties of min, max and compare).

#include "cases.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef void ( *case_fn )( const float* a, const float* b, float* out );

#define X( name, domain, kind, body ) \
	void simd_##name( const float* a, const float* b, float* out ); \
	void scalar_##name( const float* a, const float* b, float* out );
LF_SSE2_CASES( X )
#undef X

enum
{
	ANY,
	NONNEG,
	POS,
	NONZERO_B
};

enum
{
	FLOAT,
	BITS
};

typedef struct
{
	const char* name;
	int domain;
	int kind;
	case_fn simd;
	case_fn scalar;
} test_case;

#define X( name, domain, kind, body ) { #name, domain, kind, simd_##name, scalar_##name },
static const test_case cases[] = { LF_SSE2_CASES( X ) };
#undef X

#define VECTORS 100000

static uint32_t rng = 0x9e3779b9u;

static uint32_t next( void )
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return rng;
}

static float uniform( float scale )
{
	return ( (float)( next() >> 8 ) / 16777216.0f * 2.0f - 1.0f ) * scale;
}

static float bits_float( uint32_t u )
{
	float f;
	memcpy( &f, &u, sizeof f );
	return f;
}

static uint32_t float_bits( float f )
{
	uint32_t u;
	memcpy( &u, &f, sizeof u );
	return u;
}

static float lane( void )
{
	switch ( next() % 6 )
	{
		case 0:
			return uniform( 2.0f );
		case 1:
			return uniform( 1e4f );
		case 2:
			return uniform( 1e18f );
		case 3:
			return uniform( 1e-20f );
		case 4:
			return bits_float( next() & 0x807fffffu ); // subnormal (or zero)
		default:
			return ( next() & 1 ) ? -0.0f : 0.0f;
	}
}

static void inputs( int domain, float* a, float* b )
{
	for ( int i = 0; i < 4; ++i )
	{
		a[i] = lane();
		uint32_t r = next() % 12;
		b[i] = r < 2 ? a[i] : r < 3 ? -a[i] : lane();
		if ( domain == NONNEG || domain == POS )
			a[i] = fabsf( a[i] );
		if ( domain == POS && !( a[i] > 1e-30f ) )
			a[i] = 1.0f;
		if ( domain == NONZERO_B && !( fabsf( b[i] ) > 1e-6f ) )
			b[i] = 1.0f;
	}
}

static int same( int kind, float x, float y )
{
	uint32_t ux = float_bits( x ), uy = float_bits( y );
	if ( ux == uy )
		return 1;
	return kind == FLOAT && ( ux & 0x7fffffffu ) == 0 && ( uy & 0x7fffffffu ) == 0;
}

int main( void )
{
	int count = (int)( sizeof cases / sizeof cases[0] );
	int failed = 0;
	for ( int c = 0; c < count; ++c )
	{
		const test_case* t = &cases[c];
		int bad = 0;
		for ( int v = 0; v < VECTORS; ++v )
		{
			float a[4], b[4], want[4], got[4];
			inputs( t->domain, a, b );
			t->simd( a, b, want );
			t->scalar( a, b, got );
			for ( int i = 0; i < 4; ++i )
			{
				if ( same( t->kind, want[i], got[i] ) )
					continue;
				if ( bad++ < 3 )
					printf( "  %s lane %d: a = %.9g (%08x), b = %.9g (%08x): SSE2 %.9g (%08x), plain C %.9g (%08x)\n", t->name, i,
							a[i], float_bits( a[i] ), b[i], float_bits( b[i] ), want[i], float_bits( want[i] ), got[i],
							float_bits( got[i] ) );
			}
		}
		if ( bad )
		{
			printf( "FAIL %s: %d lanes differ\n", t->name, bad );
			failed++;
		}
	}
	if ( failed )
	{
		printf( "lf_sse2_scalar.h: %d of %d cases differ from Emscripten's SSE2\n", failed, count );
		return 1;
	}
	printf( "lf_sse2_scalar.h: %d cases x %d vectors, same results as Emscripten's SSE2 on wasm SIMD\n", count, VECTORS );
	return 0;
}
