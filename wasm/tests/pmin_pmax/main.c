// WeedJS E-357: wasm pmin( b, a ) / pmax( b, a ) give the same bits as Emscripten's _mm_min_ps( a, b ) /
// _mm_max_ps( a, b ) (a compare and a bitselect), for every pair of special values and for random bit patterns.
// Built with Box2D core's own flags (-O3 -ffast-math -fno-trapping-math -msimd128 -msse2); check.mjs runs it.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <wasm_simd128.h>
#include <xmmintrin.h>

__attribute__( ( noinline ) ) static void viaSse( const float* a, const float* b, float* mn, float* mx, int n )
{
	for ( int i = 0; i < n; i += 4 )
	{
		__m128 x = _mm_loadu_ps( a + i ), y = _mm_loadu_ps( b + i );
		_mm_storeu_ps( mn + i, _mm_min_ps( x, y ) );
		_mm_storeu_ps( mx + i, _mm_max_ps( x, y ) );
	}
}

__attribute__( ( noinline ) ) static void viaPminPmax( const float* a, const float* b, float* mn, float* mx, int n )
{
	for ( int i = 0; i < n; i += 4 )
	{
		v128_t x = wasm_v128_load( a + i ), y = wasm_v128_load( b + i );
		wasm_v128_store( mn + i, wasm_f32x4_pmin( y, x ) );
		wasm_v128_store( mx + i, wasm_f32x4_pmax( y, x ) );
	}
}

#define N 4096
static float A[N], B[N], M1[N], X1[N], M2[N], X2[N];

static uint32_t bits( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

static float fromBits( uint32_t u )
{
	float f;
	memcpy( &f, &u, 4 );
	return f;
}

static int compare( int n, long* checked )
{
	viaSse( A, B, M1, X1, n );
	viaPminPmax( A, B, M2, X2, n );
	for ( int i = 0; i < n; ++i )
	{
		if ( bits( M1[i] ) != bits( M2[i] ) || bits( X1[i] ) != bits( X2[i] ) )
		{
			printf( "differs: a=%08x b=%08x min %08x/%08x max %08x/%08x\n", bits( A[i] ), bits( B[i] ), bits( M1[i] ),
					bits( M2[i] ), bits( X1[i] ), bits( X2[i] ) );
			return 1;
		}
	}
	*checked += n;
	return 0;
}

int main( void )
{
	// +-0, +-1, +-inf, quiet and signalling NaNs of both signs, the smallest subnormal, the largest finite.
	const uint32_t special[] = { 0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u,
								 0x7fc00000u, 0xffc00000u, 0x7fa00000u, 0xffa00001u, 0x00000001u, 0x80000001u,
								 0x7f7fffffu, 0xff7fffffu, 0x3effffffu, 0x3f000000u };
	const int ns = (int)( sizeof( special ) / sizeof( special[0] ) );
	long checked = 0;
	int n = 0;
	for ( int i = 0; i < ns; ++i )
		for ( int j = 0; j < ns; ++j, ++n )
		{
			A[n] = fromBits( special[i] );
			B[n] = fromBits( special[j] );
		}
	while ( n % 4 ) A[n] = B[n] = 0.0f, ++n;
	if ( compare( n, &checked ) ) return 1;

	uint32_t s = 0x9e3779b9u;
	for ( int round = 0; round < 2000; ++round )
	{
		for ( int i = 0; i < N; ++i )
		{
			s ^= s << 13, s ^= s >> 17, s ^= s << 5;
			A[i] = fromBits( s );
			s ^= s << 13, s ^= s >> 17, s ^= s << 5;
			// One lane in four equal, one in eight the same value with the other sign.
			B[i] = ( i & 3 ) == 0 ? A[i] : ( i & 7 ) == 1 ? fromBits( bits( A[i] ) ^ 0x80000000u ) : fromBits( s );
		}
		if ( compare( N, &checked ) ) return 1;
	}
	printf( "pmin/pmax match _mm_min_ps/_mm_max_ps on %ld pairs\n", checked );
	return 0;
}
