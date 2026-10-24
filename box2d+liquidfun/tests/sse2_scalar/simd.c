// liquidfun-c - tests/sse2_scalar/simd.c
// SPDX-License-Identifier: Zlib
//
// The cases on Emscripten's SSE2, compiled with -msimd128 -msse2: the reference.

#include <emmintrin.h>

#include "cases.h"

#define X( name, domain, kind, body ) \
	void simd_##name( const float* a, const float* b, float* out ) \
	{ \
		body; \
	}
LF_SSE2_CASES( X )
