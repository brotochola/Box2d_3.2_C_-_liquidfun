// liquidfun-c - tests/sse2_scalar/scalar.c
// SPDX-License-Identifier: Zlib
//
// The cases on lf_sse2_scalar.h, compiled without SIMD, as in the no-SIMD build.

#include "lf_sse2_scalar.h"

#include "cases.h"

#define X( name, domain, kind, body ) \
	void scalar_##name( const float* a, const float* b, float* out ) \
	{ \
		body; \
	}
LF_SSE2_CASES( X )
