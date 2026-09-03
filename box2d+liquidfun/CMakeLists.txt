cmake_minimum_required(VERSION 3.22)
project(liquidfun-c C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Use this repo's vendored Box2D 3.2 (WASM/SAB patches included). Do not
# FetchContent erincatto/box2d — that would ignore local solver hooks.
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../box2d ${CMAKE_CURRENT_BINARY_DIR}/box2d)

add_library(liquidfun-c STATIC
	src/lf_particle_system.c
)
target_include_directories(liquidfun-c PUBLIC include)
target_include_directories(liquidfun-c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../box2d/src)
target_link_libraries(liquidfun-c PUBLIC box2d)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
	target_compile_options(liquidfun-c PRIVATE -Wall -Wextra -msse2)
endif()

option(LIQUIDFUN_C_BUILD_EXAMPLES "Build the liquidfun-c example programs" ON)
if(LIQUIDFUN_C_BUILD_EXAMPLES)
	add_executable(demo_water examples/demo_water.c)
	target_link_libraries(demo_water PRIVATE liquidfun-c)

	add_executable(demo_elastic examples/demo_elastic.c)
	target_link_libraries(demo_elastic PRIVATE liquidfun-c)

	add_executable(demo_parallel examples/demo_parallel.c)
	target_link_libraries(demo_parallel PRIVATE liquidfun-c)

	if(EMSCRIPTEN)
		target_link_options(demo_water PRIVATE -sNODERAWFS=1)
		target_link_options(demo_elastic PRIVATE -sNODERAWFS=1)
		target_link_options(demo_parallel PRIVATE -sNODERAWFS=1 -pthread -sPTHREAD_POOL_SIZE=2)
	endif()
endif()
