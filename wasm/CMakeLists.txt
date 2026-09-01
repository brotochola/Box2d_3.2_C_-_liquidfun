cmake_minimum_required(VERSION 3.22)
project(box2d_wasm C)

set(BOX2D_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# OFF = lab demo (physics_post.js). ON = WeedJS (weed_post.js → importScripts weedjs_post.js).
option(BOX2D_WEED_INTEGRATION "Build post-js for WeedJS multithreadad-game-engine" OFF)

# Box2D solver worker count ceiling; must match -sPTHREAD_POOL_SIZE and wasm_wrapper clamp.
set(BOX2D_PTHREAD_POOL_SIZE "4" CACHE STRING "Emscripten pthread pool / Box2D workerCount max (e.g. 2 or 4)")
# OFF | ON (-flto) | FULL (-flto=full). Default FULL = shipped Weed build.
set(BOX2D_LTO_MODE "FULL" CACHE STRING "LTO mode: OFF, ON (-flto), FULL (-flto=full)")
set_property(CACHE BOX2D_LTO_MODE PROPERTY STRINGS OFF ON FULL)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../box2d ${CMAKE_BINARY_DIR}/box2d)

target_compile_options(box2d PRIVATE -pthread)
target_link_options(box2d PRIVATE -pthread)

add_executable(box2d_wasm
	${CMAKE_CURRENT_SOURCE_DIR}/../box2d/src/wasm_wrapper.c
	${CMAKE_CURRENT_SOURCE_DIR}/../box2d+liquidfun/src/lf_particle_system.c
)
target_include_directories(box2d_wasm PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../box2d+liquidfun/include
)
target_link_libraries(box2d_wasm PRIVATE box2d)

if(BOX2D_WEED_INTEGRATION)
	set(BOX2D_POST_JS "${CMAKE_CURRENT_SOURCE_DIR}/../weedjs/weed_post.js")
else()
	set(BOX2D_POST_JS "${CMAKE_CURRENT_SOURCE_DIR}/../demo/physics_post.js")
endif()

# Weed: fixed 256MB — no growth. HEAP SAB is captured once at WEEDJS_READY and
# hot-rebound into Transform/RigidBody; growth would detach that buffer.
# BallsScene (~9k bodies) high-water ≈ 45MB; 256MB leaves ~5× headroom for
# Predator-scale. Full MAX_ENTITIES (65k) may need more — measure MemoryPanel
# HEAP high-water before raising entity caps that far.
# Lab stays lean at 128MB fixed.
if(BOX2D_WEED_INTEGRATION)
	set(WASM_MEM_FLAGS
		"-sALLOW_MEMORY_GROWTH=0"
		"-sINITIAL_MEMORY=256MB"
	)
else()
	set(WASM_MEM_FLAGS
		"-sALLOW_MEMORY_GROWTH=0"
		"-sINITIAL_MEMORY=128MB"
	)
endif()

set(WASM_LINK_FLAGS
	"-O3"
	"--minify=0"
	"-pthread"
	"-sSHARED_MEMORY=1"
	${WASM_MEM_FLAGS}
	"-sPTHREAD_POOL_SIZE=${BOX2D_PTHREAD_POOL_SIZE}"
	"-sENVIRONMENT=web,worker"
	"-sEXPORTED_FUNCTIONS=['_malloc','_free','_weedjs_heap_bytes_used']"
	"-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPF32','HEAP32']"
	"--post-js=${BOX2D_POST_JS}"
)

target_compile_options(box2d PRIVATE -pthread -msimd128 -msse2 -ffast-math -fno-trapping-math)
target_compile_options(box2d_wasm PRIVATE -pthread -msimd128 -msse2 -ffast-math -fno-trapping-math)
target_compile_definitions(box2d_wasm PRIVATE WASM_PTHREAD_POOL_SIZE=${BOX2D_PTHREAD_POOL_SIZE})

string(TOUPPER "${BOX2D_LTO_MODE}" _BOX2D_LTO_MODE_UPPER)
if(_BOX2D_LTO_MODE_UPPER STREQUAL "FULL")
	message(STATUS "Box2D WASM LTO enabled (-flto=full)")
	target_compile_options(box2d PRIVATE -flto=full)
	target_compile_options(box2d_wasm PRIVATE -flto=full)
	list(APPEND WASM_LINK_FLAGS "-flto=full")
elseif(_BOX2D_LTO_MODE_UPPER STREQUAL "ON" OR _BOX2D_LTO_MODE_UPPER STREQUAL "1" OR _BOX2D_LTO_MODE_UPPER STREQUAL "TRUE")
	message(STATUS "Box2D WASM LTO enabled (-flto)")
	target_compile_options(box2d PRIVATE -flto)
	target_compile_options(box2d_wasm PRIVATE -flto)
	list(APPEND WASM_LINK_FLAGS "-flto")
elseif(_BOX2D_LTO_MODE_UPPER STREQUAL "OFF" OR _BOX2D_LTO_MODE_UPPER STREQUAL "0" OR _BOX2D_LTO_MODE_UPPER STREQUAL "FALSE")
	message(STATUS "Box2D WASM LTO disabled")
else()
	message(FATAL_ERROR "BOX2D_LTO_MODE must be OFF, ON, or FULL (got '${BOX2D_LTO_MODE}')")
endif()

target_link_options(box2d_wasm PRIVATE ${WASM_LINK_FLAGS})

# Lab demo → demo/. Weed build → weedjs/ (gitignored; build_for_weed.bat copies from there).
if(BOX2D_WEED_INTEGRATION)
	set(BOX2D_WASM_OUT_DIR "${CMAKE_SOURCE_DIR}/../weedjs")
else()
	set(BOX2D_WASM_OUT_DIR "${CMAKE_SOURCE_DIR}/../demo")
endif()

set_target_properties(box2d_wasm PROPERTIES
	OUTPUT_NAME "box2d_wasm"
	SUFFIX ".js"
	RUNTIME_OUTPUT_DIRECTORY "${BOX2D_WASM_OUT_DIR}"
)
