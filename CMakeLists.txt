cmake_minimum_required(VERSION 3.16)
project(softflight C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(SANITIZE "Enable address/undefined sanitizers in Debug" OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()


if(MSVC)
  add_compile_options(/W4)
else()
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
  elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-O2 -g)
  else()
    add_compile_options(-O0 -g)
  endif()
  add_compile_options(-Wall -Wextra -Wshadow -Wconversion -Wdouble-promotion -Wno-unused-parameter)
endif()


find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

file(GLOB SRC_FILES
    src/*.c
)

add_executable(softflight ${SRC_FILES})
target_link_libraries(softflight ${SDL2_LIBRARIES})

enable_testing()
add_executable(tests tests/test_math.c tests/test_fdm.c tests/test_raster.c src/fdm.c src/raster.c src/renderer.c src/terrain.c src/platform_sdl.c)
target_link_libraries(tests ${SDL2_LIBRARIES})
add_test(NAME unit_tests COMMAND tests)
