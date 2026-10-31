# Rant Math: the C library over Rant's standard types, its tests and its single header.
# The only thing it takes from Rant is rant_types.h, from the Rant CPM fetches.

cmake_minimum_required(VERSION 3.15)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" RANT_MATH_VERSION LIMIT_COUNT 1)
project(rant_math VERSION ${RANT_MATH_VERSION} LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# A program that links Rant too shares its Rant with this. The local presets set
# CPM_rant_SOURCE to ../rant.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/get_cpm.cmake")
CPMAddPackage(NAME rant
  GITHUB_REPOSITORY KosmosisDire/Rant
  VERSION 0.0.41
  EXCLUDE_FROM_ALL YES
  SYSTEM YES)
set(RANT_MATH_TYPES "${rant_SOURCE_DIR}/src/common/stdtypes.h")
set(RANT_MATH_TYPES_HPP "${rant_SOURCE_DIR}/bindings/cpp/rant_types.hpp")
if(NOT EXISTS "${RANT_MATH_TYPES}" OR NOT EXISTS "${RANT_MATH_TYPES_HPP}")
  message(FATAL_ERROR "Rant at ${rant_SOURCE_DIR} has no src/common/stdtypes.h, Rant Math needs Rant 0.0.41 or newer")
endif()

# 1. Pack src/ and the types into dist/rant_math.h in the build tree, which depends on the
#    Rant it was packed with. Configure packs once, the build packs again when an input changes.
set(RANT_MATH_DIST "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(RANT_MATH_OUTPUTS "${RANT_MATH_DIST}/rant_math.h" "${RANT_MATH_DIST}/rant_math.c"
    "${RANT_MATH_DIST}/rant_math.hpp" "${RANT_MATH_DIST}/rant_math.cpp")
file(GLOB RANT_MATH_SRC CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
     "${CMAKE_CURRENT_SOURCE_DIR}/bindings/cpp/*")
set(RANT_MATH_PACK "${CMAKE_COMMAND}" -D "TYPES=${RANT_MATH_TYPES}" -D "TYPES_HPP=${RANT_MATH_TYPES_HPP}"
    -D "OUT=${RANT_MATH_DIST}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools/pack.cmake")
foreach(out IN LISTS RANT_MATH_OUTPUTS)
  if(NOT EXISTS "${out}")
    execute_process(COMMAND ${RANT_MATH_PACK} RESULT_VARIABLE packed)
    if(NOT packed EQUAL 0)
      message(FATAL_ERROR "Rant Math: packing src/ failed")
    endif()
    break()
  endif()
endforeach()
add_custom_command(
  OUTPUT  ${RANT_MATH_OUTPUTS}
  COMMAND ${RANT_MATH_PACK}
  DEPENDS ${RANT_MATH_SRC} "${RANT_MATH_TYPES}" "${RANT_MATH_TYPES_HPP}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/pack.cmake"
          "${CMAKE_CURRENT_SOURCE_DIR}/VERSION"
  COMMENT "Rant Math: packing src/ into dist/rant_math.h and rant_math.hpp"
  VERBATIM)
add_custom_target(rant_math_dist ALL DEPENDS ${RANT_MATH_OUTPUTS})

if(MSVC)
  set(RANT_MATH_WARNINGS /W4 /wd4204)   # 4204: a struct initialized from locals is C99
  set(RANT_MATH_WARNING_IS_ERROR /WX)
else()
  set(RANT_MATH_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  set(RANT_MATH_WARNING_IS_ERROR -Werror)
endif()
# Only our own builds stop at a warning, since a project taking this one may have a newer compiler.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  list(APPEND RANT_MATH_WARNINGS ${RANT_MATH_WARNING_IS_ERROR})
endif()

# 2. The libraries, built from the single header so they hold exactly what it ships.
function(rant_math_target name kind)
  add_library(${name} ${kind} "${RANT_MATH_DIST}/rant_math.c")
  add_dependencies(${name} rant_math_dist)
  target_include_directories(${name} PUBLIC "${RANT_MATH_DIST}")
  target_compile_options(${name} PRIVATE ${RANT_MATH_WARNINGS})
  if(UNIX)
    target_link_libraries(${name} PUBLIC m)
  endif()
endfunction()

rant_math_target(rant_math STATIC)
rant_math_target(rant_math_shared SHARED)
target_compile_definitions(rant_math_shared PRIVATE RANT_MATH_BUILD_SHARED INTERFACE RANT_MATH_LINK_SHARED)
# Its own folder keeps a Windows import library apart from the static rant_math.lib.
set_target_properties(rant_math_shared PROPERTIES OUTPUT_NAME rant_math C_VISIBILITY_PRESET hidden
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/shared")

# A wheel build (scikit-build-core) wants the library next to the package, with a private copy of
# Rant's Python types for a process with no Rant installed.
if(SKBUILD)
  install(TARGETS rant_math_shared RUNTIME DESTINATION rant_math LIBRARY DESTINATION rant_math)
  install(DIRECTORY "${rant_SOURCE_DIR}/bindings/python/rant_types/" DESTINATION rant_math/_rant_types
          FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
elseif(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  enable_testing()
  add_subdirectory(tests)
endif()
