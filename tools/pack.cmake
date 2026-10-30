# Packs src/rant_math.h with each local include written in place once, Rant's types among them,
# then every src/*.c under RANT_MATH_IMPLEMENTATION, into OUT/rant_math.h and its anchor.

cmake_minimum_required(VERSION 3.15)

if(NOT DEFINED SRC)
  set(SRC "${CMAKE_CURRENT_LIST_DIR}/../src")
endif()
if(NOT DEFINED TYPES OR NOT EXISTS "${TYPES}")
  message(FATAL_ERROR "pack: TYPES must name Rant's src/common/stdtypes.h")
endif()
if(NOT DEFINED OUT)
  message(FATAL_ERROR "pack: OUT must name the folder to write")
endif()
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/../VERSION" version LIMIT_COUNT 1)

set(header "${OUT}/rant_math.h")
set(seen "")

# Appends a file to the header with each local include at a line start replaced by that file,
# the first time only. rant_types.h is Rant's, read from TYPES.
function(pack_emit name)
  if(name IN_LIST seen)
    return()
  endif()
  set(seen ${seen} "${name}" PARENT_SCOPE)
  list(APPEND seen "${name}")
  if(name STREQUAL "rant_types.h")
    file(READ "${TYPES}" c)
  else()
    file(READ "${SRC}/${name}" c)
  endif()
  string(REPLACE "\r" "" c "\n${c}")
  file(APPEND "${header}" "\n/* ${name} */")
  while(TRUE)
    string(REGEX MATCH "\n[ \t]*#include[ \t]*\"([^\"\n]*)\"[^\n]*" line "${c}")
    if(line STREQUAL "")
      break()
    endif()
    set(inner "${CMAKE_MATCH_1}")
    string(FIND "${c}" "${line}" at)
    string(SUBSTRING "${c}" 0 ${at} before)
    string(LENGTH "${line}" skip)
    math(EXPR at "${at} + ${skip}")
    string(SUBSTRING "${c}" ${at} -1 c)
    file(APPEND "${header}" "${before}\n")
    pack_emit("${inner}")
    set(seen ${seen} PARENT_SCOPE)
  endwhile()
  if(NOT c MATCHES "\n$")
    set(c "${c}\n")
  endif()
  file(APPEND "${header}" "${c}")
endfunction()

file(MAKE_DIRECTORY "${OUT}")
file(WRITE "${header}" [==[
/* GENERATED from src/ by tools/pack.cmake. Edit src/ instead.
 * Rant Math, math on Rant's standard types. Define RANT_MATH_IMPLEMENTATION in one file. */
]==])
file(APPEND "${header}" "#define RANT_MATH_VERSION \"${version}\"\n")
pack_emit(rant_math.h)

file(GLOB parts RELATIVE "${SRC}" "${SRC}/*.c")
list(SORT parts)
file(APPEND "${header}" "\n#if defined(RANT_MATH_IMPLEMENTATION) && !defined(RANT_MATH_IMPLEMENTED)\n#define RANT_MATH_IMPLEMENTED\n")
foreach(part ${parts})
  pack_emit("${part}")
endforeach()
file(APPEND "${header}" "\n#endif /* RANT_MATH_IMPLEMENTATION */\n")

file(WRITE "${OUT}/rant_math.c" [==[
/* GENERATED. The one file that holds the implementation. */
#define RANT_MATH_IMPLEMENTATION
#include "rant_math.h"
]==])
message(STATUS "pack: wrote ${header}")
