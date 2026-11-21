# The Arduino library: dist/arduino/Rant_Math/ and dist/math-<version>-arduino.zip, which Arduino and
# PlatformIO take as they are. rant_math.cpp is its one implementation file.
#   cmake -D DIST=<the folder holding rant_math.h> -P tools/arduino.cmake

cmake_minimum_required(VERSION 3.18)

get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
file(STRINGS "${ROOT}/VERSION" VERSION LIMIT_COUNT 1)
if(NOT DEFINED DIST)
  set(DIST "${ROOT}/build/dist")
endif()
set(STAGE "${ROOT}/dist/arduino")
set(OUT "${STAGE}/Rant_Math")

file(REMOVE_RECURSE "${STAGE}")
foreach(f rant_math.h rant_math.hpp rant_math_members.hpp rant_math.cpp)
  if(NOT EXISTS "${DIST}/${f}")
    message(FATAL_ERROR "arduino: no ${DIST}/${f}, build first or set DIST")
  endif()
  file(COPY "${DIST}/${f}" DESTINATION "${OUT}/src")
endforeach()
file(COPY "${ROOT}/licenses" DESTINATION "${OUT}")
file(CONFIGURE OUTPUT "${OUT}/library.properties" CONTENT "name=Rant Math
version=@VERSION@
author=KosmosisDire
maintainer=KosmosisDire
sentence=Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.
paragraph=It carries Rant's types, so it works with Rant or without it. Include rant_math.hpp for C++ or rant_math.h for C.
category=Data Processing
url=https://github.com/KosmosisDire/rant-math
architectures=*
includes=rant_math.hpp
" @ONLY NEWLINE_STYLE LF)

set(ZIP "${ROOT}/dist/math-${VERSION}-arduino.zip")
file(REMOVE "${ZIP}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E tar cf "${ZIP}" --format=zip Rant_Math
  WORKING_DIRECTORY "${STAGE}" RESULT_VARIABLE failed)
if(failed)
  message(FATAL_ERROR "arduino: cannot write ${ZIP}")
endif()
message(STATUS "arduino: ${ZIP}")
