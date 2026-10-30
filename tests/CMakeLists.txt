# One program holds every part's tests, and ctest runs each part on its own.
set(RANT_MATH_PARTS vector rotation matrix pose motion axes line region solid curve surface points shape fit color camera image cloud neighbors grid geo detection)

set(test_sources main.c)
foreach(part ${RANT_MATH_PARTS})
  list(APPEND test_sources "${part}.c")
endforeach()

add_executable(rant_math_test ${test_sources} check.h reference.h)
target_link_libraries(rant_math_test PRIVATE rant_math)
target_include_directories(rant_math_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(rant_math_test PRIVATE ${RANT_MATH_WARNINGS})

foreach(part ${RANT_MATH_PARTS})
  add_test(NAME ${part} COMMAND rant_math_test ${part})
endforeach()

# The implementation beside Rant's rant.h in each order, compiled only.
foreach(order first after)
  add_library(rant_math_with_rant_${order} OBJECT with_rant.c)
  add_dependencies(rant_math_with_rant_${order} rant_math_dist)
  target_include_directories(rant_math_with_rant_${order} PRIVATE "${RANT_MATH_DIST}")
  target_link_libraries(rant_math_with_rant_${order} PRIVATE rant)
  target_compile_options(rant_math_with_rant_${order} PRIVATE ${RANT_MATH_WARNINGS})
endforeach()
target_compile_definitions(rant_math_with_rant_first PRIVATE RANT_FIRST)
