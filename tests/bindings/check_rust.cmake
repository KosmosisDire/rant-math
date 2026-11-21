# The Rust crate compiles the packed C from csrc, which a release tag holds and a checkout does not.
file(MAKE_DIRECTORY "${CRATE}/csrc")
file(COPY "${DIST}/rant_math.h" "${DIST}/rant_math.c" DESTINATION "${CRATE}/csrc")
execute_process(COMMAND "${CARGO}" test --release --features "nalgebra quickjs" WORKING_DIRECTORY "${CRATE}" RESULT_VARIABLE tested)
if(NOT tested EQUAL 0)
  message(FATAL_ERROR "the Rust tests failed")
endif()
