# Runs the record program, then compares its answers with the checked in vectors.
execute_process(COMMAND "${RECORD}" "${ANSWERS}" RESULT_VARIABLE recorded)
if(NOT recorded EQUAL 0)
  message(FATAL_ERROR "the record program failed: ${recorded}")
endif()
execute_process(COMMAND "${PYTHON}" "${GEN}" --types "${TYPES}" --record-answers "${ANSWERS}" --check-vectors RESULT_VARIABLE same)
if(NOT same EQUAL 0)
  message(FATAL_ERROR "the library's answers differ from tests/bindings/vectors.json")
endif()
