# The bindings: the shared vectors, the check that every generated file is current, and C++.
find_package(Python3 COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
  message(STATUS "Rant Math: no Python, so the binding checks are left out")
  return()
endif()

set(GEN "${CMAKE_SOURCE_DIR}/tools/bindgen/gen.py")
set(VECTORS "${CMAKE_CURRENT_SOURCE_DIR}/vectors.json")
file(GLOB BINDGEN_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/tools/bindgen/*.py" "${CMAKE_SOURCE_DIR}/src/*.h")

add_test(NAME bindings.current COMMAND "${Python3_EXECUTABLE}" "${GEN}" --types "${RANT_MATH_TYPES}" --check)

# The C program that asks the library every case. Its answers must match the checked in vectors.
set(RECORD_C "${CMAKE_CURRENT_BINARY_DIR}/record.c")
add_custom_command(OUTPUT "${RECORD_C}"
  COMMAND "${Python3_EXECUTABLE}" "${GEN}" --types "${RANT_MATH_TYPES}" --record-source "${RECORD_C}"
  DEPENDS ${BINDGEN_SOURCES}
  VERBATIM)
add_executable(rant_math_record "${RECORD_C}")
target_link_libraries(rant_math_record PRIVATE rant_math)
if(MSVC)
  target_compile_options(rant_math_record PRIVATE /W0 /bigobj)
else()
  target_compile_options(rant_math_record PRIVATE -w)
endif()
add_test(NAME bindings.vectors COMMAND "${CMAKE_COMMAND}"
  "-DPYTHON=${Python3_EXECUTABLE}" "-DGEN=${GEN}" "-DTYPES=${RANT_MATH_TYPES}"
  "-DRECORD=$<TARGET_FILE:rant_math_record>" "-DANSWERS=${CMAKE_CURRENT_BINARY_DIR}/answers.txt"
  -P "${CMAKE_CURRENT_SOURCE_DIR}/check_vectors.cmake")

# Python over the shared library, with Rant's checkout for the standard types.
add_test(NAME bindings.python COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/python_test.py" "${VECTORS}")
set(PYTHON_PATHS "${CMAKE_SOURCE_DIR}/bindings/python" "${rant_SOURCE_DIR}/bindings/python")
if(WIN32)
  string(REPLACE ";" "\\;" PYTHON_PATHS "${PYTHON_PATHS}")
else()
  string(REPLACE ";" ":" PYTHON_PATHS "${PYTHON_PATHS}")
endif()
set_tests_properties(bindings.python PROPERTIES ENVIRONMENT
  "PYTHONPATH=${PYTHON_PATHS};RANT_MATH_LIBRARY=$<TARGET_FILE:rant_math_shared>")

# C# over the shared library.
find_program(RANT_MATH_DOTNET dotnet)
if(RANT_MATH_DOTNET)
  add_test(NAME bindings.csharp COMMAND "${RANT_MATH_DOTNET}" run --project "${CMAKE_CURRENT_SOURCE_DIR}/csharp/Test.csproj" -c Release
           "-p:NativeLibrary=$<TARGET_FILE:rant_math_shared>" "-p:RantDir=${rant_SOURCE_DIR}" -- "${VECTORS}")
  set_tests_properties(bindings.csharp PROPERTIES TIMEOUT 600)
endif()

# Rust and its QuickJS glue through cargo.
find_program(RANT_MATH_CARGO cargo)
if(RANT_MATH_CARGO)
  add_test(NAME bindings.rust COMMAND "${CMAKE_COMMAND}" "-DCARGO=${RANT_MATH_CARGO}"
           "-DCRATE=${CMAKE_SOURCE_DIR}/bindings/rust" "-DDIST=${RANT_MATH_DIST}" -P "${CMAKE_CURRENT_SOURCE_DIR}/check_rust.cmake")
  set_tests_properties(bindings.rust PROPERTIES TIMEOUT 900)
endif()

# TypeScript over wasm, once bindings/web/build.sh has built it, which takes the emsdk.
find_program(RANT_MATH_NODE node)
if(RANT_MATH_NODE AND EXISTS "${CMAKE_SOURCE_DIR}/bindings/web/dist/index.js")
  add_test(NAME bindings.web COMMAND "${RANT_MATH_NODE}" "${CMAKE_CURRENT_SOURCE_DIR}/web_test.mjs" "${VECTORS}")
endif()

include(CheckLanguage)
check_language(CXX)
if(NOT CMAKE_CXX_COMPILER)
  return()
endif()
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rant_math_cpp_test cpp_test.cpp "${RANT_MATH_DIST}/rant_math.cpp")
add_dependencies(rant_math_cpp_test rant_math_dist)
target_include_directories(rant_math_cpp_test PRIVATE "${RANT_MATH_DIST}" "${CMAKE_CURRENT_SOURCE_DIR}")
if(MSVC)
  target_compile_options(rant_math_cpp_test PRIVATE /W4 /WX /bigobj /wd4244 /wd4267)
else()
  target_compile_options(rant_math_cpp_test PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
  target_link_libraries(rant_math_cpp_test PRIVATE m)
endif()
add_test(NAME bindings.cpp COMMAND rant_math_cpp_test "${VECTORS}")

# rant_math.hpp over the library built as C.
add_executable(rant_math_cpp_linked cpp_linked.cpp)
target_link_libraries(rant_math_cpp_linked PRIVATE rant_math)
add_test(NAME bindings.cpp.linked COMMAND rant_math_cpp_linked)

# Both implementations in one file, compiled only.
foreach(order RANT_FIRST MATH_FIRST)
  string(TOLOWER "${order}" lower)
  add_library(rant_math_cpp_both_${lower} OBJECT cpp_both_built.cpp)
  add_dependencies(rant_math_cpp_both_${lower} rant_math_dist rant_dist)
  target_include_directories(rant_math_cpp_both_${lower} PRIVATE "${RANT_MATH_DIST}" "${rant_SOURCE_DIR}/dist")
  target_compile_definitions(rant_math_cpp_both_${lower} PRIVATE ${order})
endforeach()

# rant_math.hpp beside rant.hpp in each order, compiled only, and the implementation on its own.
set(RANT_CPP_INCLUDE "${rant_SOURCE_DIR}/dist")
foreach(order RANT_FIRST MATH_FIRST TYPES_FIRST ALONE)
  string(TOLOWER "${order}" lower)
  add_library(rant_math_cpp_${lower} OBJECT cpp_with_rant.cpp)
  add_dependencies(rant_math_cpp_${lower} rant_math_dist rant_dist)
  target_include_directories(rant_math_cpp_${lower} PRIVATE "${RANT_MATH_DIST}" "${RANT_CPP_INCLUDE}")
  target_compile_definitions(rant_math_cpp_${lower} PRIVATE ${order})
endforeach()
