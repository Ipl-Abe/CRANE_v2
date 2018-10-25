add_library(greetings SHARED sample.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../exe)
add_executable(sample sample.cpp)