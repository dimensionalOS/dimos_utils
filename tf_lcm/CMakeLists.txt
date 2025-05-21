cmake_minimum_required(VERSION 3.10)
project(tf_lcm)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find package LCM
find_package(lcm REQUIRED)

# Add pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# Include directories - Fix the include paths
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_lcm_msgs  # Directory containing message headers
)

# Add library
add_library(tf_lcm
  src/transform_storage.cpp
  src/buffer.cpp
  src/broadcaster.cpp
  src/listener.cpp
)

# Link against LCM
target_link_libraries(tf_lcm lcm lcm-coretypes)

# Add Python bindings
pybind11_add_module(_tf_lcm_py python/tf_lcm_pybind.cpp)
target_link_libraries(_tf_lcm_py PRIVATE tf_lcm)

# Install targets
install(TARGETS tf_lcm _tf_lcm_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/python3/site-packages/tf_lcm_py
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Add tests
add_subdirectory(test)
