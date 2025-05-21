# Python bindings specific CMake configuration
cmake_minimum_required(VERSION 3.10)

# Find Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python binding module
pybind11_add_module(_tf_lcm_py tf_lcm_pybind.cpp)
target_link_libraries(_tf_lcm_py PRIVATE tf_lcm)

# Installation paths for Python module
if(DEFINED PYTHON_INSTALL_DIR)
  # Custom install location provided
  set(PYTHON_MODULE_INSTALL_DIR ${PYTHON_INSTALL_DIR})
else()
  # Default to Python's site-packages
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(PYTHON_MODULE_INSTALL_DIR ${PYTHON_SITE_PACKAGES}/tf_lcm_py)
endif()

# Install Python module
install(TARGETS _tf_lcm_py 
  DESTINATION ${PYTHON_MODULE_INSTALL_DIR}
)

# Install Python package files
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/tf_lcm_py/__init__.py
  DESTINATION ${PYTHON_MODULE_INSTALL_DIR}
)

# Install examples
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/tf_lcm/examples
  FILES_MATCHING PATTERN "*.py"
)

# Create a Python wheel package
add_custom_target(python_wheel
  COMMAND ${Python3_EXECUTABLE} -m pip wheel --no-deps -w ${CMAKE_BINARY_DIR}/dist ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building Python wheel package"
)

# Add an easy install target
add_custom_target(install_python
  COMMAND ${Python3_EXECUTABLE} -m pip install --user ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Installing tf_lcm_py package locally"
)
