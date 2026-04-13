cmake_minimum_required(VERSION 3.8)
project(defined_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (defined_bringup/platform_interface.py)
# This makes `from defined_bringup import ...` work inside the container
# without any sys.path manipulation.
ament_python_install_package(${PROJECT_NAME})

# Install launch files and config (share directory)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
