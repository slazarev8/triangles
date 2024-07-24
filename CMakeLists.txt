 
cmake_minimum_required (VERSION 3.8)

project ("Triangles")

# Add source to this project's executable.
add_executable (Triangles "main.cpp" "intersections.cpp" "tests_runner.cpp" "geometry_utils.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET Triangles PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
