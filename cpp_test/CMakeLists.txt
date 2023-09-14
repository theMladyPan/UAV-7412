cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject)

# Add the executable
add_executable(MyProject main.cpp)

# If Eigen is in a non-standard location, use find_package or target_include_directories
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(MyProject Eigen3::Eigen)
