cmake_minimum_required(VERSION 3.20)
project(robotics_cpp
	VERSION 0.1.0
	DESCRIPTION "Robotics essential in cpp"
	LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)

include(FetchContent)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Eigen: header only
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(Eigen)
if(NOT eigen_POPULATED)
	FetchContent_MakeAvailable(Eigen)
endif()
add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers SYSTEM INTERFACE ${eigen_SOURCE_DIR})

# tinyxml2
FetchContent_Declare(
  tinyxml2
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
  GIT_TAG        11.0.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(tinyxml2)

add_subdirectory(src/components)

if(PROJECT_IS_TOP_LEVEL)
  FetchContent_Declare(
    googletest
    URL                      https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
  add_subdirectory(tests/components)
endif()
