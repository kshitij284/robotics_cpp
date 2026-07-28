add_library(components_lib INTERFACE)
target_include_directories(components_lib 
	INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(components_lib INTERFACE Eigen3::Eigen)
target_include_directories(components_lib SYSTEM INTERFACE
    ${eigen_SOURCE_DIR})
