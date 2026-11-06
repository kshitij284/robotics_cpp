add_library(components_lib STATIC
	urdf/urdf_parser.cpp
)
target_include_directories(components_lib 
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(components_lib 
	PUBLIC eigen_headers
	PRIVATE tinyxml2::tinyxml2)

