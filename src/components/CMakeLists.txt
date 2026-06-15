add_library(components_lib INTERFACE)

target_include_directories(components_lib 
	INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
