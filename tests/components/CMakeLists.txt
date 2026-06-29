add_executable(test_components
	test_quaternion.cpp
)

target_link_libraries(test_components
	PRIVATE
	components_lib
	GTest::gtest_main
)

target_set_warnings(test_components PRIVATE)
target_enable_sanitizers(test_components)

gtest_discover_tests(test_components
    PROPERTIES TIMEOUT 30
)
