add_executable(test_components
	#[[ math_test/test_vec3.cpp
	math_test/test_mat3.cpp
	math_test/test_quaternion.cpp
	math_test/test_se3.cpp
	kinematics_test/test_joint.cpp
	kinematics_test/test_chain.cpp
	kinematics_test/test_jacobian.cpp
	kinematics_test/test_iksolver.cpp
	]]
	urdf_test/test_urdf.cpp
	
)

target_compile_definitions(test_components PRIVATE 
    TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/urdf_test/resources/"
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
