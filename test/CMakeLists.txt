add_executable( test_gladys test_gladys.cpp )
target_link_libraries( test_gladys gladys )
add_test( gladys test_gladys )
