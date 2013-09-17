macro(add_gladys_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} gladys)
    add_test(${name} ${name})
endmacro()

add_gladys_test(test_gladys)
add_gladys_test(test_dstar)
add_gladys_test(test_bresenham)
add_gladys_test(test_visibility)
add_gladys_test(test_frontier)

add_executable(bench_visibility bench_visibility.cpp)
target_link_libraries(bench_visibility gladys)
