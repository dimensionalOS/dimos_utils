add_executable(test_broadcaster test_broadcaster.cpp)
target_link_libraries(test_broadcaster tf_lcm)

add_executable(test_listener test_listener.cpp)
target_link_libraries(test_listener tf_lcm)

add_executable(test_lookup test_lookup.cpp)
target_link_libraries(test_lookup tf_lcm)

add_executable(test_static_transform test_static_transform.cpp)
target_link_libraries(test_static_transform tf_lcm)

add_executable(test_timeout test_timeout.cpp)
target_link_libraries(test_timeout tf_lcm)

add_executable(test_time_travel test_time_travel.cpp)
target_link_libraries(test_time_travel tf_lcm)

# Simple TF demo that demonstrates core functionality
add_executable(simple_tf_demo simple_tf_demo.cpp)
target_link_libraries(simple_tf_demo tf_lcm)

# Minimal test for core functionality
add_executable(minimal_test minimal_test.cpp)
target_link_libraries(minimal_test tf_lcm)

# Chained transform test for graph traversal
add_executable(chained_test chained_test.cpp)
target_link_libraries(chained_test tf_lcm)

# Time travel demo test
add_executable(time_travel_demo time_travel_demo.cpp)
target_link_libraries(time_travel_demo tf_lcm)

# Robot link lookup test
add_executable(robot_link_lookup_test robot_link_lookup_test.cpp)
target_link_libraries(robot_link_lookup_test tf_lcm)

# Transform benchmark test
add_executable(transform_benchmark transform_benchmark.cpp)
target_link_libraries(transform_benchmark tf_lcm)

# Time travel benchmark test
add_executable(time_travel_benchmark time_travel_benchmark.cpp)
target_link_libraries(time_travel_benchmark tf_lcm)
