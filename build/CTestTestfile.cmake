# CMake generated Testfile for 
# Source directory: /home/analys/geo_ws/src/mavros_controllers/geometric_controller
# Build directory: /home/analys/geo_ws/src/mavros_controllers/geometric_controller/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_geometric_controller_gtest_geometric_controller-test "/home/analys/geo_ws/src/mavros_controllers/geometric_controller/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/analys/geo_ws/src/mavros_controllers/geometric_controller/build/test_results/geometric_controller/gtest-geometric_controller-test.xml" "--return-code" "/home/analys/geo_ws/src/mavros_controllers/geometric_controller/build/devel/lib/geometric_controller/geometric_controller-test --gtest_output=xml:/home/analys/geo_ws/src/mavros_controllers/geometric_controller/build/test_results/geometric_controller/gtest-geometric_controller-test.xml")
set_tests_properties(_ctest_geometric_controller_gtest_geometric_controller-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/analys/geo_ws/src/mavros_controllers/geometric_controller/CMakeLists.txt;67;catkin_add_gtest;/home/analys/geo_ws/src/mavros_controllers/geometric_controller/CMakeLists.txt;0;")
subdirs("gtest")
