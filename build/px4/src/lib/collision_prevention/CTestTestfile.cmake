# CMake generated Testfile for 
# Source directory: /home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/src/lib/collision_prevention
# Build directory: /home/sebastian-elliott-pedrosa/T1GR3/build/px4/src/lib/collision_prevention
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(functional-CollisionPrevention "/home/sebastian-elliott-pedrosa/T1GR3/build/px4/functional-CollisionPrevention")
set_tests_properties(functional-CollisionPrevention PROPERTIES  _BACKTRACE_TRIPLES "/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/cmake/gtest/px4_add_gtest.cmake;125;add_test;/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/src/lib/collision_prevention/CMakeLists.txt;42;px4_add_functional_gtest;/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/src/lib/collision_prevention/CMakeLists.txt;0;")
add_test(unit-ObstacleMath "/home/sebastian-elliott-pedrosa/T1GR3/build/px4/unit-ObstacleMath")
set_tests_properties(unit-ObstacleMath PROPERTIES  WORKING_DIRECTORY "/home/sebastian-elliott-pedrosa/T1GR3/build/px4" _BACKTRACE_TRIPLES "/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/cmake/gtest/px4_add_gtest.cmake;71;add_test;/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/src/lib/collision_prevention/CMakeLists.txt;43;px4_add_unit_gtest;/home/sebastian-elliott-pedrosa/T1GR3/PX4-Autopilot/src/lib/collision_prevention/CMakeLists.txt;0;")
