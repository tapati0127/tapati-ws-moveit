# CMake generated Testfile for 
# Source directory: /home/tapati/my_ws/src/motoman_motomini_support
# Build directory: /home/tapati/my_ws/build/motoman_motomini_support
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_motoman_motomini_support_roslaunch-check_test_launch_test.xml "/home/tapati/my_ws/build/motoman_motomini_support/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/tapati/my_ws/build/motoman_motomini_support/test_results/motoman_motomini_support/roslaunch-check_test_launch_test.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/tapati/my_ws/build/motoman_motomini_support/test_results/motoman_motomini_support" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/tapati/my_ws/build/motoman_motomini_support/test_results/motoman_motomini_support/roslaunch-check_test_launch_test.xml.xml\" \"/home/tapati/my_ws/src/motoman_motomini_support/test/launch_test.xml\" ")
subdirs("gtest")
