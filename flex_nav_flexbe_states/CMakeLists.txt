cmake_minimum_required(VERSION 3.0)
project(flex_nav_flexbe_states)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options
catkin_package(
    LIBRARIES ${PROJECT_NAME})

# install executables
install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tests  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# run tests
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(tests/run_rostest.test)
endif()
