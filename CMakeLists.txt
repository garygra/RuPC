cmake_minimum_required(VERSION 3.10)
project(RuPC)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0072 NEW)
# TODO: go back to c++11?
add_definitions(-O3 -fPIC -g )
if(APPLE)
	set(CMAKE_CXX_STANDARD 14)
else()
	add_definitions(-lstdc++fs)
	link_libraries(stdc++fs)
endif()

find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

link_libraries(
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	# ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
	)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(realsense2 REQUIRED)

find_package(OpenCV CONFIG REQUIRED) 

# find_package(PkgConfig REQUIRED)
# pkg_search_module(GLFW REQUIRED glfw3)
# include_directories(${GLFW_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

add_subdirectory(executables)

file(MAKE_DIRECTORY out)

