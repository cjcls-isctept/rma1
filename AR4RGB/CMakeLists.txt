cmake_minimum_required( VERSION 2.6 ) 
project( AR4RGBD )

find_package( OpenThreads ) 
find_package( osg ) 
find_package( osgDB ) 
find_package( osgUtil ) 
find_package( osgViewer )
find_package( osgShadow )
find_package( osgGA )
find_package(PCL 1.6 COMPONENTS)

set(CMAKE_CXX_FLAGS "-g -Wno-deprecated")

macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} ) 
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY})
endmacro()

add_executable(ar4rgbd ar4rgbd.cpp ) 

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(ar4rgbd ${PCL_LIBRARIES})

config_project(ar4rgbd OPENTHREADS ) 
config_project(ar4rgbd OSG ) 
config_project(ar4rgbd OSGDB ) 
config_project(ar4rgbd OSGGA ) 
config_project(ar4rgbd OSGUTIL ) 
config_project(ar4rgbd OSGVIEWER )

