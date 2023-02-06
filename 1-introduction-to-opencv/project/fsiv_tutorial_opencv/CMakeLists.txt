CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(minitutorial_opencv)
ENABLE_LANGUAGE(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

FIND_PACKAGE(OpenCV REQUIRED )
LINK_LIBRARIES(${OpenCV_LIBS})
include_directories ("${OpenCV_INCLUDE_DIRS}")

add_executable(esqueleto esqueleto.cc)
add_executable(show_img show_img.cc)
add_executable(show_video show_video.cc)
add_executable(comp_stats comp_stats.cc)

#add_executable(pract1 pract1.cc)
#add_executable(pract2 pract2.cc)
#...

 
