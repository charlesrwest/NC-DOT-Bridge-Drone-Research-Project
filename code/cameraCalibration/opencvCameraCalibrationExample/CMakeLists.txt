cmake_minimum_required (VERSION 2.8.3)
PROJECT(test)

#Get c++11
ADD_DEFINITIONS(-std=c++11)

FILE(GLOB SOURCEFILES *.cpp *.c)

message( ${SOURCEFILES} )

#set path to library
link_directories(/usr/lib/x86_64-linux-gnu)


#Add the compilation target
ADD_EXECUTABLE(cameraCalibration ${SOURCEFILES})

#link libraries to executable
target_link_libraries(cameraCalibration opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_features2d)
