cmake_minimum_required(VERSION 3.0.0)
project(opencvProject VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_library( WIRINGPI_LIBRARIES NAMES wiringPi )

link_directories( ${OpenCV_LIBRARY_DIRS} )

find_package( OpenMP )
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_executable(opencvProject main.cpp crc.cpp)

target_link_libraries( opencvProject ${OpenCV_LIBS} )
target_link_libraries( opencvProject ${WIRINGPI_LIBRARIES} )
target_include_directories( opencvProject PUBLIC /usr/local/include/opencv4 )

#target_link_libraries(testDetect PRIVATE OpenMP::OpenMP_CXX)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)