cmake_minimum_required(VERSION 2.8.3)
project(DualSLAM_tools) 

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )


link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${PCL_DEFINITIONS})

add_executable(convert_points convert_points.cpp)

target_link_libraries(convert_points
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_executable(fit_hall fit_hall.cpp)

target_link_libraries(fit_hall
    ${PCL_LIBRARIES}
    )

add_executable(merge_hall merge_hall.cpp)

target_link_libraries(merge_hall
    ${PCL_LIBRARIES}
    )

add_executable(visualize visualize.cpp)

target_link_libraries(visualize
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    )

add_executable(visual_pcd visual_pcd.cpp)

target_link_libraries(visual_pcd
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBS}
    )


add_executable(video_strengthening video_strengthening.cpp)

target_link_libraries(video_strengthening
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    )


#add_executable(test_BA test_BA.cpp)

#target_link_libraries(test_BA
#    ${OpenCV_LIBS}
#    ${EIGEN3_LIBS}
#    ${Pangolin_LIBRARIES}
#    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
#    ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
#)
