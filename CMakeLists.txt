cmake_minimum_required(VERSION 3.10)
project(OpenCV_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR})

add_library(serialib src/serialib.cpp)
add_library(rapidcsv INTERFACE)

add_executable(test_cam src/test_cam.cpp)
target_include_directories(test_cam PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_cam PRIVATE ${OpenCV_LIBS})

add_executable(stream src/stream.cpp)
target_include_directories(stream PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stream PRIVATE ${OpenCV_LIBS})

add_executable(vo src/VisualOdometry.cpp)
target_include_directories(vo PRIVATE ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
target_link_libraries(vo PRIVATE ${OpenCV_LIBS})

add_executable(mahony src/Mahony.cpp)
target_include_directories(mahony PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(mahony PRIVATE Eigen3::Eigen)

add_executable(serial_test src/serial_read.cpp)
target_include_directories(rapidcsv INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(serialib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(serial_test PRIVATE serialib rapidcsv)

add_executable(dispersion src/disp.cpp)
target_include_directories(rapidcsv INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(dispersion PRIVATE rapidcsv)
