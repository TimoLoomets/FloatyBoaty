find_package(OpenCV REQUIRED)
MESSAGE(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS}")
# MESSAGE(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")
find_package(yaml-cpp REQUIRED)


file(GLOB_RECURSE LIB_VISUALIZER ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(visualizer STATIC ${LIB_VISUALIZER})
# add_executable(visualizer ${LIB_VISUALIZER})
target_link_libraries(visualizer ${OpenCV_LIBS} yaml-cpp)
target_include_directories (visualizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ManualTest ${CMAKE_CURRENT_SOURCE_DIR}/test/ManualTest.cpp)
target_link_libraries(ManualTest visualizer ${OpenCV_LIBS} yaml-cpp)