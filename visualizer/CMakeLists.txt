project(Visualizer)

find_package(OpenCV REQUIRED)
MESSAGE(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS}")
# MESSAGE(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")
find_package(yaml-cpp REQUIRED)
set(VISUALIZER_DEPS ${OpenCV_LIBS} yaml-cpp)


file(GLOB_RECURSE LIB_VISUALIZER ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} STATIC ${LIB_VISUALIZER})
target_link_libraries(${PROJECT_NAME} ${VISUALIZER_DEPS})
target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ManualTest ${CMAKE_CURRENT_SOURCE_DIR}/test/ManualTest.cpp)
target_link_libraries(ManualTest ${PROJECT_NAME} ${VISUALIZER_DEPS})