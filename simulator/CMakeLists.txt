project(Simulator)

file(GLOB_RECURSE MAP_EDITOR ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${MAP_EDITOR})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Visualizer)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)