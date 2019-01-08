add_library(evaluation Evaluation.cpp)

add_executable(evaluation_node node/evaluation_node.cpp)
target_link_libraries(evaluation_node evaluation ${catkin_LIBRARIES} )
