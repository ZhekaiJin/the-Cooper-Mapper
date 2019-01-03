
add_library(graph
            graph.cpp
            solver_g2o.cpp
            keyframe.cpp)
target_link_libraries(graph 
                    scan_match
                    ${G2O_LIBRARIES}
                    ${G2O_TYPES_DATA}
                    ${G2O_CORE_LIBRARY}
                    ${G2O_STUFF_LIBRARY}
                    ${G2O_SOLVER_PCG}
                    ${G2O_SOLVER_CSPARSE}
                    ${G2O_TYPES_SLAM3D}
                    ${G2O_TYPES_SLAM3D_ADDONS}
                    ${catkin_LIBRARIES} 
                    ${PCL_LIBRARIES})
                    
add_executable(graph_node node/graph_node.cpp)
target_link_libraries(graph_node graph ${catkin_LIBRARIES} ${PCL_LIBRARIES})
