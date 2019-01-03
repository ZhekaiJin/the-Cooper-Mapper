add_library(scan_match
            ScanMatch.cpp)
target_link_libraries(scan_match ${catkin_LIBRARIES} ${PCL_LIBRARIES})
