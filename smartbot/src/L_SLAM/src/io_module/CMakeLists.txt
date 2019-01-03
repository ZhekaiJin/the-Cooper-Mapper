
add_executable(featureExtracter feature_extracter.cpp)
target_link_libraries(featureExtracter ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenMP_LIBS} )

add_executable(indexConvert indexConvert.cpp)
target_link_libraries(indexConvert ${catkin_LIBRARIES} ${PCL_LIBRARIES}  )