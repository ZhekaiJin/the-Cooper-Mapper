add_executable(fpdReceiver
            fpdReceiver.cpp utmProjection.cpp
            transPointCLoud.cpp)
#target_link_libraries(fpdReceiver ${catkin_LIBRARIES} ${PROJ4_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(fpdReceiver ${catkin_LIBRARIES} proj ${YAML_CPP_LIBRARIES})

add_executable(imuReceiver
            imuReceiver.cpp)
target_link_libraries(imuReceiver ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
