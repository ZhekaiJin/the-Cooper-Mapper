#lib
add_library(loam
            ScanRegistration.cpp
            MultiScanRegistration.cpp
            OrganizedScanRegistration.cpp
            LaserOdometry.cpp
            LaserMatcher.cpp
            LaserMappingLocal.cpp
            LaserMapping.cpp
            LaserLocalization.cpp
            TransformMaintenance.cpp)
target_link_libraries(loam  scan_match ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(multi_scan_registration_node node/multi_scan_registration_node.cpp)
target_link_libraries(multi_scan_registration_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(organised_scan_registration_node node/organised_scan_registration_node.cpp)
target_link_libraries(organised_scan_registration_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(laser_odometry_node node/laser_odometry_node.cpp)
target_link_libraries(laser_odometry_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(laser_mappinglocal_node node/laser_mappinglocal_node.cpp)
target_link_libraries(laser_mappinglocal_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(laser_mapping_node node/laser_mapping_node.cpp)
target_link_libraries(laser_mapping_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(laser_localization_node node/laser_localization_node.cpp)
target_link_libraries(laser_localization_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

add_executable(transform_maintenance_node node/transform_maintenance_node.cpp)
target_link_libraries(transform_maintenance_node loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )

#nodelet lib
add_library(MultiScanRegistrationNodelet nodelet/MultiScanRegistrationNodelet.cpp)
target_link_libraries(MultiScanRegistrationNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(OrganisedScanRegistrationNodelet nodelet/OrganisedScanRegistrationNodelet.cpp)
target_link_libraries(OrganisedScanRegistrationNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(LaserOdometryNodelet nodelet/LaserOdometryNodelet.cpp)
target_link_libraries(LaserOdometryNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(LaserMappingNodelet nodelet/LaserMappingNodelet.cpp)
target_link_libraries(LaserMappingNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(LaserMappingLocalNodelet nodelet/LaserMappingLocalNodelet.cpp)
target_link_libraries(LaserMappingLocalNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(LaserLocalizationNodelet nodelet/LaserLocalizationNodelet.cpp)
target_link_libraries(LaserLocalizationNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(TransformMaintenanceNodelet nodelet/TransformMaintenanceNodelet.cpp)
target_link_libraries(TransformMaintenanceNodelet
  loam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
