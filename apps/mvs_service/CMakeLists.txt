file (GLOB SOURCES "mvs_service.cpp")

#set(BIN mvs_service)

add_executable(mvs_service ${SOURCES})
set_property(TARGET mvs_service PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
add_dependencies(mvs_service ext_mve)
target_link_libraries(mvs_service tex
    ${TBB_LIBRARIES}
    -lmve -lmve_util
    ${Open3D_LIBRARIES}
    ${OpenCV_LIBRARIES}
     ${catkin_LIBRARIES}
    )
