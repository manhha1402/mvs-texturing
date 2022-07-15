file (GLOB SOURCES "test_function.cpp")

#set(BIN test_function)

add_executable(test_function ${SOURCES})
set_property(TARGET test_function PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
add_dependencies(test_function ext_mve)
target_link_libraries(test_function tex
    ${TBB_LIBRARIES}
    -lmve -lmve_util
    ${Open3D_LIBRARIES}
    ${OpenCV_LIBRARIES}
     ${catkin_LIBRARIES}
    )
