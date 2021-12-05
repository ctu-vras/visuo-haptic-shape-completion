catkin_add_gtest(mujoco mujoco.cpp)
target_link_libraries(mujoco ${catkin_LIBRARIES} ${mujoco_LIBRARIES} ${glfw3_LIBRARIES} ${libglew_LIBRARIES} ${OPENGL_gl_LIBRARY})
add_dependencies(mujoco ${catkin_EXPORTED_TARGETS})
