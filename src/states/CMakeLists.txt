add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(ViveRosState ViveRosState.cpp)
target_link_libraries(ViveRosState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_data_directory(data)
