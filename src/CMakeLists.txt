set(CONTROLLER_NAME HumanRetargetingController)

add_library(${CONTROLLER_NAME} SHARED
  HumanRetargetingController.cpp
  State.cpp
  ArmSide.cpp
  RosPoseManager.cpp
  ArmRetargetingManager.cpp
  RetargetingManagerSet.cpp
)

target_link_libraries(${CONTROLLER_NAME} PUBLIC 
  mc_rtc::mc_control_fsm
  mc_rtc::mc_rtc_ros
  baseline_walking_controller::BaselineWalkingController mc_rbdyn::mc_manus) # selim 2026-03-13 : added 'mc_rbdyn::mc_manus' to match hugo's last commit


install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR} EXPORT ${TARGETS_EXPORT_NAME})

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)
