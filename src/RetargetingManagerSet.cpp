// 2026-03-13 - Copied Hugo's version based on his last commit (feature branch)
#include <mc_manus/ManusDevice.h>
#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Cylinder.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>
#include <mc_rtc/gui/Transform.h>
#include <mc_rtc/logging.h>
#include <mc_rtc_ros/ros.h>

#include <HumanRetargetingController/ArmRetargetingManager.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/RosPoseManager.h>

#include <sensor_msgs/msg/joy.hpp>

#include <iostream>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <vector>
#include <string>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);

  humanWaistPoseTopicName = static_cast<std::string>(mcRtcConfig("humanWaistPoseTopicName"));

  robotBaseLinkName = static_cast<std::string>(mcRtcConfig("robotBaseLinkName"));

  mcRtcConfig("mirrorRetargeting", mirrorRetargeting);

  mcRtcConfig("enableGripper", enableGripper);

  mcRtcConfig("syncJoints", syncJoints);

  mcRtcConfig("humanWaistPoseFromOrigin", humanWaistPoseFromOrigin);

  mcRtcConfig("pointMarkerSize", pointMarkerSize);
  mcRtcConfig("phaseMarkerPoseOffset", phaseMarkerPoseOffset);
}

RetargetingManagerSet::RetargetingManagerSet(HumanRetargetingController * ctlPtr,
                                             const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);

  for(auto [armSide, retargetingManagerConfig] :
      static_cast<std::map<std::string, mc_rtc::Configuration>>(mcRtcConfig("ArmRetargetingManagerList")))
  {
    retargetingManagerConfig.add("armSide", armSide);
    this->emplace(strToArmSide(armSide), std::make_shared<ArmRetargetingManager>(ctlPtr, retargetingManagerConfig));
  }
}

void RetargetingManagerSet::reset()
{
  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] ROS node handle is already instantiated.");
    nh_.reset();
  }

  rclcpp::NodeOptions node_options;
  nh_= rclcpp::Node::make_shared("RetargetingManagerSet", node_options);
  // Use a dedicated queue so as not to call callbacks of other modules
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);

  // Note: ROS node handle must be instantiated first
  humanWaistPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWaistPoseTopicName);

  isReady_ = false;
  isEnabled_ = false;

  calibRobots_.reset();

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->reset();
  }

  if(ctlPtr_->robot().hasDevice<mc_rbdyn::ManusDevice>("left_hand"))
  {
    manus_glove_left_ = &ctlPtr_->robot().device<mc_rbdyn::ManusDevice>("left_hand");
  }

  if(ctlPtr_->robot().hasDevice<mc_rbdyn::ManusDevice>("right_hand"))
  {
    manus_glove_right_ = &ctlPtr_->robot().device<mc_rbdyn::ManusDevice>("right_hand");
  }
}

void RetargetingManagerSet::update()
{
  executor_->spin_once(std::chrono::seconds(0));

  updateReadiness();

  updateEnablement();

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->updatePre();
  }

  if(config_.mirrorRetargeting)
  {
    makeRobotPosesMirrored();
  }

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->updatePost();
  }

  if(config_.enableGripper && isEnabled_)
  {
    updateGripper();
  }


  updateGUI();

  clearJoyMsg();
}

void RetargetingManagerSet::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->stop();
  }
}

//TODO: Example of how to use the map {Note Selim 13-03-2026 - TO CHECK}
void RetargetingManagerSet::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("ResetPosture", [this]() {
                            if(ctl().config().has("PostureTask"))
                            {
                              auto postureTask = ctl().getPostureTask(ctl().robot().name());
                              postureTask->load(ctl().solver(), ctl().config()("PostureTask"));
                            }
                            if(config_.enableGripper)
                            {
                              for(const auto & gripperName : std::set<std::string>{"l_gripper", "r_gripper"})
                              {
                                ctl().robot().gripper(gripperName).setTargetOpening(1.0);
                              }
                            }
                          }));

  gui.addElement({ctl().name(), config_.name, "Status"},
                 mc_rtc::gui::Label("isReady", [this]() { return isReady_ ? "Yes" : "No"; }),
                 // This implementation is for the Enalbe the retargeting without 
                 mc_rtc::gui::Label("isEnabled", [this](){ return isEnabled_ ? "Yes" : "No"; }));
                 
  if(!config_.mirrorRetargeting)
  {
    gui.addElement({ctl().name(), config_.name, "Calib"},
                   mc_rtc::gui::Button("clearRobot", [this]() { clearCalibRobot(); }));
  }

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->addToGUI(gui);
  }
}

void RetargetingManagerSet::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeElement({ctl().name()}, "ResetPosture");
  gui.removeCategory({ctl().name(), config_.name});

  // GUI of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::addToLogger(mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(config_.name + "_isReady", isReady_);
  MC_RTC_LOG_HELPER(config_.name + "_isEnabled", isEnabled_);

  logger.addLogEntry(config_.name + "_humanWaistValid", this, [this]() { return humanWaistPoseManager_->isValid(); });
  logger.addLogEntry(config_.name + "_humanWaistPose", this, [this]() {
    return humanWaistPoseManager_->isValid() ? humanWaistPoseManager_->pose() : sva::PTransformd::Identity();
  });

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->addToLogger(logger);
  }
}

void RetargetingManagerSet::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);

  // Log of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::enable()
{
  if(!isReady_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is not ready.");
    return;
  }

  if(isEnabled_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already enabled.");
    return;
  }

  isEnabled_ = true;
  mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting.");

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->enable();
  }
}

void RetargetingManagerSet::disable()
{
  if(!isEnabled_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already disabled.");
    return;
  }

  isEnabled_ = false;
  mc_rtc::log::success("[RetargetingManagerSet] Disable retargeting.");

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->disable();
  }

  // Update target position of joints
  {
    std::map<std::string, std::vector<double>> targetPosture;
    const auto & robotQ = ctl().robot().q();
    for(const auto & jointName : config_.syncJoints)
    {
      targetPosture.emplace(jointName, robotQ[ctl().robot().jointIndexByName(jointName)]);
    }

    auto postureTask = ctl().getPostureTask(ctl().robot().name());
    postureTask->target(targetPosture);
  }
}

void RetargetingManagerSet::updateReadiness()
{
  // Check readiness
  isReady_ = true;
  std::string invalidReasonStr = "";

  if(!humanWaistPoseManager_->isValid())
  {
    isReady_ = false;
    invalidReasonStr += "[Waist] " + humanWaistPoseManager_->invalidReasonStr_;
  }

  for(const auto & armManagerKV : *this)
  {
    if(!armManagerKV.second->calibResult_.isInitialized)
    {
      isReady_ = false;
      invalidReasonStr += "[" + std::to_string(armManagerKV.first) + "Arm] calib-uninitialized; ";
    }

    std::map<std::string, std::shared_ptr<RosPoseManager>> poseManagerMap = {
        {"Elbow", armManagerKV.second->humanElbowPoseManager_}, {"Wrist", armManagerKV.second->humanWristPoseManager_}};

    for(const auto & [poseName, poseManager] : poseManagerMap)
    {
      if(humanWaistPoseManager_->isValid())
      {
        constexpr double distThre = 2.0; // [m]
        poseManager->setDistValidityCheck(humanWaistPoseManager_->pose().translation(), distThre);
      }
      else
      {
        poseManager->clearDistValidityCheck();
      }

      if(!poseManager->isValid())
      {
        isReady_ = false;
        invalidReasonStr += "[" + std::to_string(armManagerKV.first) + poseName + "] " + poseManager->invalidReasonStr_;
      }
    }
  }

  // Disable retargeting if not ready
  if(!isReady_ && isEnabled_)
  {
    mc_rtc::log::error("Disable retargeting because it is not ready: {}", invalidReasonStr);
    disable();
  }
}

//Function to enable the retargeting only with the Valve index controllers
void RetargetingManagerSet::updateEnablement()
{
  bool enableFlag = false;
  bool disableFlag = false;

  //Check the VR contollers Buttons
  if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
  {
    const sensor_msgs::msg::Joy & leftHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::LeftHandJoyMsg");

    if(leftHandJoyMsg.buttons.size() > 0 && leftHandJoyMsg.buttons[0])
    {
      disableFlag = true;
    }
  }
  if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
  {
    const sensor_msgs::msg::Joy & rightHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::RightHandJoyMsg");

    if(rightHandJoyMsg.buttons.size() > 0 && rightHandJoyMsg.buttons[0])
    {
      printf("enableFlag\n");
      enableFlag = true;
    }
  }

  if(disableFlag && isEnabled_)
  {
    disable();
  }
  else if(enableFlag && !isEnabled_)
  {
    enable();
  }
}


// Convert ergonomics map to vector
std::vector<double> RetargetingManagerSet::ergonomicsMapToVector_(
    const std::unordered_map<std::string,
                             std::vector<mc_rbdyn::ManusDevice::Ergonomics>> & fingers,
    const std::vector<std::string> & ergo_type_order) const
{
  // New vector: ergonomics values in the order of ergo_type_order
  std::vector<double> result;
  result.reserve(ergo_type_order.size());

  for(const auto & type_name : ergo_type_order)
  {
    bool found = false;
    for(const auto & [fingerName, ergonomics] : fingers)
    {
      for(const auto & e : ergonomics)
      {
        if(e.type == type_name)
        {
          result.push_back(e.value);
          found = true;
          break;
        }
      }
      if(found) { break; }
    }

    if(!found)
    {
      result.push_back(0.0);
    }
  }

  return result;
}


void RetargetingManagerSet::updateGripper()
{

  mc_rtc::log::info("updateGripper called");

  mc_rtc::log::info("l_gripper exists: {}", ctl().robot().hasGripper("l_gripper"));
  mc_rtc::log::info("r_gripper exists: {}", ctl().robot().hasGripper("r_gripper"));
  mc_rtc::log::info("left glove null: {}", manus_glove_left_ == nullptr);
  mc_rtc::log::info("right glove null: {}", manus_glove_right_ == nullptr);

  // Process each Manus glove
  auto processGlove = [this](const std::string & gripperName,
                            mc_rbdyn::ManusDevice * glove)
  {
    mc_rtc::log::info("processGlove({})", gripperName);
    if(!glove) {
      mc_rtc::log::warning("Glove {} is NULL", gripperName);
      return;
    }
    if(!ctl().robot().hasGripper(gripperName)) { 
      mc_rtc::log::warning("Robot has NO gripper {}", gripperName);
      return;
    }

    // 1) Read Manus ergonomics
    const auto fingers = glove->getFingers();
    mc_rtc::log::info("{} fingers map size = {}", gripperName, fingers.size());

    // 2) Ordered list of ergonomics → Leap joints (NO PINKY)
  static const std::vector<std::string> LEAP_TO_LGRIPPER_ORDER = {
    // Index finger      (L_MCP_1 ... L_FINGERTIP_1)
    "IndexMCPStretch",   // MCP
    "IndexPIPStretch",   // PIP
    "IndexDIPStretch",   // DIP
    "IndexDIPStretch",   // TIP  (reuse DIP)

    // Middle finger
    "MiddleMCPStretch",  
    "MiddlePIPStretch",  
    "MiddleDIPStretch",  
    "MiddleDIPStretch",  

    // Ring finger 
    "RingMCPStretch",    
    "RingPIPStretch",    
    "RingDIPStretch",    
    "RingDIPStretch",    

    // Thumb  
    "ThumbMCPStretch",   
    "ThumbPIPStretch",   
    "ThumbDIPStretch",   
    "ThumbDIPStretch"    
  };

    // Convert Manus ergonomics → Leap joint vector
    std::vector<double> q =
        ergonomicsMapToVector_(fingers, LEAP_TO_LGRIPPER_ORDER);

    // convert to rad and clamp for safety
    for(double & v : q)
    {
      v = mc_rtc::constants::toRad(v);   // deg → rad
      v = std::clamp(v, -1.5, 1.5);      // safety clamp
    }

    // Send directly to Leap hand through the robot gripper
    ctl().robot().gripper(gripperName).setTargetQ(q);
  };

  processGlove("l_gripper", manus_glove_left_);
  processGlove("r_gripper", manus_glove_right_);

}


void RetargetingManagerSet::updateGUI()
{
  // Add buttons
  ctl().gui()->removeElement({ctl().name(), config_.name}, "EnableRetargeting");
  // Change this for some test
  ctl().gui()->removeElement({ctl().name(), config_.name}, "DisableRetargeting");
  if(isReady_ && !isEnabled_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("EnableRetargeting", [this]() { enable(); }));
  }
  if(isEnabled_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("DisableRetargeting", [this]() { disable(); }));
  }

  // Add pose markers
  {
    auto getPointColor = [](const std::string & _entityName) {
      return _entityName == "Human" ? mc_rtc::gui::Color(1.0, 0.0, 0.0, 0.5) : mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.5);
    };
    auto getArrowColor = [](const std::string & _entityName) {
      return _entityName == "Human" ? mc_rtc::gui::Color(0.4, 0.2, 0.2, 0.6) : mc_rtc::gui::Color(0.2, 0.4, 0.2, 0.6);
    };
    auto getPose = [](const std::shared_ptr<ArmRetargetingManager> & _armManager, const std::string & _entityName,
                      const std::string & _poseName) -> std::optional<sva::PTransformd> {
      if(_poseName == "Shoulder")
      {
        return _entityName == "Human" ? _armManager->humanShoulderPose_ : _armManager->robotShoulderPose_;
      }
      else if(_poseName == "Elbow")
      {
        return _entityName == "Human" ? _armManager->humanElbowPose_ : _armManager->robotElbowPose_;
      }
      else if(_poseName == "Wrist")
      {
        return _entityName == "Human" ? _armManager->humanWristPose_ : _armManager->robotWristPose_;
      }
      else
      {
        mc_rtc::log::error_and_throw("[RetargetingManagerSet] Invalid pose name: {}", _poseName);
      }
    };

    ctl().gui()->removeCategory({ctl().name(), config_.name, "Marker"});

    for(const auto & [armSide, armManager] : *this)
    {
      for(const auto & poseName : {"Shoulder", "Elbow", "Wrist"})
      {
        for(const auto & entityName : {"Human", "Robot"})
        {
          if(getPose(armManager, entityName, poseName).has_value())
          {
            ctl().gui()->addElement(
                {ctl().name(), config_.name, "Marker"},
                mc_rtc::gui::Point3D(entityName + std::to_string(armSide) + poseName + "Point",
                                     mc_rtc::gui::PointConfig(getPointColor(entityName), config_.pointMarkerSize),
                                     [=]() { return getPose(armManager, entityName, poseName).value().translation(); }),
                mc_rtc::gui::Transform(entityName + std::to_string(armSide) + poseName + "Transform",
                                       [=]() { return getPose(armManager, entityName, poseName).value(); }));
          }
        }
      }

      for(const auto & [poseName1, poseName2] :
          std::vector<std::pair<std::string, std::string>>{{"Shoulder", "Elbow"}, {"Elbow", "Wrist"}})
      {
        for(const auto & entityName : {"Human", "Robot"})
        {
          if(getPose(armManager, entityName, poseName1).has_value()
             && getPose(armManager, entityName, poseName2).has_value())
          {
            ctl().gui()->addElement(
                {ctl().name(), config_.name, "Marker"},
                mc_rtc::gui::Arrow(
                    entityName + std::to_string(armSide) + poseName1 + "To" + poseName2 + "Arrow",
                    getArrowColor(entityName),
                    [=]() { return getPose(armManager, entityName, poseName1).value().translation(); },
                    [=]() { return getPose(armManager, entityName, poseName2).value().translation(); }));
          }
        }
      }
    }
  }

  //TODO: CHECK THIS FOR THE ROTATION {Note Selim 13-03-2026 - TO CHECK}
  // Add phase marker
  {
    auto getPose = [this]() -> sva::PTransformd {
      return sva::PTransformd(config_.phaseMarkerPoseOffset.rotation(),
                              ctl().robot().posW().translation() + config_.phaseMarkerPoseOffset.translation());
    };
    auto getColor = [this]() -> mc_rtc::gui::Color {
      return isEnabled_ ? mc_rtc::gui::Color(1.0, 0.0, 1.0, 0.5) : mc_rtc::gui::Color(0.0, 1.0, 1.0, 0.5);
    };

    if(isReady_)
    {
      ctl().gui()->addElement(
          {ctl().name(), config_.name, "Marker"},
          mc_rtc::gui::Ellipsoid(
              "Phase", {0.2, 0.2, 0.15}, [=]() { return getPose(); }, [=]() { return getColor(); }));
    }
    else
    {
      ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                              mc_rtc::gui::Cylinder(
                                  "Phase", {0.1, 0.01}, [=]() { return getPose(); }, [=]() { return getColor(); }));
    }
  }

  // Visualize robot for calibration
  if(calibRobots_)
  {
    mc_rtc::ROSBridge::update_robot_publisher("calib", ctl().dt(), calibRobots_->robot());
  }
}

void RetargetingManagerSet::clearJoyMsg()
{
  if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
  {
    ctl().datastore().remove("HRC::ViveRos::LeftHandJoyMsg");
  }

  if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
  {
    ctl().datastore().remove("HRC::ViveRos::RightHandJoyMsg");
  }
}

void RetargetingManagerSet::makeRobotPosesMirrored()
{
  if(!(this->at(ArmSide::Left) && this->at(ArmSide::Right)))
  {
    return;
  }

  auto mirrorPose = [&](const sva::PTransformd & pose) -> sva::PTransformd {
    const Eigen::Vector3d & rpy = mc_rbdyn::rpyFromMat(pose.rotation());
    return sva::PTransformd(
        mc_rbdyn::rpyToMat(-1.0 * rpy.x(), rpy.y(), -1.0 * rpy.z()),
        Eigen::Vector3d(pose.translation().x(), -1.0 * pose.translation().y(), pose.translation().z()));
  };

  std::unordered_map<ArmSide, std::array<std::optional<sva::PTransformd>, 3>> mirroredRobotPosesMap;

  for(const auto & armSide : ArmSides::Both)
  {
    mirroredRobotPosesMap[armSide].fill(std::nullopt);

    mirroredRobotPosesMap.at(armSide)[0] = this->at(armSide)->robotShoulderPose_;

    if(this->at(armSide)->robotShoulderPose_.has_value() && this->at(opposite(armSide))->robotShoulderPose_.has_value())
    {
      if(this->at(opposite(armSide))->robotElbowPose_.has_value())
      {
        mirroredRobotPosesMap.at(armSide)[1] =
            mirrorPose(this->at(opposite(armSide))->robotElbowPose_.value()
                       * this->at(opposite(armSide))->robotShoulderPose_.value().inv())
            * this->at(armSide)->robotShoulderPose_.value();
      }

      if(this->at(opposite(armSide))->robotWristPose_.has_value())
      {
        mirroredRobotPosesMap.at(armSide)[2] =
            mirrorPose(this->at(opposite(armSide))->robotWristPose_.value()
                       * this->at(opposite(armSide))->robotShoulderPose_.value().inv())
            * this->at(armSide)->robotShoulderPose_.value();
      }
    }
  }

  for(const auto & armSide : ArmSides::Both)
  {
    this->at(armSide)->robotShoulderPose_ = mirroredRobotPosesMap.at(armSide)[0];
    this->at(armSide)->robotElbowPose_ = mirroredRobotPosesMap.at(armSide)[1];
    this->at(armSide)->robotWristPose_ = mirroredRobotPosesMap.at(armSide)[2];
  }
}

void RetargetingManagerSet::makeCalibRobot()
{
  if(calibRobots_)
  {
    return;
  }

  calibRobots_ = mc_rbdyn::Robots::make();
  ctl().robots().copy(*calibRobots_);
  auto frames = ctl().config()("frames", std::vector<mc_rbdyn::RobotModule::FrameDescription>{});
  calibRobots_->robot().makeFrames(frames);
}

void RetargetingManagerSet::clearCalibRobot()
{
  if(!calibRobots_)
  {
    return;
  }

  mc_rtc::ROSBridge::stop_robot_publisher("calib");
  calibRobots_.reset();
}

// Original Version - Commented 2026-03-13 by selim
// #include <mc_rtc/gui/Arrow.h>
// #include <mc_rtc/gui/Button.h>
// #include <mc_rtc/gui/Cylinder.h>
// #include <mc_rtc/gui/Ellipsoid.h>
// #include <mc_rtc/gui/Label.h>
// #include <mc_rtc/gui/Point3D.h>
// #include <mc_rtc/gui/Transform.h>
// #include <mc_rtc_ros/ros.h>

// #include <HumanRetargetingController/ArmRetargetingManager.h>
// #include <HumanRetargetingController/HumanRetargetingController.h>
// #include <HumanRetargetingController/RetargetingManagerSet.h>
// #include <HumanRetargetingController/RosPoseManager.h>

// #include <sensor_msgs/msg/joy.hpp>

// using namespace HRC;

// void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
// {
//   mcRtcConfig("name", name);

//   humanWaistPoseTopicName = static_cast<std::string>(mcRtcConfig("humanWaistPoseTopicName"));

//   robotBaseLinkName = static_cast<std::string>(mcRtcConfig("robotBaseLinkName"));

//   mcRtcConfig("mirrorRetargeting", mirrorRetargeting);

//   mcRtcConfig("enableGripper", enableGripper);

//   mcRtcConfig("syncJoints", syncJoints);

//   mcRtcConfig("humanWaistPoseFromOrigin", humanWaistPoseFromOrigin);

//   mcRtcConfig("pointMarkerSize", pointMarkerSize);
//   mcRtcConfig("phaseMarkerPoseOffset", phaseMarkerPoseOffset);
// }

// RetargetingManagerSet::RetargetingManagerSet(HumanRetargetingController * ctlPtr,
//                                              const mc_rtc::Configuration & mcRtcConfig)
// : ctlPtr_(ctlPtr)
// {
//   config_.load(mcRtcConfig);

//   for(auto [armSide, retargetingManagerConfig] :
//       static_cast<std::map<std::string, mc_rtc::Configuration>>(mcRtcConfig("ArmRetargetingManagerList")))
//   {
//     retargetingManagerConfig.add("armSide", armSide);
//     this->emplace(strToArmSide(armSide), std::make_shared<ArmRetargetingManager>(ctlPtr, retargetingManagerConfig));
//   }
// }

// void RetargetingManagerSet::reset()
// {
//   // Setup ROS
//   if(nh_)
//   {
//     mc_rtc::log::error("[RetargetingManagerSet] ROS node handle is already instantiated.");
//     nh_.reset();
//   }

//   rclcpp::NodeOptions node_options;
//   nh_= rclcpp::Node::make_shared("RetargetingManagerSet", node_options);
//   // Use a dedicated queue so as not to call callbacks of other modules
//   executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//   executor_->add_node(nh_);

//   // Note: ROS node handle must be instantiated first
//   humanWaistPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWaistPoseTopicName);

//   isReady_ = false;
//   isEnabled_ = false;

//   calibRobots_.reset();

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->reset();
//   }
// }

// void RetargetingManagerSet::update()
// {
//   executor_->spin_once(std::chrono::seconds(0));

//   updateReadiness();

//   updateEnablement();

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->updatePre();
//   }

//   if(config_.mirrorRetargeting)
//   {
//     makeRobotPosesMirrored();
//   }

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->updatePost();
//   }

//   if(config_.enableGripper && isEnabled_)
//   {
//     updateGripper();
//   }

//   updateGUI();

//   clearJoyMsg();
// }

// void RetargetingManagerSet::stop()
// {
//   removeFromGUI(*ctl().gui());
//   removeFromLogger(ctl().logger());

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->stop();
//   }
// }

// void RetargetingManagerSet::addToGUI(mc_rtc::gui::StateBuilder & gui)
// {
//   ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("ResetPosture", [this]() {
//                             if(ctl().config().has("PostureTask"))
//                             {
//                               auto postureTask = ctl().getPostureTask(ctl().robot().name());
//                               postureTask->load(ctl().solver(), ctl().config()("PostureTask"));
//                             }
//                             if(config_.enableGripper)
//                             {
//                               for(const auto & gripperName : std::set<std::string>{"l_gripper", "r_gripper"})
//                               {
//                                 ctl().robot().gripper(gripperName).setTargetOpening(1.0);
//                               }
//                             }
//                           }));

//   gui.addElement({ctl().name(), config_.name, "Status"},
//                  mc_rtc::gui::Label("isReady", [this]() { return isReady_ ? "Yes" : "No"; }),
//                  mc_rtc::gui::Label("isEnabled", [this]() { return isEnabled_ ? "Yes" : "No"; }));

//   if(!config_.mirrorRetargeting)
//   {
//     gui.addElement({ctl().name(), config_.name, "Calib"},
//                    mc_rtc::gui::Button("clearRobot", [this]() { clearCalibRobot(); }));
//   }

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->addToGUI(gui);
//   }
// }

// void RetargetingManagerSet::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
// {
//   gui.removeElement({ctl().name()}, "ResetPosture");
//   gui.removeCategory({ctl().name(), config_.name});

//   // GUI of each RetargetingManager is not removed here (removed via stop method)
// }

// void RetargetingManagerSet::addToLogger(mc_rtc::Logger & logger)
// {
//   MC_RTC_LOG_HELPER(config_.name + "_isReady", isReady_);
//   MC_RTC_LOG_HELPER(config_.name + "_isEnabled", isEnabled_);

//   logger.addLogEntry(config_.name + "_humanWaistValid", this, [this]() { return humanWaistPoseManager_->isValid(); });
//   logger.addLogEntry(config_.name + "_humanWaistPose", this, [this]() {
//     return humanWaistPoseManager_->isValid() ? humanWaistPoseManager_->pose() : sva::PTransformd::Identity();
//   });

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->addToLogger(logger);
//   }
// }

// void RetargetingManagerSet::removeFromLogger(mc_rtc::Logger & logger)
// {
//   logger.removeLogEntries(this);

//   // Log of each RetargetingManager is not removed here (removed via stop method)
// }

// void RetargetingManagerSet::enable()
// {
//   if(!isReady_)
//   {
//     mc_rtc::log::error("[RetargetingManagerSet] Retargeting is not ready.");
//     return;
//   }

//   if(isEnabled_)
//   {
//     mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already enabled.");
//     return;
//   }

//   isEnabled_ = true;
//   mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting.");

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->enable();
//   }
// }

// void RetargetingManagerSet::disable()
// {
//   if(!isEnabled_)
//   {
//     mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already disabled.");
//     return;
//   }

//   isEnabled_ = false;
//   mc_rtc::log::success("[RetargetingManagerSet] Disable retargeting.");

//   for(const auto & armManagerKV : *this)
//   {
//     armManagerKV.second->disable();
//   }

//   // Update target position of joints
//   {
//     std::map<std::string, std::vector<double>> targetPosture;
//     const auto & robotQ = ctl().robot().q();
//     for(const auto & jointName : config_.syncJoints)
//     {
//       targetPosture.emplace(jointName, robotQ[ctl().robot().jointIndexByName(jointName)]);
//     }

//     auto postureTask = ctl().getPostureTask(ctl().robot().name());
//     postureTask->target(targetPosture);
//   }
// }

// void RetargetingManagerSet::updateReadiness()
// {
//   // Check readiness
//   isReady_ = true;
//   std::string invalidReasonStr = "";

//   if(!humanWaistPoseManager_->isValid())
//   {
//     isReady_ = false;
//     invalidReasonStr += "[Waist] " + humanWaistPoseManager_->invalidReasonStr_;
//   }

//   for(const auto & armManagerKV : *this)
//   {
//     if(!armManagerKV.second->calibResult_.isInitialized)
//     {
//       isReady_ = false;
//       invalidReasonStr += "[" + std::to_string(armManagerKV.first) + "Arm] calib-uninitialized; ";
//     }

//     std::map<std::string, std::shared_ptr<RosPoseManager>> poseManagerMap = {
//         {"Elbow", armManagerKV.second->humanElbowPoseManager_}, {"Wrist", armManagerKV.second->humanWristPoseManager_}};

//     for(const auto & [poseName, poseManager] : poseManagerMap)
//     {
//       if(humanWaistPoseManager_->isValid())
//       {
//         constexpr double distThre = 2.0; // [m]
//         poseManager->setDistValidityCheck(humanWaistPoseManager_->pose().translation(), distThre);
//       }
//       else
//       {
//         poseManager->clearDistValidityCheck();
//       }

//       if(!poseManager->isValid())
//       {
//         isReady_ = false;
//         invalidReasonStr += "[" + std::to_string(armManagerKV.first) + poseName + "] " + poseManager->invalidReasonStr_;
//       }
//     }
//   }

//   // Disable retargeting if not ready
//   if(!isReady_ && isEnabled_)
//   {
//     mc_rtc::log::error("Disable retargeting because it is not ready: {}", invalidReasonStr);
//     disable();
//   }
// }

// void RetargetingManagerSet::updateEnablement()
// {
//   bool enableFlag = false;
//   bool disableFlag = false;

//   if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
//   {
//     const sensor_msgs::msg::Joy & leftHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::LeftHandJoyMsg");

//     if(leftHandJoyMsg.buttons.size() > 0 && leftHandJoyMsg.buttons[0])
//     {
//       disableFlag = true;
//     }
//   }
//   if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
//   {
//     const sensor_msgs::msg::Joy & rightHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::RightHandJoyMsg");

//     if(rightHandJoyMsg.buttons.size() > 0 && rightHandJoyMsg.buttons[0])
//     {
//       enableFlag = true;
//     }
//   }

//   if(disableFlag && isEnabled_)
//   {
//     disable();
//   }
//   else if(enableFlag && !isEnabled_)
//   {
//     enable();
//   }
// }

// void RetargetingManagerSet::updateGripper()
// {
//   if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
//   {
//     const sensor_msgs::msg::Joy & leftHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::LeftHandJoyMsg");

//     std::string gripperName = "l_gripper";
//     if(config_.mirrorRetargeting)
//     {
//       gripperName = "r_gripper";
//     }
//     if(leftHandJoyMsg.axes.size() > 2)
//     {
//       ctl().robot().gripper(gripperName).setTargetOpening(1.0 - leftHandJoyMsg.axes[2]);
//     }
//   }
//   if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
//   {
//     const sensor_msgs::msg::Joy & rightHandJoyMsg = ctl().datastore().get<sensor_msgs::msg::Joy>("HRC::ViveRos::RightHandJoyMsg");

//     std::string gripperName = "r_gripper";
//     if(config_.mirrorRetargeting)
//     {
//       gripperName = "l_gripper";
//     }
//     if(rightHandJoyMsg.axes.size() > 2)
//     {
//       ctl().robot().gripper(gripperName).setTargetOpening(1.0 - rightHandJoyMsg.axes[2]);
//     }
//   }
// }

// void RetargetingManagerSet::updateGUI()
// {
//   // Add buttons
//   ctl().gui()->removeElement({ctl().name(), config_.name}, "EnableRetargeting");
//   ctl().gui()->removeElement({ctl().name(), config_.name}, "DisableRetargeting");
//   if(isReady_ && !isEnabled_)
//   {
//     ctl().gui()->addElement({ctl().name(), config_.name},
//                             mc_rtc::gui::Button("EnableRetargeting", [this]() { enable(); }));
//   }
//   if(isEnabled_)
//   {
//     ctl().gui()->addElement({ctl().name(), config_.name},
//                             mc_rtc::gui::Button("DisableRetargeting", [this]() { disable(); }));
//   }

//   // Add pose markers
//   {
//     auto getPointColor = [](const std::string & _entityName) {
//       return _entityName == "Human" ? mc_rtc::gui::Color(1.0, 0.0, 0.0, 0.5) : mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.5);
//     };
//     auto getArrowColor = [](const std::string & _entityName) {
//       return _entityName == "Human" ? mc_rtc::gui::Color(0.4, 0.2, 0.2, 0.6) : mc_rtc::gui::Color(0.2, 0.4, 0.2, 0.6);
//     };
//     auto getPose = [](const std::shared_ptr<ArmRetargetingManager> & _armManager, const std::string & _entityName,
//                       const std::string & _poseName) -> std::optional<sva::PTransformd> {
//       if(_poseName == "Shoulder")
//       {
//         return _entityName == "Human" ? _armManager->humanShoulderPose_ : _armManager->robotShoulderPose_;
//       }
//       else if(_poseName == "Elbow")
//       {
//         return _entityName == "Human" ? _armManager->humanElbowPose_ : _armManager->robotElbowPose_;
//       }
//       else if(_poseName == "Wrist")
//       {
//         return _entityName == "Human" ? _armManager->humanWristPose_ : _armManager->robotWristPose_;
//       }
//       else
//       {
//         mc_rtc::log::error_and_throw("[RetargetingManagerSet] Invalid pose name: {}", _poseName);
//       }
//     };

//     ctl().gui()->removeCategory({ctl().name(), config_.name, "Marker"});

//     for(const auto & [armSide, armManager] : *this)
//     {
//       for(const auto & poseName : {"Shoulder", "Elbow", "Wrist"})
//       {
//         for(const auto & entityName : {"Human", "Robot"})
//         {
//           if(getPose(armManager, entityName, poseName).has_value())
//           {
//             ctl().gui()->addElement(
//                 {ctl().name(), config_.name, "Marker"},
//                 mc_rtc::gui::Point3D(entityName + std::to_string(armSide) + poseName + "Point",
//                                      mc_rtc::gui::PointConfig(getPointColor(entityName), config_.pointMarkerSize),
//                                      [=]() { return getPose(armManager, entityName, poseName).value().translation(); }),
//                 mc_rtc::gui::Transform(entityName + std::to_string(armSide) + poseName + "Transform",
//                                        [=]() { return getPose(armManager, entityName, poseName).value(); }));
//           }
//         }
//       }

//       for(const auto & [poseName1, poseName2] :
//           std::vector<std::pair<std::string, std::string>>{{"Shoulder", "Elbow"}, {"Elbow", "Wrist"}})
//       {
//         for(const auto & entityName : {"Human", "Robot"})
//         {
//           if(getPose(armManager, entityName, poseName1).has_value()
//              && getPose(armManager, entityName, poseName2).has_value())
//           {
//             ctl().gui()->addElement(
//                 {ctl().name(), config_.name, "Marker"},
//                 mc_rtc::gui::Arrow(
//                     entityName + std::to_string(armSide) + poseName1 + "To" + poseName2 + "Arrow",
//                     getArrowColor(entityName),
//                     [=]() { return getPose(armManager, entityName, poseName1).value().translation(); },
//                     [=]() { return getPose(armManager, entityName, poseName2).value().translation(); }));
//           }
//         }
//       }
//     }
//   }

//   // Add phase marker
//   {
//     auto getPose = [this]() -> sva::PTransformd {
//       return sva::PTransformd(config_.phaseMarkerPoseOffset.rotation(),
//                               ctl().robot().posW().translation() + config_.phaseMarkerPoseOffset.translation());
//     };
//     auto getColor = [this]() -> mc_rtc::gui::Color {
//       return isEnabled_ ? mc_rtc::gui::Color(1.0, 0.0, 1.0, 0.5) : mc_rtc::gui::Color(0.0, 1.0, 1.0, 0.5);
//     };

//     if(isReady_)
//     {
//       ctl().gui()->addElement(
//           {ctl().name(), config_.name, "Marker"},
//           mc_rtc::gui::Ellipsoid(
//               "Phase", {0.2, 0.2, 0.15}, [=]() { return getPose(); }, [=]() { return getColor(); }));
//     }
//     else
//     {
//       ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
//                               mc_rtc::gui::Cylinder(
//                                   "Phase", {0.1, 0.01}, [=]() { return getPose(); }, [=]() { return getColor(); }));
//     }
//   }

//   // Visualize robot for calibration
//   if(calibRobots_)
//   {
//     mc_rtc::ROSBridge::update_robot_publisher("calib", ctl().dt(), calibRobots_->robot());
//   }
// }

// void RetargetingManagerSet::clearJoyMsg()
// {
//   if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
//   {
//     ctl().datastore().remove("HRC::ViveRos::LeftHandJoyMsg");
//   }

//   if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
//   {
//     ctl().datastore().remove("HRC::ViveRos::RightHandJoyMsg");
//   }
// }

// void RetargetingManagerSet::makeRobotPosesMirrored()
// {
//   if(!(this->at(ArmSide::Left) && this->at(ArmSide::Right)))
//   {
//     return;
//   }

//   auto mirrorPose = [&](const sva::PTransformd & pose) -> sva::PTransformd {
//     const Eigen::Vector3d & rpy = mc_rbdyn::rpyFromMat(pose.rotation());
//     return sva::PTransformd(
//         mc_rbdyn::rpyToMat(-1.0 * rpy.x(), rpy.y(), -1.0 * rpy.z()),
//         Eigen::Vector3d(pose.translation().x(), -1.0 * pose.translation().y(), pose.translation().z()));
//   };

//   std::unordered_map<ArmSide, std::array<std::optional<sva::PTransformd>, 3>> mirroredRobotPosesMap;

//   for(const auto & armSide : ArmSides::Both)
//   {
//     mirroredRobotPosesMap[armSide].fill(std::nullopt);

//     mirroredRobotPosesMap.at(armSide)[0] = this->at(armSide)->robotShoulderPose_;

//     if(this->at(armSide)->robotShoulderPose_.has_value() && this->at(opposite(armSide))->robotShoulderPose_.has_value())
//     {
//       if(this->at(opposite(armSide))->robotElbowPose_.has_value())
//       {
//         mirroredRobotPosesMap.at(armSide)[1] =
//             mirrorPose(this->at(opposite(armSide))->robotElbowPose_.value()
//                        * this->at(opposite(armSide))->robotShoulderPose_.value().inv())
//             * this->at(armSide)->robotShoulderPose_.value();
//       }

//       if(this->at(opposite(armSide))->robotWristPose_.has_value())
//       {
//         mirroredRobotPosesMap.at(armSide)[2] =
//             mirrorPose(this->at(opposite(armSide))->robotWristPose_.value()
//                        * this->at(opposite(armSide))->robotShoulderPose_.value().inv())
//             * this->at(armSide)->robotShoulderPose_.value();
//       }
//     }
//   }

//   for(const auto & armSide : ArmSides::Both)
//   {
//     this->at(armSide)->robotShoulderPose_ = mirroredRobotPosesMap.at(armSide)[0];
//     this->at(armSide)->robotElbowPose_ = mirroredRobotPosesMap.at(armSide)[1];
//     this->at(armSide)->robotWristPose_ = mirroredRobotPosesMap.at(armSide)[2];
//   }
// }

// void RetargetingManagerSet::makeCalibRobot()
// {
//   if(calibRobots_)
//   {
//     return;
//   }

//   calibRobots_ = mc_rbdyn::Robots::make();
//   ctl().robots().copy(*calibRobots_);
//   auto frames = ctl().config()("frames", std::vector<mc_rbdyn::RobotModule::FrameDescription>{});
//   calibRobots_->robot().makeFrames(frames);
// }

// void RetargetingManagerSet::clearCalibRobot()
// {
//   if(!calibRobots_)
//   {
//     return;
//   }

//   mc_rtc::ROSBridge::stop_robot_publisher("calib");
//   calibRobots_.reset();
// }
