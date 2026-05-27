#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/TransformTask.h>

#include <HumanRetargetingController/ArmRetargetingManager.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/RosPoseManager.h>

// #include <fstream>
// #include <chrono>
// #include <iomanip>
// #include <sstream>
// #include <filesystem>

// std::ofstream csvFile_;
// bool csvInitialized_ = false;

using namespace HRC;

void ArmRetargetingManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  armSide = strToArmSide(mcRtcConfig("armSide"));

  humanElbowPoseTopicName = static_cast<std::string>(mcRtcConfig("humanElbowPoseTopicName"));
  humanWristPoseTopicName = static_cast<std::string>(mcRtcConfig("humanWristPoseTopicName"));

  elbowTaskName = static_cast<std::string>(mcRtcConfig("elbowTaskName"));
  wristTaskName = static_cast<std::string>(mcRtcConfig("wristTaskName"));

  if(mcRtcConfig.has("stiffness"))
  {
    if(mcRtcConfig("stiffness").isNumeric())
    {
      stiffness = Eigen::Vector6d::Constant(mcRtcConfig("stiffness"));
    }
    else
    {
      stiffness = mcRtcConfig("stiffness");
    }
  }

  mcRtcConfig("robotCalibPostures", robotCalibPostures);

  mcRtcConfig("calibResultConfig", calibResultConfig);
}

ArmRetargetingManager::CalibResult::CalibResult()
{
  reset();
}

void ArmRetargetingManager::CalibResult::reset()
{
  isInitialized = false;

  humanTransFromBaseToShoulder = sva::PTransformd::Identity();
  robotTransFromBaseToShoulder = sva::PTransformd::Identity();

  elbowRotTransFromHumanToRobot = sva::PTransformd::Identity();
  wristRotTransFromHumanToRobot = sva::PTransformd::Identity();

  elbowScale = 1.0;
  wristScale = 1.0;
}

void ArmRetargetingManager::CalibResult::load(const mc_rtc::Configuration & mcRtcConfig)
{
  isInitialized = true;

  humanTransFromBaseToShoulder = mcRtcConfig("humanTransFromBaseToShoulder");
  robotTransFromBaseToShoulder = mcRtcConfig("robotTransFromBaseToShoulder");

  elbowRotTransFromHumanToRobot = mcRtcConfig("elbowRotTransFromHumanToRobot");
  wristRotTransFromHumanToRobot = mcRtcConfig("wristRotTransFromHumanToRobot");

  elbowScale = mcRtcConfig("elbowScale");
  wristScale = mcRtcConfig("wristScale");
}

std::string ArmRetargetingManager::CalibResult::dump() const
{
  mc_rtc::Configuration mcRtcConfig;

  auto calibConfig = mcRtcConfig.add("calibResultConfig");

  calibConfig.add("humanTransFromBaseToShoulder", humanTransFromBaseToShoulder);
  calibConfig.add("robotTransFromBaseToShoulder", robotTransFromBaseToShoulder);

  calibConfig.add("elbowRotTransFromHumanToRobot", elbowRotTransFromHumanToRobot);
  calibConfig.add("wristRotTransFromHumanToRobot", wristRotTransFromHumanToRobot);

  calibConfig.add("elbowScale", elbowScale);
  calibConfig.add("wristScale", wristScale);

  return mcRtcConfig.dump(true, true);
}

ArmRetargetingManager::ArmRetargetingManager(HumanRetargetingController * ctlPtr,
                                             const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);
}

// original reset function
void ArmRetargetingManager::reset()
{
  humanElbowPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanElbowPoseTopicName);
  humanWristPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWristPoseTopicName);

  humanShoulderPose_ = std::nullopt;
  humanElbowPose_ = std::nullopt;
  humanWristPose_ = std::nullopt;

  robotShoulderPose_ = std::nullopt;
  robotElbowPose_ = std::nullopt;
  robotWristPose_ = std::nullopt;

  stiffnessRatioFunc_ = nullptr;

  humanCalibSource_.clear();
  robotCalibSource_.clear();

  if(!config_.calibResultConfig.empty())
  {
    calibResult_.load(config_.calibResultConfig);
  }
}

// test reset function 31-03-2026 for csv handling
// void ArmRetargetingManager::reset()
// {
//   humanElbowPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanElbowPoseTopicName);
//   humanWristPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWristPoseTopicName);

//   humanShoulderPose_ = std::nullopt;
//   humanElbowPose_ = std::nullopt;
//   humanWristPose_ = std::nullopt;

//   robotShoulderPose_ = std::nullopt;
//   robotElbowPose_ = std::nullopt;
//   robotWristPose_ = std::nullopt;

//   stiffnessRatioFunc_ = nullptr;

//   humanCalibSource_.clear();
//   robotCalibSource_.clear();

//   if(!config_.calibResultConfig.empty())
//   {
//     calibResult_.load(config_.calibResultConfig);
//   }

//   // ===== CSV INIT =====
//   if(csvFile_.is_open())
//   {
//     csvFile_.close();
//   }

//   // get current time (thread-safe)
//   auto now = std::chrono::system_clock::now();
//   std::time_t t = std::chrono::system_clock::to_time_t(now);

//   std::tm tm;
//   localtime_r(&t, &tm);

//   // build folder path: /tmp/arm_logs_YYYYMMDD/
//   std::ostringstream folderPath;
//   folderPath << "/tmp/arm_logs_" << std::put_time(&tm, "%Y%m%d");
//   // folderPath << "/$HOME/Documents/Retargeting/arm_logs_" << std::put_time(&tm, "%Y%m%d");


//   // create directory safely
//   std::error_code ec;
//   std::filesystem::create_directories(folderPath.str(), ec);
//   if(ec)
//   {
//     mc_rtc::log::error("[ArmRetargetingManager] Failed to create directory: {}", ec.message());
//   }

//   // build file path: /log_HHMMSS.csv
//   std::ostringstream filePath;
//   filePath << folderPath.str()
//          << "/log_" << std::put_time(&tm, "%H%M%S")
//          << "_" << (config_.armSide == ArmSide::Left ? "L" : "R")
//          << ".csv";

//   // open file
//   csvFile_.open(filePath.str(), std::ios::out);

//   if(csvFile_.is_open())
//   {
//     csvInitialized_ = true;

//     mc_rtc::log::success("[ArmRetargetingManager] CSV logging to: {}", filePath.str());

//     csvFile_ << "time,"
//             << "human_elbow_x,human_elbow_y,human_elbow_z,"
//             << "robot_elbow_x,robot_elbow_y,robot_elbow_z,"
//             << "human_wrist_x,human_wrist_y,human_wrist_z,"
//             << "robot_wrist_x,robot_wrist_y,robot_wrist_z,"
//             << "human_wrist_qx,human_wrist_qy,human_wrist_qz,human_wrist_qw,"
//             << "robot_wrist_qx,robot_wrist_qy,robot_wrist_qz,robot_wrist_qw\n";
//   }
//   else
//   {
//     csvInitialized_ = false;
//     mc_rtc::log::error("[ArmRetargetingManager] Failed to open CSV file");
//   }
// }

// TODO: check the line hugo mentionned
void ArmRetargetingManager::updatePre()
{
  // Calculate human and robot poses
  if(calibResult_.isInitialized)
  {
    humanShoulderPose_ = std::nullopt;
    humanElbowPose_ = std::nullopt;
    humanWristPose_ = std::nullopt;

    robotShoulderPose_ = calibResult_.robotTransFromBaseToShoulder
                         * ctl().robot().frame(ctl().retargetingManagerSet_->config().robotBaseLinkName).position();

    robotElbowPose_ = std::nullopt;
    robotWristPose_ = std::nullopt;

    // mc_rtc::log::info("Waist pose:\n{}", humanWaistPoseManager()->pose());
    // mc_rtc::log::info("Elbow raw pose:\n{}", humanElbowPoseManager_->pose());
    // mc_rtc::log::info("Wrist raw pose:\n{}", humanWristPoseManager_->pose()); // if quaternion does not change VR issue, if it changes correctly no VR issue
    // mc_rtc::log::info("==== RAW INPUT DEBUG ====");
    // mc_rtc::log::info("Waist valid: {}", humanWaistPoseManager()->isValid());
    // mc_rtc::log::info("Elbow valid: {}", humanElbowPoseManager_->isValid());
    // mc_rtc::log::info("Wrist valid: {}", humanWristPoseManager_->isValid());
   

    if(humanWaistPoseManager()->isValid())
    {
      auto scalePose = [&](const sva::PTransformd & pose, double scale) -> sva::PTransformd {
        return sva::PTransformd(pose.rotation(), scale * pose.translation());
      };

      // TODO: Check this line - hugo's comment - 2026-03-13
      const auto & humanWaistPoseFromOrigin = ctl().retargetingManagerSet_->config_.humanWaistPoseFromOrigin;
      // mc_rtc::log::info("Waist from origin:\n{}", humanWaistPoseFromOrigin);

      // original function
      humanShoulderPose_ = calibResult_.humanTransFromBaseToShoulder * humanWaistPoseFromOrigin;

      // test 19-03-2026
      // humanShoulderPose_ =
      //   humanWaistPoseManager()->pose() * // go tu current waist (live from frame)
      //   calibResult_.humanTransFromBaseToShoulder * // apply shoulder offset relative to waist
      //   humanWaistPoseManager()->pose().inv() * // go back to waist-local frame
      //   humanWaistPoseFromOrigin; // express everything in calibrated frame (same as elbow/wrist)

      // // test 31-03-2026
      // if(humanShoulderPose_)
      // {
      //   mc_rtc::log::info("Human shoulder pose:\n{}", humanShoulderPose_.value());
      // }

      // mc_rtc::log::info("==== FRAME DEBUG ====");
      // mc_rtc::log::info("humanWaistPose:\n{}", humanWaistPoseManager()->pose());
      // mc_rtc::log::info("humanWaistPoseFromOrigin:\n{}", humanWaistPoseFromOrigin);

      // // MOST IMPORTANT CHECK
      // if(humanShoulderPose_)
      // {
      //   auto test = humanWaistPoseManager()->pose().inv() * humanShoulderPose_.value();
      //   mc_rtc::log::info("Shoulder expressed in waist frame (MUST be constant):\n{}", test);
      // }
      ////////////////////////////

      if(humanElbowPoseManager_->isValid())
      {
        humanElbowPose_ =
            humanElbowPoseManager_->pose() * humanWaistPoseManager()->pose().inv() * humanWaistPoseFromOrigin;

        robotElbowPose_ =
            calibResult_.elbowRotTransFromHumanToRobot
            * scalePose(humanElbowPose_.value() * humanShoulderPose_.value().inv(), calibResult_.elbowScale)
            * robotShoulderPose_.value();
      }

      if(humanWristPoseManager_->isValid())
      {
        humanWristPose_ =
            humanWristPoseManager_->pose() * humanWaistPoseManager()->pose().inv() * humanWaistPoseFromOrigin;
        
        mc_rtc::log::info("Waist pose:\n{}", humanWaistPoseManager()->pose());
        mc_rtc::log::info("humanWristPose_:\n{}", humanWristPose_.value());

        // wristRotTransFromHumanToRObot applied before scaling and before transforming to robot shoulder frame ?
        
        robotWristPose_ =
            calibResult_.wristRotTransFromHumanToRobot
            * scalePose(humanWristPose_.value() * humanShoulderPose_.value().inv(), calibResult_.wristScale)
            * robotShoulderPose_.value();

        const auto & A = calibResult_.wristRotTransFromHumanToRobot;

        const auto B =
          scalePose(
            humanWristPose_.value() * humanShoulderPose_.value().inv(),
            calibResult_.wristScale
          );

        const auto & C = robotShoulderPose_.value();

        mc_rtc::log::info("A (wristRotTransFromHumanToRobot) rotation:\n{}", A.rotation());
        mc_rtc::log::info("B (scaled human wrist) rotation:\n{}", B.rotation());
        mc_rtc::log::info("C (robot shoulder) rotation:\n{}", C.rotation());
        mc_rtc::log::info("Final robotWristPose_ rotation:\n{}", robotWristPose_.value().rotation());
      }


      // test 31-03-2026
      // mc_rtc::log::info("==== LIMB DEBUG ====");
      // if(humanElbowPose_ && humanShoulderPose_)
      // {
      //   auto relHuman = humanElbowPose_.value() * humanShoulderPose_.value().inv();
      //   mc_rtc::log::info("Human elbow relative to shoulder:\n{}", relHuman);
      // }

      // if(robotElbowPose_ && robotShoulderPose_)
      // {
      //   auto relRobot = robotElbowPose_.value() * robotShoulderPose_.value().inv();
      //   mc_rtc::log::info("Robot elbow relative to shoulder:\n{}", relRobot);
      // }


    }
  }
}

void ArmRetargetingManager::updatePost()
{
  // Set task target
  if(ctl().retargetingManagerSet_->isEnabled_)
  {
    // mc_rtc::log::info("==== FINAL TARGET DEBUG ====");
    setTaskTarget(elbowTask(), robotElbowPose_.value());
    // mc_rtc::log::info("FINAL robot elbow sent to task:\n{}", robotElbowPose_.value());
    setTaskTarget(wristTask(), robotWristPose_.value()); // og line 
    // mc_rtc::log::info("FINAL robot wrist sent to task:\n{}", robotWristPose_.value());

    // test 19-03-2026 force wrist rotation
    // if(robotWristPose_)
    // {
    //   auto pose = robotWristPose_.value();

    //   double t = ctl().t();

    //   // rotate ONLY around wrist Y (axis [0 0 1])
    //   pose.rotation() =
    //     Eigen::AngleAxisd(0.8 * std::sin(t), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //   setTaskTarget(wristTask(), pose);
    // }
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() < stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      for(const auto & task : {elbowTask(), wristTask()})
      {
        task->stiffness(stiffnessRatio * config_.stiffness);
      }
    }
    else
    {
      for(const auto & task : {elbowTask(), wristTask()})
      {
        task->stiffness(Eigen::VectorXd(config_.stiffness));
      }
      stiffnessRatioFunc_.reset();
    }
  }


  // add csv 01-04-2026
  // if(csvInitialized_ && humanElbowPose_ && robotElbowPose_
  //                   && humanWristPose_ && robotWristPose_)
  // {
  //   const auto & he = humanElbowPose_.value().translation();
  //   const auto & re = robotElbowPose_.value().translation();
  //   const auto & hw = humanWristPose_.value().translation();
  //   const auto & rw = robotWristPose_.value().translation();

  //   auto qh = Eigen::Quaterniond(humanWristPose_.value().rotation());
  //   auto qr = Eigen::Quaterniond(robotWristPose_.value().rotation());

  //   csvFile_ << ctl().t() << ","
  //           << he.x() << "," << he.y() << "," << he.z() << ","
  //           << re.x() << "," << re.y() << "," << re.z() << ","
  //           << hw.x() << "," << hw.y() << "," << hw.z() << ","
  //           << rw.x() << "," << rw.y() << "," << rw.z() << ","

  //           << qh.x() << "," << qh.y() << "," << qh.z() << "," << qh.w() << ","
  //           << qr.x() << "," << qr.y() << "," << qr.z() << "," << qr.w()

  //           << std::endl;
  // }
}

void ArmRetargetingManager::stop()
{

  // if(csvFile_.is_open())
  // {
  //   csvFile_.close();
  // }

  for(const auto & task : {elbowTask(), wristTask()})
  {
    ctl().solver().removeTask(task);
  }

  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void ArmRetargetingManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  if(!ctl().retargetingManagerSet_->config().mirrorRetargeting)
  {
    const std::vector<std::string> & calibCategory = {ctl().name(), ctl().retargetingManagerSet_->config_.name,
                                                      std::to_string(config_.armSide), "Calib"};

    gui.addElement(calibCategory,
                   mc_rtc::gui::Label("isInitialized", [this]() { return calibResult_.isInitialized ? "Yes" : "No"; }),
                   mc_rtc::gui::Button("reset", [this]() { calibResult_.reset(); }),
                   mc_rtc::gui::Button("update", [this]() { updateCalib(); }));
    gui.addElement(calibCategory, mc_rtc::gui::ElementsStacking::Horizontal,
                   mc_rtc::gui::Button("setHuman-X", [this]() { setHumanCalibSource("X"); }),
                   mc_rtc::gui::Button("setHuman-Y", [this]() { setHumanCalibSource("Y"); }),
                   mc_rtc::gui::Button("setHuman-Z", [this]() { setHumanCalibSource("Z"); }));
    gui.addElement(calibCategory, mc_rtc::gui::ElementsStacking::Horizontal,
                   mc_rtc::gui::Button("setRobot-X", [this]() { setRobotCalibSource("X"); }),
                   mc_rtc::gui::Button("setRobot-Y", [this]() { setRobotCalibSource("Y"); }),
                   mc_rtc::gui::Button("setRobot-Z", [this]() { setRobotCalibSource("Z"); }));
  }
}

void ArmRetargetingManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), ctl().retargetingManagerSet_->config_.name, std::to_string(config_.armSide)});
}

void ArmRetargetingManager::addToLogger(mc_rtc::Logger & logger)
{
  const auto & name = ctl().retargetingManagerSet_->config_.name + "_" + std::to_string(config_.armSide);

  logger.addLogEntry(name + "_humanElbowValid", this, [this]() { return humanElbowPoseManager_->isValid(); });
  logger.addLogEntry(name + "_humanElbowPose", this, [this]() {
    return humanElbowPoseManager_->isValid() ? humanElbowPoseManager_->pose() : sva::PTransformd::Identity();
  });
  logger.addLogEntry(name + "_humanWristValid", this, [this]() { return humanWristPoseManager_->isValid(); });
  logger.addLogEntry(name + "_humanWristPose", this, [this]() {
    return humanWristPoseManager_->isValid() ? humanWristPoseManager_->pose() : sva::PTransformd::Identity();
  });

  // // ajout du  31-03-2026
  // // ===== CUSTOM DEBUG DATA =====
  // // Human shoulder
  // logger.addLogEntry(name + "_humanShoulderPose", this, [this]() {
  //   return humanShoulderPose_.has_value() ? humanShoulderPose_.value() : sva::PTransformd::Identity();
  // });

  // // Robot shoulder
  // logger.addLogEntry(name + "_robotShoulderPose", this, [this]() {
  //   return robotShoulderPose_.has_value() ? robotShoulderPose_.value() : sva::PTransformd::Identity();
  // });

  // // Robot elbow target
  // logger.addLogEntry(name + "_robotElbowPose", this, [this]() {
  //   return robotElbowPose_.has_value() ? robotElbowPose_.value() : sva::PTransformd::Identity();
  // });

  // // Robot wrist target
  // logger.addLogEntry(name + "_robotWristPose", this, [this]() {
  //   return robotWristPose_.has_value() ? robotWristPose_.value() : sva::PTransformd::Identity();
  // });

  // // Relative elbow (human)
  // logger.addLogEntry(name + "_humanElbowRel", this, [this]() {
  //   if(humanElbowPose_ && humanShoulderPose_)
  //   {
  //     return humanElbowPose_.value() * humanShoulderPose_.value().inv();
  //   }
  //   return sva::PTransformd::Identity();
  // });

  // // Relative elbow (robot)
  // logger.addLogEntry(name + "_robotElbowRel", this, [this]() {
  //   if(robotElbowPose_ && robotShoulderPose_)
  //   {
  //     return robotElbowPose_.value() * robotShoulderPose_.value().inv();
  //   }
  //   return sva::PTransformd::Identity();
  // });

}

void ArmRetargetingManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void ArmRetargetingManager::enable()
{
  for(const auto & task : {elbowTask(), wristTask()})
  {
    task->reset();
    ctl().solver().addTask(task);
    task->stiffness(0.0);
  }

  constexpr double stiffnessInterpDuration = 2.0; // [sec]
  stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
}

void ArmRetargetingManager::disable()
{
  for(const auto & task : {elbowTask(), wristTask()})
  {
    ctl().solver().removeTask(task);
  }

  stiffnessRatioFunc_ = nullptr;
}

const std::shared_ptr<RosPoseManager> & ArmRetargetingManager::humanWaistPoseManager() const
{
  return ctl().retargetingManagerSet_->humanWaistPoseManager_;
}

const std::shared_ptr<mc_tasks::TransformTask> & ArmRetargetingManager::elbowTask() const
{
  return ctl().retargetingTasks_.at(config_.elbowTaskName);
}

const std::shared_ptr<mc_tasks::TransformTask> & ArmRetargetingManager::wristTask() const
{
  return ctl().retargetingTasks_.at(config_.wristTaskName);
}

void ArmRetargetingManager::setTaskTarget(const std::shared_ptr<mc_tasks::TransformTask> & task,
                                          const sva::PTransformd & pose)
{
  if(const auto & impTask = std::dynamic_pointer_cast<mc_tasks::force::ImpedanceTask>(task))
  {
    impTask->targetPose(pose);
    impTask->targetVel(sva::MotionVecd::Zero());
    impTask->targetAccel(sva::MotionVecd::Zero());
  }
  else
  {
    task->target(pose);
    task->targetVel(sva::MotionVecd::Zero());
  }
}

// TODO: Set the human calibration - hugo's comment - 2026-03-13
void ArmRetargetingManager::setHumanCalibSource(const std::string & axis)
{
  if(!(humanWaistPoseManager()->isValid() && humanElbowPoseManager_->isValid() && humanWristPoseManager_->isValid()))
  {
    mc_rtc::log::error("[ArmRetargetingManager({})] Human pose is invalid. Waist: {}, Elbow: {}, Wrist: {}",
                       std::to_string(config_.armSide), humanWaistPoseManager()->isValid(),
                       humanElbowPoseManager_->isValid(), humanWristPoseManager_->isValid());
    return;
  }

  const auto & basePose = humanWaistPoseManager()->pose();
  const auto & elbowPose = humanElbowPoseManager_->pose();
  const auto & wristPose = humanWristPoseManager_->pose();
  humanCalibSource_[axis] = std::array<sva::PTransformd, 2>{elbowPose * basePose.inv(), wristPose * basePose.inv()};

  mc_rtc::log::success("[ArmRetargetingManager({})] setHumanCalibSource({}) succeeded.",
                       std::to_string(config_.armSide), axis);
}

void ArmRetargetingManager::setRobotCalibSource(const std::string & axis)
{

  // test 03-04-2026
  if(!ctl().retargetingManagerSet_->calibRobots_)
  {
    mc_rtc::log::error("calibRobots_ is null");
    return;
  }
  ////


  ctl().retargetingManagerSet_->makeCalibRobot();

  auto & calibRobot = ctl().retargetingManagerSet_->calibRobots_->robot();
  for(const auto & [jointName, jointPos] : config_.robotCalibPostures.at(axis))
  {
    // test 03-04-2026
    if(!calibRobot.hasJoint(jointName))
    {
      mc_rtc::log::error("Joint {} not found!", jointName);
      return;
    }
    ////

    calibRobot.q()[calibRobot.jointIndexByName(jointName)][0] = jointPos;
  }
  calibRobot.forwardKinematics();

  const auto & basePose = calibRobot.frame(ctl().retargetingManagerSet_->config().robotBaseLinkName).position();
  const auto & elbowPose = calibRobot.frame(elbowTask()->frame().name()).position();

  mc_rtc::log::info("Looking for frame: {}", wristTask()->frame().name()); // test 03-04-2026
  const auto & wristPose = calibRobot.frame(wristTask()->frame().name()).position();
  robotCalibSource_[axis] = std::array<sva::PTransformd, 2>{elbowPose * basePose.inv(), wristPose * basePose.inv()};

  mc_rtc::log::success("[ArmRetargetingManager({})] setRobotCalibSource({}) succeeded.",
                       std::to_string(config_.armSide), axis);
}

void ArmRetargetingManager::updateCalib()
{
  ctl().retargetingManagerSet_->clearCalibRobot();

  if(!(humanCalibSource_.count("X") && humanCalibSource_.count("Y") && humanCalibSource_.count("Z")
       && robotCalibSource_.count("X") && robotCalibSource_.count("Y") && robotCalibSource_.count("Z")))
  {
    mc_rtc::log::error("[ArmRetargetingManager({})] Calibration source is missing. Human: (X: {}, Y: {}, Z: {}), "
                       "Robot: (X: {}, Y: {}, Z: {})",
                       std::to_string(config_.armSide), humanCalibSource_.count("X"), humanCalibSource_.count("Y"),
                       humanCalibSource_.count("Z"), robotCalibSource_.count("X"), robotCalibSource_.count("Y"),
                       robotCalibSource_.count("Z"));
    return;
  }

  calibResult_.isInitialized = true;
  mc_rtc::log::warning("Calibration loaded: {}", calibResult_.isInitialized);

  calibResult_.humanTransFromBaseToShoulder = calcShoulderPoseForCalib(humanCalibSource_);
  calibResult_.robotTransFromBaseToShoulder = calcShoulderPoseForCalib(robotCalibSource_);

  calibResult_.elbowRotTransFromHumanToRobot = robotCalibSource_.at("X")[0] * humanCalibSource_.at("X")[0].inv();
  calibResult_.elbowRotTransFromHumanToRobot.translation().setZero();
  calibResult_.wristRotTransFromHumanToRobot = robotCalibSource_.at("X")[1] * humanCalibSource_.at("X")[1].inv();
  calibResult_.wristRotTransFromHumanToRobot.translation().setZero();

  const auto & humanElbowWristLengths =
      calcLimbLengthsForCalib(humanCalibSource_, calibResult_.humanTransFromBaseToShoulder);
  const auto & robotElbowWristLengths =
      calcLimbLengthsForCalib(robotCalibSource_, calibResult_.robotTransFromBaseToShoulder);
  calibResult_.elbowScale = robotElbowWristLengths[0] / humanElbowWristLengths[0];
  calibResult_.wristScale = robotElbowWristLengths[1] / humanElbowWristLengths[1];

  mc_rtc::log::success("[ArmRetargetingManager({})] Calibration result:\n{}", std::to_string(config_.armSide),
                       calibResult_.dump());
}

sva::PTransformd ArmRetargetingManager::calcShoulderPoseForCalib(const CalibSource & calibSource) const
{
  sva::PTransformd pose;

  std::vector<std::string> axes = {"X", "Y", "Z"};
  Eigen::Matrix3d posMat, dirMat;
  for(size_t i = 0; i < axes.size(); i++)
  {
    posMat.col(i) = calibSource.at(axes[i])[0].translation();


    // test commented 2026-04-02
    auto raw_d = calibSource.at(axes[i])[1].translation() - calibSource.at(axes[i])[0].translation();

    mc_rtc::log::info("[CALIB DEBUG] Axis {} raw d = {}", axes[i], raw_d.transpose());
    mc_rtc::log::info("[CALIB DEBUG] Axis {} norm = {}", axes[i], raw_d.norm());

    if(raw_d.norm() < 1e-6)
    {
      mc_rtc::log::error("[CALIB ERROR] Axis {} has near-zero direction!", axes[i]);
    }

    dirMat.col(i) = raw_d.normalized();

    mc_rtc::log::info("[CALIB DEBUG] Axis {} normalized d = {}", axes[i], dirMat.col(i).transpose());

    // og line below 
    //dirMat.col(i) = (calibSource.at(axes[i])[1].translation() - calibSource.at(axes[i])[0].translation()).normalized();

    // 18-03-2026 freezed temporarily to see if it fixes the left/right arm mirroring mismatch
    if(config_.armSide == ArmSide::Right && axes[i] == "Y")
    {
      // note selim 17-03-2026, check here if issue with hand mirroring ?
      posMat.col(i).y() *= -1.0;
      dirMat.col(i).y() *= -1.0;
    }
  }

  {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    auto addContribution = [&](const Eigen::Vector3d & P, const Eigen::Vector3d & d) {
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      A += I - d * d.transpose();
      b += (I - d * d.transpose()) * P;
    };

    for(size_t i = 0; i < axes.size(); i++)
    {
      addContribution(posMat.col(i), dirMat.col(i));
    }

    pose.translation() = A.ldlt().solve(b);
  }

  {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(dirMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d rotation = U * V.transpose();

    pose.rotation() = rotation.transpose();
  }

  return pose;
}

std::array<double, 2> ArmRetargetingManager::calcLimbLengthsForCalib(const CalibSource & calibSource,
                                                                     const sva::PTransformd & shoulderPose) const
{
  std::array<double, 2> elbowWristLengths = {0.0, 0.0};

  std::vector<std::string> axes = {"X", "Y", "Z"};
  for(size_t i = 0; i < axes.size(); i++)
  {
    for(int j = 0; j < 2; j++)
    {
      elbowWristLengths[j] += (calibSource.at(axes[i])[j].translation() - shoulderPose.translation()).norm();
    }
  }

  for(int j = 0; j < 2; j++)
  {
    elbowWristLengths[j] /= static_cast<double>(axes.size());
  }

  return elbowWristLengths;
}