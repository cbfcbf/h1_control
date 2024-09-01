#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

class TestController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  TestController() = default;
  ~TestController() override = default;

  // Initialize the controller
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override {
    // Retrieve all joint handles from the EffortJointInterface
    const auto& joint_names = hw->getNames();
    for (const auto& joint_name : joint_names) {
      try {
        joint_handles_.emplace_back(hw->getHandle(joint_name));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Failed to get handle for joint: " << joint_name);
        return false;
      }
    }

    if (joint_handles_.empty()) {
      ROS_ERROR("No joints found in the EffortJointInterface.");
      return false;
    }

    ROS_INFO_STREAM("Initialized TestController with " << joint_handles_.size() << " joints.");
    return true;
  }

  // Called when the controller is started
  void starting(const ros::Time& time) override {
    ROS_INFO("Starting TestController");
  }

  // The main control loop
  void update(const ros::Time& time, const ros::Duration& period) override {
    ROS_INFO("Updating TestController:");

    for (auto& joint_handle : joint_handles_) {
      // Print joint name and current command (effort)
      ROS_INFO_STREAM("Joint Name: " << joint_handle.getName()
                      << ", Commanded Effort: " << joint_handle.getCommand());
      // joint_handle.setCommand(20);
      // joint_handle.getName();
    }
  }

  // Called when the controller is stopped
  void stopping(const ros::Time& time) override {
    ROS_INFO("Stopping TestController");
  }

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;  // Handles to the joints
};
// Export the controller as a plugin
PLUGINLIB_EXPORT_CLASS(TestController, controller_interface::ControllerBase)



namespace legged {
class TestController2 : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
public:
  TestController2() = default;
  ~TestController2() override = default;

  // Initialize the controller
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override {
    // Retrieve all joint handles from the EffortJointInterface
    auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"left_hip_yaw_joint","left_hip_roll_joint","left_hip_pitch_joint","left_knee_joint","left_ankle_joint",
    "right_hip_yaw_joint","right_hip_roll_joint","right_hip_pitch_joint","right_knee_joint","right_ankle_joint","torso_joint",
    "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint",
    "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint"};
    for (const auto& joint_name : joint_names) {
      hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
    }
    auto* contactInterface = robot_hw->get<ContactSensorInterface>();
    std::vector<std::string> contact_names{"right_ankle_link", "left_ankle_link"};

    for (const auto& name : contact_names) {
    contactHandles_.push_back(contactInterface->getHandle(name));
    }
    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
    return true;
  }

  // Called when the controller is started
  void starting(const ros::Time& time) override {
    ROS_INFO("Starting TestController");
  }

  // The main control loop
  void update(const ros::Time& time, const ros::Duration& period) override {
    // ROS_INFO("Updating TestController, and set command position=10");
    for (size_t j=0; j<19;j++){
      hybridJointHandles_[j].setCommand(0, 0, 100, 3, 0);}
    for (size_t j=0; j<2;j++){  
    bool iscontact=contactHandles_[j].isContact();
    ROS_INFO("Handle %zu: %s", j, iscontact ? "true" : "false");
    }

    auto orientation_ = imuSensorHandle_.getOrientation();
    ROS_INFO("IMU Orientation - x: %f, y: %f, z: %f, w: %f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
  }
  

  // Called when the controller is stopped
  void stopping(const ros::Time& time) override {
    ROS_INFO("Stopping TestController");
  }

protected:
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;
};
}

// Export the controller as a plugin
PLUGINLIB_EXPORT_CLASS(legged::TestController2, controller_interface::ControllerBase)


