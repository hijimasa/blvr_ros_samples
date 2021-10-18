#include <string>
#include <ros/package.h>
#include <blvr_controller/blvr_interface.h>

BlvrDriveRobotHW::BlvrDriveRobotHW()
{
  name_ = "blvr_controller";

  if(!getMotorNames(nh, "blvr_controller/motor_name", motor_names_))
  {
    ROS_ERROR_NAMED(name_, "Wheel param is not found.");
    return;
  }
  if(!getMotorDirections(nh, "blvr_controller/motor_direction", motor_directions_))
  {
    ROS_ERROR_NAMED(name_, "Wheel param is not found.");
    return;
  }
  device_name_ = "/dev/ttyACM0";
  nh.param("blvr_controller/device_name", device_name_, device_name_);
  nh.param("blvr_controller/gear_ratio", gear_ratio_, 50.0);
  nh.param("blvr_controller/encoder_resolution", encoder_resolution_, 30.0);
  motor_num_ = motor_names_.size();

  joint_info_.resize(motor_num_);
  for(size_t i = 0; i < motor_num_; i++)
  {
    hardware_interface::JointStateHandle state_handle(motor_names_[i], &(joint_info_[i].pos_), &(joint_info_[i].vel_), &(joint_info_[i].eff_));
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(motor_names_[i]), &(joint_info_[i].cmd_));
    jnt_vel_interface.registerHandle(vel_handle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);

  serial_port_ = std::make_shared<BlvrComunicator>();
}

BlvrDriveRobotHW::~BlvrDriveRobotHW()
{
  if (serial_port_->is_open) {
    serial_port_->closeDevice();
  }
}

void
BlvrDriveRobotHW::init()
{
  if (serial_port_->openDevice(device_name_) != BlvrComunicator::return_type::SUCCESS) {
    ROS_ERROR_NAMED(name_, "Device can not be opened");
    return;
  }
}

void
BlvrDriveRobotHW::read()
{
  for (size_t i = 0; i < motor_num_; i++) 
  {
    if(serial_port_->setExcitation(i+1) != 0)
    {
      fprintf(stderr, "%s:%s:%d: can't set ecitation to motor (id: %d)\n",
                __FILE__, __func__, __LINE__, (int)i);
    } 
    //RCLCPP_INFO(rclcpp::get_logger("BlvrDriver"), "Got position %.5f, velocity %.5f for joint %d!", position_states_[i], velocity_states_[i], i);
    int rpm = 0;
    serial_port_->readRpm(i+1, &rpm);
    joint_info_[i].vel_ = static_cast<double>(motor_directions_[i] * rpm) * 2.0 * M_PI / gear_ratio_ / 60.0;
  }

}

void
BlvrDriveRobotHW::write()
{
  for (size_t i = 0; i < motor_num_; i++) 
  {
    if(serial_port_->setExcitation(i+1) != 0)
    {
      fprintf(stderr, "%s:%s:%d: can't set ecitation to motor (id: %d)\n",
                __FILE__, __func__, __LINE__, (int)i);
    } 
//    RCLCPP_INFO(rclcpp::get_logger("BlvrDriver"), "Motor velocity changed: %.5f", velocity_commands_[i]);

    // Generate the motor command message
    int rpm = motor_directions_[i] * static_cast<int>(joint_info_[i].cmd_ * 60.0 * gear_ratio_ / (2.0 * M_PI));

  std::cout << "rpm[" << i << "] = " << rpm << std::endl;
    serial_port_->directDataDrive(i+1, BlvrComunicator::RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_CURRENT, 15000000, rpm, static_cast<int>(getPeriod().toSec()*1000.0), static_cast<int>(getPeriod().toSec()*1000.0), 10000);
  }

  std::cout << "cmd[0], cmd[1] = " << joint_info_[0].cmd_ << ", " << joint_info_[1].cmd_ << std::endl;
}

bool
BlvrDriveRobotHW::getMotorNames(ros::NodeHandle& controller_nh,
  const std::string& motor_param,
  std::vector<std::string>& motor_names)
{
  XmlRpc::XmlRpcValue motor_list;
  if (!controller_nh.getParam(motor_param, motor_list))
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Couldn't retrieve motor param '" << motor_param << "'.");
    return false;
  }

  if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (motor_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_,
        "Motor param '" << motor_param << "' is an empty list.");
      return false;
    }

    for (size_t i = 0; i < motor_list.size(); i++)
    {
      if (motor_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM_NAMED(name_,
          "Motor param '" << motor_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    motor_names.resize(motor_list.size());
    for (size_t i = 0; i < motor_list.size(); i++)
    {
      motor_names[i] = static_cast<std::string>(motor_list[i]);
    }
  }
  else if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    motor_names.push_back(motor_list);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Motor param '" << motor_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

bool
BlvrDriveRobotHW::getMotorDirections(ros::NodeHandle& controller_nh,
  const std::string& motor_param,
  std::vector<int>& motor_directions)
{
  XmlRpc::XmlRpcValue motor_list;
  if (!controller_nh.getParam(motor_param, motor_list))
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Couldn't retrieve motor param '" << motor_param << "'.");
    return false;
  }

  if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (motor_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_,
        "Motor param '" << motor_param << "' is an empty list.");
      return false;
    }

    for (size_t i = 0; i < motor_list.size(); i++)
    {
      if (motor_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM_NAMED(name_,
          "Motor param '" << motor_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    motor_directions.resize(motor_list.size());
    for (size_t i = 0; i < motor_list.size(); i++)
    {
      motor_directions[i] = static_cast<int>(motor_list[i]);
    }
  }
  else if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    motor_directions.push_back(motor_list);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Motor param '" << motor_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}
