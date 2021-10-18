#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <blvr_controller/blvr_interface.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "blvr_controller");

  BlvrDriveRobotHW robot_hw;
  controller_manager::ControllerManager cm(&robot_hw, robot_hw.nh);

  ros::Rate rate(1 / robot_hw.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev = ros::Time::now();

  robot_hw.init();
  while(ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - prev;

    robot_hw.read();
    cm.update(now, dt);
    robot_hw.write();

    prev = now;
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
