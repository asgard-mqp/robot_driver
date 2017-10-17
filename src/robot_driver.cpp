#include <ros/ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_publisher");
  ros::NodeHandle n;
  int baud_rate;
  std::string frame_id;

  n.getParam("/robot_driver/port", port);
  n.getParam("/robot_driver/baud_rate", baud_rate);
  n.getParam("/robot_driver/frame_id", frame_id);
  ROS_INFO("Running with port: %s and baud rate: %d", port.c_str(), baud_rate);

  boost::asio::io_service io;
  tf::Transform transform;


  try
  {

    while (ros::ok())
    {
     //ros::Time::now();
	
      ros::spinOnce();
    }

    return 0;
  }
  catch (boost::system::system_error ex)
  {
    ROS_ERROR("robot_driver: Error instantiating robot object. Are you sure you have the correct port and baud rate? Error was: %s", ex.what());
    return -1;
  }
}