#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


#include <iostream>

//Start byte 
//subsystem ID
//data 4 bytes
//checksum



void read_callback(bool& data_available, boost::asio::deadline_timer& timeout, const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (error || !bytes_transferred)
  {
    // No data was read!
    data_available = false;
    return;
  }

  timeout.cancel();  // will cause wait_callback to fire with an error
  data_available = true;
}

void wait_callback(boost::asio::serial_port& ser_port, const boost::system::error_code& error)
{
  if (error)
  {
    // Data was read and this timeout was canceled
    return;
  }

  ser_port.cancel();  // will cause read_callback to fire with an error
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_driver_node");
  ros::NodeHandle n;
  std::string port;
  int baud_rate;
  std::string frame_id;

  n.getParam("/robot_driver_node/port", port);
  n.getParam("/robot_driver_node/baud_rate", baud_rate);
  n.getParam("/robot_driver_node/frame_id", frame_id);
  ROS_INFO("Running with port: %s and baud rate: %d", port.c_str(), baud_rate);

  boost::asio::io_service io;
  boost::asio::deadline_timer timeout(io);

  unsigned char  my_buffer[1];
  bool data_available = false;
  boost::array<uint8_t, 5> out;

  out[0]='a';
  out[1]='b';
  out[2]='c';
  out[3]='d';
  out[4]='\n';

  try
  {
    boost::asio::serial_port serial_(io,port); // UART port for the Cortex
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    int runs =0;
    ROS_INFO("about to receive");

    while (ros::ok())
    {
      runs ++;
     //ros::Time::now();
      serial_.async_read_some(boost::asio::buffer(my_buffer),
        boost::bind(&read_callback, boost::ref(data_available), boost::ref(timeout),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
      timeout.expires_from_now(boost::posix_time::milliseconds(5)); //500 ms timeout
      timeout.async_wait(boost::bind(&wait_callback, boost::ref(serial_),
        boost::asio::placeholders::error));

      io.run();
      io.reset();
      //boost::asio::write(serial_,  boost::asio::buffer(&out[0], 5));

      ROS_INFO("received %d",data_available);
      data_available = 0;

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
