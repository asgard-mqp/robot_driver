#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
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

double wheelRadius;
int gearRatioNumerator;
int gearRatioDenominator;
constexpr float inchToMeter = 0.0254;
constexpr float PI = 3.1415926535897;
boost::asio::serial_port* serial_;

void read_callback(bool& data_available, boost::asio::deadline_timer& timeout, const boost::system::error_code& error, std::size_t bytes_transferred){
  if (error || !bytes_transferred) {
    // No data was read!

    data_available = false;
    return;
  }
//  ROS_INFO("Data");

  timeout.cancel();  // will cause wait_callback to fire with an error
  data_available = true;
}

void wait_callback(boost::asio::serial_port& ser_port, const boost::system::error_code& error){
  if (error){
    // Data was read and this timeout was canceled
    return;
  }
//  ROS_INFO("no data");

  ser_port.cancel();  // will cause read_callback to fire with an error
}

union Converter {
  int32_t full_data;     // occupies 4 bytes
  uint8_t bytes[4];     // occupies 4 byte
};  

void send_UART_msg(uint8_t system_ID, int data) {
  Converter out_data;
  out_data.full_data=data;

  uint8_t startByte = 0xFA;

  boost::array<uint8_t, 7> packet;
  packet[0] = startByte;
  packet[1] = system_ID;

  for (int i=0; i<4;i++) {
    packet[i+2] = out_data.bytes[i];
  }

  uint8_t checksum=0x00; 

  for(int i=0; i<6;i++){//all the important bytes
    checksum += packet[i];
  }
  packet[6] = checksum;

  boost::asio::write(*serial_,  boost::asio::buffer(&packet[0], 7));
  /*ROS_INFO("OUT %02X:%02X:%02X:%02X:%02X:%02X:%02X",packet[0],packet[1],
    packet[2],packet[3],packet[4],packet[5],packet[6]);*/
}

int linear_to_rotationalV(float meters_per_second){
  float circumference = wheelRadius * inchToMeter *PI;//diameter * conversion * pi
  const float rps = gearRatioDenominator*(meters_per_second/circumference)/gearRatioNumerator;//rotations per second
  const int send = rps*60*360;
  //ROS_INFO("rps  %1.4f, m/s %1.4f  send %d",rps,meters_per_second,send);

  return (send);
}
void kp_Callback(const std_msgs::Float32::ConstPtr& msg) {
  const int new_const = msg->data *65536;
  ROS_INFO("kp %f",msg->data);
  send_UART_msg(0x16, new_const);
}
void ki_Callback(const std_msgs::Float32::ConstPtr& msg) {
  const int new_const = msg->data *65536;
  ROS_INFO("ki %f",msg->data);
  send_UART_msg(0x17, new_const);
}
void kd_Callback(const std_msgs::Float32::ConstPtr& msg) {
  const int new_const = msg->data *65536;
  ROS_INFO("kd %f",msg->data);
  send_UART_msg(0x18, new_const);
}


void left_Drive_Callback(const std_msgs::Float32::ConstPtr& msg) {
  const int degree_per_minute = linear_to_rotationalV(msg->data);
  send_UART_msg(0x01, degree_per_minute);
  ROS_INFO("send left %d",degree_per_minute/360);
}

void right_Drive_Callback(const std_msgs::Float32::ConstPtr& msg) {
  const int degree_per_minute = linear_to_rotationalV(msg->data);
  send_UART_msg(0x02,degree_per_minute);
  ROS_INFO("send right %d",degree_per_minute/360);
}
void arm_goal_Callback(const std_msgs::Bool::ConstPtr& msg) {
  const int new_const = msg->data?1:0;
  ROS_INFO("kd %f",msg->data);
  send_UART_msg(0x03, new_const);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_driver_node");
  ros::NodeHandle n;
  std::string port;
  int baud_rate;
  std::string frame_id;

  n.getParam("/robot_driver_node/port", port);
  n.getParam("/robot_driver_node/baud_rate", baud_rate);
  n.getParam("/robot_driver_node/frame_id", frame_id);
  n.getParam("/robot_driver_node/wheel_radius", wheelRadius);
  n.getParam("/robot_driver_node/gear_Ratio_Numerator",gearRatioNumerator);
  n.getParam("/robot_driver_node/gear_Ratio_Denominator",gearRatioDenominator);

  ROS_INFO("Running with port: %s and baud rate: %d", port.c_str(), baud_rate);
  ROS_INFO("Gear ratio  %d / %d", gearRatioNumerator, gearRatioDenominator);
  ROS_INFO("wheel radius %1.2f, inchToMeter %1.5f ,  PI  %1.4f",wheelRadius,inchToMeter,PI);
  boost::asio::io_service io;
  boost::asio::deadline_timer timeout(io);

  ros::Subscriber right_drive_sub = n.subscribe("rwheel_vtarget", 10, right_Drive_Callback);
  ros::Subscriber left_drive_sub = n.subscribe("lwheel_vtarget", 10, left_Drive_Callback);

  ros::Subscriber kp_sub = n.subscribe("kp", 10, kp_Callback);
  ros::Subscriber ki_sub = n.subscribe("ki", 10, ki_Callback);
  ros::Subscriber kd_sub = n.subscribe("kd", 10, kd_Callback);
  ros::Subscriber arm_goal_sub = n.subscribe("arm_goal", 10, arm_goal_Callback);




  ros::Publisher lwheel_pub = n.advertise<std_msgs::Int16>("lwheel", 100);
  ros::Publisher rwheel_pub = n.advertise<std_msgs::Int16>("rwheel", 100);
  ros::Publisher arm_state_pub = n.advertise<std_msgs::Int16>("arm_state", 100);


  unsigned char my_buffer[1];
  bool data_available = false;

  unsigned char packet[7];

  try {
    serial_ = new boost::asio::serial_port(io,port); // UART port for the Cortex
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    ROS_INFO("about to receive");
    while (ros::ok()) {
     //ros::Time::now();

      serial_->async_read_some(boost::asio::buffer(my_buffer),
        boost::bind(&read_callback, boost::ref(data_available), boost::ref(timeout),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
      timeout.expires_from_now(boost::posix_time::milliseconds(1)); //5 ms timeout
      timeout.async_wait(boost::bind(&wait_callback, boost::ref(*serial_),
        boost::asio::placeholders::error));
      io.run();
      io.reset();
  //    ROS_INFO("started");
      if(data_available && my_buffer[0] != 0xFA){
        ROS_INFO("wrong startByte  %02x",my_buffer[0]);
        data_available = false; // I lied no data available
      }
      if(data_available){
        packet[0] = my_buffer[0];
        boost::asio::read(*serial_, boost::asio::buffer(&packet[1], 6));
//        ROS_INFO("IN %02X:%02X:%02X:%02X:%02X:%02X:%02X",packet[0],packet[1],
//         packet[2],packet[3],packet[4],packet[5],packet[6]);
//        ROS_INFO("Start %02X",my_buffer[0]);
        std_msgs::Int16 encoder_ticks;
        Converter input;
        for (int i=0; i<4;i++) {
          input.bytes[i] = packet[i+2];
        }
        encoder_ticks.data = input.full_data;
        switch(packet[1]){

          case 0xf1://left
            //read
          lwheel_pub.publish(encoder_ticks);
 //         ROS_INFO("lwheel");
          break;
          case 0xf2://right
            //read
          rwheel_pub.publish(encoder_ticks);
          break;
          case 0xf3://arm
            //read
          arm_state_pub.publish(encoder_ticks); //really is the arm state not encoder pos
          break;
          default:
          ROS_INFO("Invalid packet subsystem ID %02X",packet[1]);
        }

      }
      data_available = 0;

      ros::spinOnce();
    }

    free(serial_);
    return 0;
  } catch (boost::system::system_error& ex) {
    ROS_ERROR("robot_driver: Error instantiating robot object. Are you sure you have the correct port and baud rate? Error was: %s", ex.what());

    return -1;
  }

  free(serial_);
}
