/*
Author: Curtis Manore
File: getMatrix.cpp
Description: displays the transformation matrix between two ROS tf frames

*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>  



int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");


  ros::NodeHandle n;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //ros::Publisher chatter_pub = n.advertise<tf2::transformToEigen>("chatter", 1000);

  ros::Rate loop_rate(10);


  while (ros::ok())
  {


    //ROS_INFO("%s", msg.data.c_str());
    try
    {
      geometry_msgs::TransformStamped transform_to_robot = tfBuffer.lookupTransform("cam_0_optical_frame","imu",ros::Time::now(),ros::Duration(0.05));
      Eigen::Affine3d transform_matrix = tf2::transformToEigen(transform_to_robot);
    

      //chatter_pub.publish(transform_matrix);
      std::cout << transform_matrix.matrix();
      std::cout << "\n";
    }
   catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   ros::Duration(1.0).sleep();
   continue;
 }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
