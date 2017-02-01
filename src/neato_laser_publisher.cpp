/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h> //new
#include <sensor_msgs/PointCloud.h> //new
#include <sensor_msgs/Image.h>
//#include <pcl/io/pcd_io.h>
#include <boost/asio.hpp>
#include <xv_11_laser_driver/xv11_laser.h>
#include <std_msgs/UInt16.h>
#include <opencv2/core.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <laser_geometry/laser_geometry.h>//new


int main(int argc, char **argv)
{
  ros::init(argc, argv, "neato_laser_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  std::string frame_id;
  int firmware_number;
 
  std_msgs::UInt16 rpms; 
  laser_geometry::LaserProjection projector_;//new
  

  priv_nh.param("port", port, std::string("/dev/ttyUSB1"));
  priv_nh.param("baud_rate", baud_rate, 115200);
  priv_nh.param("frame_id", frame_id, std::string("map"));
  priv_nh.param("firmware_version", firmware_number, 1);

  boost::asio::io_service io;

  try {
    xv_11_laser_driver::XV11Laser laser(port, baud_rate, firmware_number, io);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000); //original
    ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1000);
    ros::Publisher point_pub = n.advertise<sensor_msgs::PointCloud>("point", 1000); //new was "points"
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("lidargrid", 10);
    while (ros::ok()) {
      sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = ros::Time::now();
      laser.poll(scan);
      rpms.data=laser.rpms;
      laser_pub.publish(scan);
      sensor_msgs::PointCloud cloud; //new
      projector_.projectLaser(*scan,cloud); //new

      // Create a blank greyscale opencv image.
      // iterate over points in cloud.
      //   for each point, map to pixel.
      // convert opencv image to ros Image message
      // publish ros image message.
      //sensor_msgs::ImagePtr toImageMsg() const;
      
      int width = 100;
      cv::Mat grey = cv::Mat::zeros(width, width, CV_8UC1);


      point_pub.publish(cloud);   //new
      motor_pub.publish(rpms);
      for (int i=0; i < cloud.points.size(); i++){
      //for (; pt.x != pt.x.end(); i){
        geometry_msgs::Point32 pt = cloud.points[i];
        double x = (double)pt.x;
        double y = (double)pt.y;
        int xPixel = (int)(((x+5.0)/10.0)*width);
        int yPixel = (int)(((y+5.0)/10.0)*width);
        if(xPixel >= 0 && xPixel < width && yPixel >= 0 && yPixel < width) {   
            grey.at<uint8_t>(xPixel,yPixel)=255;
        }
        //grey.at<uint8_t>(0,0)=255;

      }
      cv_bridge::CvImage out_msg;
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      out_msg.image = grey;
      image_pub.publish(out_msg);
    }
    laser.close();
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
