/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
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
*   * Neither the name of the Willow Garage nor the names of its
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
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// aruco api: https://docs.opencv.org/3.4/d9/d53/aruco_8hpp.html
#include <opencv2/aruco.hpp>
#include <aruco_markers/Marker.h>
#include <aruco_markers/MarkerArray.h>

#include <ros/console.h>

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "find_marker_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  // Publishers
  aruco_markers::MarkerArray marker_array;
  ros::Publisher pub_marker = nh.advertise<aruco_markers::MarkerArray>("/markers", 1);

  // raw/original image publisher
  sensor_msgs::ImagePtr img_raw_msg;
  image_transport::Publisher pub_raw_img = it.advertise("/camera/image_raw", 480*640);

  // drawn marker on image publisher
  sensor_msgs::ImagePtr img_marker_msg;
  image_transport::Publisher pub_marker_img = it.advertise("/camera/image_marker", 480*640);

  // retrieving parameters from launch file
  std::vector<double> cam_matrix_data;
  nh.getParam("/find_marker_node/camera_matrix/data", cam_matrix_data);

  std::vector<double> dist_coeffs_data;
  nh.getParam("/find_marker_node/distortion_coefficients/data", dist_coeffs_data);

  int video_device_num; // video device number
  nh.getParam("/find_marker_node/video_device_num", video_device_num);

  int lr; // loop rate
  nh.getParam("/find_marker_node/loop_rate", lr);

  float marker_size;
  nh.getParam("/find_marker_node/marker_size", marker_size);

  // predefined aruco marker dictionary
  cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

  // aruco markers parameters
  std::vector<int> marker_ids; // ids of markers detected
  std::vector<std::vector<cv::Point2f>> marker_corners; // vector locations of detected markers' corners (clockwise order; pixel coordinates)
  std::vector<cv::Vec3d> rvecs, tvecs; // rotation & translation vectors of detected markers

  // calibrated camera parameters
  cv::Mat camera_matrix(cv::Size(3,3), CV_64F, cam_matrix_data.data());
  cv::Mat distort_coeffs(cv::Size(1,5), CV_64F, dist_coeffs_data.data());

  // opens up camera device
  cv::VideoCapture capture(video_device_num);
  cv::Mat image; // drawn on image
  cv::Mat raw_img; // copied raw imag

  uint count = 0;
  ros::Rate loop_rate(lr);

  while (nh.ok())
  {
    // capture new image frame
    capture >> image;

    if(!image.empty() && cv::sum(image-raw_img)[0] != 0) // checks if a new image
    {
      image.copyTo(raw_img);

      marker_array.markers.clear();
      marker_ids.clear();

      cv::aruco::detectMarkers(image, dict, marker_corners, marker_ids);

      if (marker_ids.size() > 0)
      {
        aruco_markers::Marker marker;

        cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, distort_coeffs, rvecs, tvecs);

        // publishes to /markers topic
        for(int i = 0; i < marker_ids.size(); ++i)
        {
          marker.header.frame_id = "aruco_markers";
          marker.header.stamp = ros::Time::now();
          marker.header.seq = count;

          marker.id = marker_ids[i];

          marker.rvec.x = rvecs[i][0];
          marker.rvec.y = rvecs[i][1];
          marker.rvec.z = rvecs[i][2];

          marker.tvec.x = tvecs[i][0];
          marker.tvec.y = tvecs[i][1];
          marker.tvec.z = tvecs[i][2];

          marker_array.markers.push_back(marker);

          cv::aruco::drawAxis(image, camera_matrix, distort_coeffs, rvecs[i], tvecs[i], marker_size);
        }
      }
      
      // cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
      // cv::imshow("image", image);
      // cv::waitKey(1);

      img_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_img).toImageMsg();
      pub_raw_img.publish(img_raw_msg);

      img_marker_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      pub_marker_img.publish(img_marker_msg);

  		pub_marker.publish(marker_array);
      ++count;
	  }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
