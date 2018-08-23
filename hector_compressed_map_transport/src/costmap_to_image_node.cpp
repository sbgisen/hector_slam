//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>

//#include <opencv2/opencv.hpp>//画像保存用ヘッダー

#include <hector_map_tools/HectorMapTools.h>

using namespace std;

/**
 * @brief This node provides occupancy grid maps as images via image_transport, so the transmission consumes less bandwidth.
 * The provided code is a incomplete proof of concept.
 */
class MapAsImageProvider
{
public:
  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    image_transport_publisher_full_ = image_transport_->advertise("costmap_image/full", 1);

    pose_sub_ = n_.subscribe("pose", 1, &MapAsImageProvider::poseCallback, this);
    map_sub_ = n_.subscribe("/move_base/local_costmap/costmap", 1, &MapAsImageProvider::mapCallback, this);

    //Which frame_id makes sense?
    cv_img_full_.header.frame_id = "costmap_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::BGRA8;//ここで画像のエンコード方式を指定

    ROS_INFO("CostMap to Image node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

  //We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    pose_ptr_ = pose;
  }

  //The map->image conversion runs every time a new map is received at the moment
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    // Only if someone is subscribed to it, do work and publish full map image
    if (image_transport_publisher_full_.getNumSubscribers() > 0){

        cv::Mat map_mat = cv::Mat(size_y, size_x, CV_8UC4);

      const std::vector<int8_t>& map_data (map->data);
      cv::Vec4b px;


      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int size_y_rev = size_y-1;

      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * (size_y_rev -y);
        //int idx_map_y = size_x * (size_y -y);

        for (int x = 0; x < size_x; ++x){

          //マップの数値に対する画像ピクセルの色を指定

          switch (map_data[idx_map_y + x])
          {
            case 0:
              px[0] = 0;
              px[1] = 0;
              px[2] = 0;
              px[3] = 0; 
              break;
            case 99:
              px[0] = 255;
              px[1] = 255;
              px[2] = 0;
              px[3] = 255;
              break;
            case 100:
              px[0] = 0;
              px[1] = 255;
              px[2] = 255;
              px[3] = 255;
              break;
            default:
              unsigned char v = (255 * map_data[idx_map_y + x]) / 100;
              px[0] = 255-v;
              px[1] = 0;
              px[2] = v;
              px[3] = 255;
              break;
          }
          map_mat.at<cv::Vec4b>(y,x) = px;
        }
      }

      cv_img_full_.image = map_mat;
      //cv::imwrite("localCostmap.png",map_mat);//画像をローカルに保存
      image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    }
  }

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;

  image_transport::Publisher image_transport_publisher_full_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStampedConstPtr pose_ptr_;

  cv_bridge::CvImage cv_img_full_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_image_node");

  MapAsImageProvider map_image_provider;

  ros::spin();

  return 0;
}
