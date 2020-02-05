/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <message_filters/subscriber.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#include <boost/thread.hpp>
#undef min
#undef max

#include "laser_scan_matcher/Math.h"
#include "laser_scan_matcher/karto_tools.h"

#define MAP_IDX(sx, i, j) (sx * j + i)

namespace scan_tools {

class LaserScanMatcher {
 public:
  LaserScanMatcher();
  ~LaserScanMatcher();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request& req,
                   nav_msgs::GetMap::Response& res);

 private:
  // Ros handle
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;

  tf::Transform base_to_laser_;  // static, cached

  // Publisher
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::ServiceServer ss_;

  // Coordinate parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;

  // Keyframe parameters
  double kf_dist_linear_;
  double kf_dist_linear_sq_;
  double kf_dist_angular_;

  boost::mutex map_mutex_;
  boost::mutex map_to_odom_mutex_;

  bool initialized_;
  bool got_map_;

  tf::Transform f2b_;  // fixed-to-base tf (pose of base frame in fixed frame)
  tf::Transform f2b_kf_;  // pose of the last keyframe scan in fixed frame

  tf::Transform odom_to_base_tf;

  sm_params input_;
  sm_result output_;
  LDP prev_ldp_scan_;

  // Grid map parameters
  double resolution_;

  // The map will be published / send to service callers
  nav_msgs::GetMap::Response map_;
  ros::Duration map_update_interval_;

  tf::Transform map_to_odom_;
  boost::thread* transform_thread_;

  std::map<std::string, LaserRangeFinder*> lasers_;
  std::map<std::string, bool> lasers_inverted_;
  bool inverted_laser_;

  LocalizedRangeScanVector allScans_;

  // Methods
  bool processScan(LaserRangeFinder* laser,
                   const sensor_msgs::LaserScan::ConstPtr& scan);
  void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp);
  void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);

  bool newKeyframeNeeded(const tf::Transform& d);

  void publishTransform();
  void publishLoop(double transform_publish_period);

  bool getOdomPose(tf::Transform& odom_to_base_tf, const ros::Time& t);
  LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
  LocalizedRangeScan* addScan(LaserRangeFinder* laser,
                              const sensor_msgs::LaserScan::ConstPtr& scan,
                              const tf::Transform& odom_to_base_tf);
  bool updateMap();

};  // LaserScanMatcher

}  // namespace scan_tools

#endif  // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
