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

#include <laser_scan_matcher/laser_scan_matcher.h>

FILE * poseFile = std::fopen("/home/wenjian/Experiment/plicp_pose.txt", "w+");

namespace scan_tools 
{

LaserScanMatcher::LaserScanMatcher():
  initialized_(false),
  got_map_(false)
{
  // Initiate parameters
  map_to_odom_.setIdentity();
  ros::NodeHandle nh_private_("~");
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if (!nh_private_.getParam ("map_frame", map_frame_))
    map_frame_ = "map";

  // Keyframe params: when to generate the keyframe scan.
  // If either is set to 0, reduces to frame-to-frame matching

  if (!nh_private_.getParam ("kf_dist_linear", kf_dist_linear_))
    kf_dist_linear_ = 0.10;
  if (!nh_private_.getParam ("kf_dist_angular", kf_dist_angular_))
    kf_dist_angular_ = 10.0 * (M_PI / 180.0);

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  if(!nh_private_.getParam("resolution", resolution_))
    resolution_ = 0.025;
  double tmp;
  if(!nh_private_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  double transform_publish_period;
  nh_private_.param("transform_publish_period", transform_publish_period, 0.05);
  
  tfB_ = new tf::TransformBroadcaster();

  // CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;

  // State variables

  f2b_.setIdentity();
  f2b_kf_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // Publishers
  sst_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1,true);
  sstm_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = nh_.advertiseService("dynamic_map", &LaserScanMatcher::mapCallback, this);

  // Subscribers
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&LaserScanMatcher::scanCallback, this, _1));
  
  transform_thread_ = new boost::thread(boost::bind(&LaserScanMatcher::publishLoop, this, transform_publish_period));
}

LaserScanMatcher::~LaserScanMatcher()
{
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void LaserScanMatcher::publishTransform()
{
  boost::mutex::scoped_lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
}

void LaserScanMatcher::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

bool LaserScanMatcher::getOdomPose(tf::Transform& odom_to_base_tf, const ros::Time& t)
{
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),   
                                            tf::Vector3(0, 0, 0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try{
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e){
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  odom_to_base_tf = odom_pose;

  return true;
}

LocalizedRangeScan* LaserScanMatcher::addScan(LaserRangeFinder* laser,
                                              const sensor_msgs::LaserScan::ConstPtr& scan, 
                                              const tf::Transform& odom_to_base_tf)
{
  // Create a vector of doubles for karto
  std::vector<double> readings;

  if (lasers_inverted_[scan->header.frame_id])
  {
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin(); it != scan->ranges.rend(); ++it)
    {
      readings.push_back(*it);
    }
  } 
  else
  {
    for(std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end(); ++it)
    {
      readings.push_back(*it);
    }
  }
  
  // create localized range scan
  LocalizedRangeScan* range_scan = new LocalizedRangeScan(laser, readings);
  range_scan->SetOdometricPose(Pose2(odom_to_base_tf.getOrigin().x(), odom_to_base_tf.getOrigin().y(), tf::getYaw(odom_to_base_tf.getRotation())));

  tf::Transform map_to_base_tf = map_to_odom_ * odom_to_base_tf;
  range_scan->SetCorrectedPose(Pose2(map_to_base_tf.getOrigin().x(), map_to_base_tf.getOrigin().y(), tf::getYaw(map_to_base_tf.getRotation())));
  
  return range_scan;
}

void LaserScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
  static ros::Time last_map_update(0,0);
  
  // Check whether we know about this laser yet
  LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if(processScan(laser, scan))
  {
    map_to_odom_mutex_.lock();
    map_to_odom_ = f2b_kf_ * odom_to_base_tf.inverse();
    map_to_odom_mutex_.unlock();
    
    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      if(updateMap())
      {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

bool LaserScanMatcher::processScan(LaserRangeFinder* laser, const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if(!getOdomPose(odom_to_base_tf, scan->header.stamp))
    return false;

  // If first scan, cache the tf from base to the scanner
  if (!initialized_)
  {
    input_.min_reading = scan->range_min;
    input_.max_reading = scan->range_max;

    laserScanToLDP(scan, prev_ldp_scan_);
    
    LocalizedRangeScan* pScan = addScan(laser, scan, odom_to_base_tf);
    allScans_.push_back(pScan);
    
    initialized_ = true;
    
    return true;
  }
  
  LDP curr_ldp_scan;
  laserScanToLDP(scan, curr_ldp_scan);
  
  LocalizedRangeScan* pScan = addScan(laser, scan, odom_to_base_tf);

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // estimated change since last scan
  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = (f2b_kf_ * base_to_laser_).inverse() * map_to_odom_ * odom_to_base_tf * base_to_laser_;
  
  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // Scan matching - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf::Transform corr_ch;

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * base_to_laser_.inverse();

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;
  }
  else
  {
    corr_ch.setIdentity();
    ROS_WARN("Error in scan matching");
  }
  
  // **** swap old and new

  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
    
    Pose2 corrected_pose;
    corrected_pose.SetX(f2b_kf_.getOrigin().x());
    corrected_pose.SetY(f2b_kf_.getOrigin().y());
    corrected_pose.SetHeading(tf::getYaw(f2b_kf_.getRotation()));
    
    pScan->SetCorrectedPose(corrected_pose);
    allScans_.push_back(pScan);
  
    std::fprintf(poseFile, "%.6f\t\%.6f\t\%.6f\n",
                 pScan->GetCorrectedPose().GetX(), 
                 pScan->GetCorrectedPose().GetY(), 
                 pScan->GetCorrectedPose().GetHeading());

    return true;
  }
  else
  {
    ld_free(curr_ldp_scan);
    delete pScan;
    
    return false;
  }
}

bool LaserScanMatcher::updateMap()
{
  boost::mutex::scoped_lock lock(map_mutex_);

  OccupancyGrid* occ_grid = OccupancyGrid::CreateFromScans(allScans_, resolution_);

  if(!occ_grid)
    return false;

  if(!got_map_) 
  {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  // Translate to ROS format
  int width = occ_grid->GetWidth();
  int height = occ_grid->GetHeight();
  Vector2<double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if(map_.map.info.width != (unsigned int) width || 
    map_.map.info.height != (unsigned int) height ||
    map_.map.info.origin.position.x != offset.GetX() ||
    map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++) 
    {
      // Getting the value at position x,y
      int value = occ_grid->GetValue(Vector2<int>(x, y));

      switch (value)
      {
        case GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
        break;
      
        case GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        break;
        
        case GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
        break;
      
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
        break;
      }
    }
  }
  
  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  delete occ_grid;

  return true;
}

bool LaserScanMatcher::newKeyframeNeeded(const tf::Transform& d)
{
  if (fabs(tf::getYaw(d.getRotation())) > kf_dist_angular_) 
    return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x * x + y * y > kf_dist_linear_sq_) 
    return true;

  return false;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp)
{
  unsigned int n = scan->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // Calculate position in laser frame
    double r = scan->ranges[i];
    if ((r > scan->range_min) && (r < scan->range_max))
    {
      // Fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }
    ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

LaserRangeFinder* LaserScanMatcher::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try{
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e){
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
             scan->header.frame_id.c_str(),
             laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(),
             yaw);
  
    base_to_laser_ = laser_pose;
    
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try{
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException& e){
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first scan
    
    LaserRangeFinder* laser = LaserRangeFinder::CreateLaserRangeFinder();
    laser->SetRangeThreshold(12.0);  // for UTM-30LX
    laser->SetOffsetPose(Pose2(laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;
  }

  return lasers_[scan->header.frame_id];
}

void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

bool LaserScanMatcher::mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

}  // namespace scan_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserScanMatcher");

  scan_tools::LaserScanMatcher laser_scan_matcher;
  
  ros::spin();
  
  return 0;
}