-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom", --"base_link",
  odom_frame = "odom",

  -- If enabled, the local, non-loop-closed, continuous pose will be published as the odom_frame in the map_frame.
  provide_odom_frame = false, --true,

  -- If enabled, the published pose will be restricted to a pure 2D pose (no roll, pitch, or z-offset).
  -- This prevents potentially unwanted out-of-plane poses in 2D mode that can occur due to the pose extrapolation step
  -- (e.g. if the pose shall be published as a ‘base-footprint’-like frame)
  publish_frame_projected_to_2d = true, --false,

  -- If enabled, subscribes to nav_msgs/Odometry on the topic “odom”. Odometry must be provided in this case, and the information will be included in SLAM.
  use_odometry = true, --false,

  -- If enabled, subscribes to sensor_msgs/NavSatFix(GPS) on the topic “fix”. Navigation data must be provided in this case, and the information will be included in the global SLAM.
  use_nav_sat = false,

  -- If enabled, subscribes to cartographer_ros_msgs/LandmarkList on the topic named “landmarks”. Landmarks must be provided in this case, and the information will be included in SLAM.
  use_landmarks = false,

  -- Number of laser scan topics to subscribe to. Subscribes to sensor_msgs/LaserScan on the “scan” topic for one laser scanner, or topics “scan_1”, “scan_2”, etc. for multiple laser scanners.
  num_laser_scans = 1, --0,
  -- Number of multi-echo laser scan topics to subscribe to. Subscribes to sensor_msgs/MultiEchoLaserScan on the “echoes” topic for one laser scanner, or topics “echoes_1”, “echoes_2”, etc. for multiple laser scanners.
  num_multi_echo_laser_scans = 0, --1,

  -- Number of point clouds to split each received (multi-echo) laser scan into. Subdividing a scan makes it possible to unwarp scans acquired while the scanners are moving. There is a corresponding trajectory builder option to accumulate the subdivided scans into a point cloud that will be used for scan matching.
  num_subdivisions_per_laser_scan = 10, --1
  -- Number of point cloud topics to subscribe to. Subscribes to sensor_msgs/PointCloud2 on the “points2” topic for one rangefinder, or topics “points2_1”, “points2_2”, etc. for multiple rangefinders.
  num_point_clouds = 0,

  -- Timeout in seconds to use for looking up transforms using tf2.
  lookup_transform_timeout_sec = 5.2, --0.2,

  -- Interval in seconds at which to publish the submap poses, e.g. 0.3 seconds.
  submap_publish_period_sec = 0.1, --0.3

  -- Interval in seconds at which to publish poses, e.g. 5e-3 for a frequency of 200 Hz.
  pose_publish_period_sec = 5e-3,

  -- Interval in seconds at which to publish the trajectory markers, e.g. 30e-3 for 30 milliseconds.
  trajectory_publish_period_sec = 30e-3,

  -- Fixed ratio sampling for range finders messages.
  rangefinder_sampling_ratio = 1.0,

  -- Fixed ratio sampling for odometry messages.
  odometry_sampling_ratio = 0.1, --0.5,

  -- Fixed ratio sampling for fixed frame messages.
  fixed_frame_pose_sampling_ratio = 1.0, --0.5,

  --Fixed ratio sampling for IMU messages.
  imu_sampling_ratio = 0.5,

  --Fixed ratio sampling for landmarks messages.
  landmarks_sampling_ratio = 1,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-------------- Input --
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.min_range = 0.3 -- to tune local slam for lower latency, Decrease
TRAJECTORY_BUILDER_2D.max_range = 10. -- to tune local slam for lower latency, Decrease
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
--TRAJECTORY_BUILDER_nD.imu_gravity_time_constant =

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1 --10.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2



-------------- Local SLAM --
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- to tune local slam for lower latency, Decrease
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- to tune local slam for lower latency, Decrease
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 --  to tune local slam for lower latency, Increase
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- to tune local slam for lower latency, Increase
--TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02 -- Increase
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50 -- to tune local slam for lower latency, Decrease



POSE_GRAPH.optimization_problem.huber_scale = 1e2
-------------- Global SLAM --
POSE_GRAPH.optimize_every_n_nodes = 35  -- to reduce global slam latency, Decrease
--MAP_BUILDER.num_background_threads = 2 -- to reduce global slam latency, Increase up to number of cores
--POSE_GRAPH.global_sampling_ratio = 0.00001 -- to reduce global slam latency, Decrease
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001 -- to reducd global slam latency, Decrease
POSE_GRAPH.constraint_builder.min_score = 0.65 -- to reduce global slam latency, Increase
--POSE_GRAPH.global_constraint_search_after_n_seconds = 20 -- Increase
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease



return options
