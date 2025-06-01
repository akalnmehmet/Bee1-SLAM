-- Beemobs Bee1 3D Cartographer Configuration
-- For detailed 3D mapping when needed

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Map Builder Configuration
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true

-- Trajectory Builder 3D Configuration
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 100.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- Motion Filter
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 1.0
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(10.0)

-- High Resolution Adaptive Voxel Filter
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.

-- Low Resolution Adaptive Voxel Filter
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.

-- Real-Time Correlative Scan Matcher
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- Ceres Scan Matcher
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- Submap Configuration
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49

-- IMU Configuration
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 1.

-- Pose Graph 3D Configuration
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 160
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options