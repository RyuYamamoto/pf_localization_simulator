#include <particle_filter_localization/particle_filter_localization.h>
#include <particle_filter_localization/utils.h>

ParticleFilterLocalization::ParticleFilterLocalization() : Node("particle_filter_localization")
{
  particle_num_ = this->declare_parameter("particle_num", 100);
  frame_id_ = this->declare_parameter("frame_id", "map");
  map_file_path_ = this->declare_parameter("map_file_path", "");
  sigma_vv_ = this->declare_parameter("sigma_vv", 0.01);
  sigma_vw_ = this->declare_parameter("sigma_vw", 0.02);
  sigma_wv_ = this->declare_parameter("sigma_wv", 0.03);
  sigma_ww_ = this->declare_parameter("sigma_ww", 0.04);
  distance_rate_ = this->declare_parameter("distance_rate", 0.14);
  direction_rate_ = this->declare_parameter("direction_rate", 0.05);

  particle_filter_ptr_ = std::make_shared<ParticleFilter>(particle_num_);

  initialpose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&ParticleFilterLocalization::initialposeCallback, this, std::placeholders::_1));
  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "twist", 1, std::bind(&ParticleFilterLocalization::twistCallback, this, std::placeholders::_1));
  observation_subscriber_ = this->create_subscription<nav_sim_msgs::msg::LandmarkInfoArray>(
    "/nav_sim/observation", 11,
    std::bind(&ParticleFilterLocalization::observationCallback, this, std::placeholders::_1));

  particle_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle", 1);
  estimated_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);

  motion_noise_std_vec_(0) = sigma_vv_ * sigma_vv_;
  motion_noise_std_vec_(1) = sigma_vw_ * sigma_vw_;
  motion_noise_std_vec_(2) = sigma_wv_ * sigma_wv_;
  motion_noise_std_vec_(3) = sigma_ww_ * sigma_ww_;

  // init particles
  particle_filter_ptr_->initParticles(Eigen::Vector3d::Zero());

  parseYaml(map_file_path_);

  latest_stamp_ = rclcpp::Clock().now();
}

void ParticleFilterLocalization::parseYaml(const std::string filename)
{
  YAML::Node config = YAML::LoadFile(filename);

  int id = -1;
  std::map<std::string, std::pair<double, double>> landmarks;
  for (YAML::const_iterator itr = config.begin(); itr != config.end(); ++itr) {
    const std::string landmark_id = std::to_string(++id);
    const double pos_x = itr->second["x"].as<double>();
    const double pos_y = itr->second["y"].as<double>();
    landmarks[landmark_id] = std::make_pair(pos_x, pos_y);
  }
  particle_filter_ptr_->setMap(landmarks);
}

void ParticleFilterLocalization::initialposeCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  particle_filter_ptr_->initParticles(utils::convertToVector(msg.pose.pose));
}

void ParticleFilterLocalization::twistCallback(const geometry_msgs::msg::TwistStamped & msg)
{
  const rclcpp::Time current_stamp = rclcpp::Clock().now();
  const double dt = (current_stamp - latest_stamp_).seconds();

  // update particle pose
  particle_filter_ptr_->motionUpdate(
    prev_twist_.twist.linear.x, prev_twist_.twist.angular.z, dt, motion_noise_std_vec_);

  // publish estimated pose from particles
  geometry_msgs::msg::PoseStamped estimated_pose;
  estimated_pose.header.frame_id = frame_id_;
  estimated_pose.header.stamp = rclcpp::Clock().now();
  estimated_pose.pose = estimatedCurrentPose();
  estimated_pose_publisher_->publish(estimated_pose);

  // publish particles for visualization
  publishParticles(current_stamp);

  latest_stamp_ = current_stamp;
  prev_twist_ = msg;
}

void ParticleFilterLocalization::observationCallback(
  const nav_sim_msgs::msg::LandmarkInfoArray & msg)
{
  if (msg.landmark_array.empty()) return;
  particle_filter_ptr_->observationUpdate(msg, distance_rate_, direction_rate_);
  particle_filter_ptr_->resampling();
}

geometry_msgs::msg::Pose ParticleFilterLocalization::estimatedCurrentPose()
{
  geometry_msgs::msg::Pose result;
  double highest_weight = 0.0;

  for (std::size_t idx = 0; idx < particle_filter_ptr_->getParticleSize(); ++idx) {
    const auto p = particle_filter_ptr_->getParticle(idx);
    if (highest_weight < p.weight) {
      highest_weight = p.weight;
      result = utils::convertToPose(p.vec);
    }
  }

  return result;
}

void ParticleFilterLocalization::publishParticles(const rclcpp::Time stamp)
{
  geometry_msgs::msg::PoseArray particle_array;

  particle_array.header.stamp = stamp;
  particle_array.header.frame_id = frame_id_;

  for (std::size_t idx = 0; idx < particle_filter_ptr_->getParticleSize(); ++idx) {
    const auto particle = particle_filter_ptr_->getParticle(idx);

    particle_array.poses.push_back(utils::convertToPose(particle.vec));
  }

  particle_publisher_->publish(particle_array);
}
