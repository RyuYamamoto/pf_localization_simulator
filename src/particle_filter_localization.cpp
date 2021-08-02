#include <particle_filter_localization/particle_filter_localization.h>
#include <particle_filter_localization/utils.h>

ParticleFilterLocalization::ParticleFilterLocalization()
{
  pnh_.param<int>("particle_num", particle_num_, 100);
  pnh_.param<std::string>("frame_id", frame_id_, "map");
  pnh_.param<std::string>("map_file_path", map_file_path_, "");
  pnh_.param<double>("sigma_vv", sigma_vv_, 0.01);
  pnh_.param<double>("sigma_vw", sigma_vw_, 0.02);
  pnh_.param<double>("sigma_wv", sigma_wv_, 0.03);
  pnh_.param<double>("sigma_ww", sigma_ww_, 0.04);
  pnh_.param<double>("distance_rate", distance_rate_, 0.14);
  pnh_.param<double>("direction_rate", direction_rate_, 0.05);

  particle_filter_ptr_ = boost::make_shared<ParticleFilter>(particle_num_);

  initialpose_subscriber_ =
    pnh_.subscribe("/initialpose", 1, &ParticleFilterLocalization::initialposeCallback, this);
  pose_subscriber_ =
    pnh_.subscribe("current_pose", 1, &ParticleFilterLocalization::poseCallback, this);
  twist_subscriber_ = pnh_.subscribe("twist", 1, &ParticleFilterLocalization::twistCallback, this);
  observation_subscriber_ =
    pnh_.subscribe("/nav_sim/observation", 1, &ParticleFilterLocalization::observationCallback, this);
  particle_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("particle", 1);

  motion_noise_std_vec_(0) = sigma_vv_ * sigma_vv_;
  motion_noise_std_vec_(1) = sigma_vw_ * sigma_vw_;
  motion_noise_std_vec_(2) = sigma_wv_ * sigma_wv_;
  motion_noise_std_vec_(3) = sigma_ww_ * sigma_ww_;

  // init particles
  particle_filter_ptr_->initParticles(Eigen::Vector3d::Zero());

  parseYaml(map_file_path_);

  latest_stamp_ = ros::Time::now();
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
  const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  particle_filter_ptr_->initParticles(utils::convertToVector(msg.pose.pose));
}

void ParticleFilterLocalization::twistCallback(const geometry_msgs::TwistStamped & msg)
{
  twist_ = msg;
}

void ParticleFilterLocalization::poseCallback(const geometry_msgs::PoseStamped & msg)
{
  const ros::Time current_stamp = ros::Time::now();
  const double dt = (current_stamp - latest_stamp_).toSec();

  // update particle pose
  update(prev_twist_.twist.linear.x, prev_twist_.twist.angular.z, dt);

  // publish particles for visualization
  publishParticles(current_stamp);

  latest_stamp_ = current_stamp;
  prev_twist_ = twist_;
}

void ParticleFilterLocalization::observationCallback(const nav_sim::LandmarkInfoArray & msg)
{
  observationUpdate(msg);
}

void ParticleFilterLocalization::update(const double velocity, const double omega, const double dt)
{
  particle_filter_ptr_->motionUpdate(velocity, omega, dt, motion_noise_std_vec_);
}

void ParticleFilterLocalization::observationUpdate(const nav_sim::LandmarkInfoArray observation)
{
  if(observation.landmark_array.empty()) return;
  particle_filter_ptr_->observationUpdate(observation, distance_rate_, direction_rate_);
}

void ParticleFilterLocalization::publishParticles(const ros::Time stamp)
{
  geometry_msgs::PoseArray particle_array;

  particle_array.header.stamp = stamp;
  particle_array.header.frame_id = frame_id_;

  for (std::size_t idx = 0; idx < particle_filter_ptr_->getParticleSize(); ++idx) {
    const auto particle = particle_filter_ptr_->getParticle(idx);

    particle_array.poses.push_back(utils::convertToPose(particle.vec));
  }

  particle_publisher_.publish(particle_array);
}
