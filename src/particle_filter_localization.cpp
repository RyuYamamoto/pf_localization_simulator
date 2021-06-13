#include <particle_filter_localization/particle_filter_localization.h>
#include <particle_filter_localization/utils.h>

ParticleFilterLocalization::ParticleFilterLocalization()
{
  pnh_.param<int>("particle_num", particle_num_, 100);
  pnh_.param<std::string>("frame_id", frame_id_, "map");
  pnh_.param<double>("sigma_vv", sigma_vv_, 0.01);
  pnh_.param<double>("sigma_vw", sigma_vw_, 0.02);
  pnh_.param<double>("sigma_wv", sigma_wv_, 0.03);
  pnh_.param<double>("sigma_ww", sigma_ww_, 0.04);

  particle_filter_ptr_ = boost::make_shared<ParticleFilter>(particle_num_);

  initialpose_subscriber_ =
    pnh_.subscribe("/initialpose", 1, &ParticleFilterLocalization::initialposeCallback, this);
  pose_subscriber_ =
    pnh_.subscribe("current_pose", 1, &ParticleFilterLocalization::poseCallback, this);
  twist_subscriber_ = pnh_.subscribe("/nav_sim/twist", 1, &ParticleFilterLocalization::twistCallback, this);
  particle_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("particle", 1);

  motion_noise_std_vec_(0) = sigma_vv_ * sigma_vv_;
  motion_noise_std_vec_(1) = sigma_vw_ * sigma_vw_;
  motion_noise_std_vec_(2) = sigma_wv_ * sigma_wv_;
  motion_noise_std_vec_(3) = sigma_ww_ * sigma_ww_;

  latest_stamp_ = ros::Time::now();
}

void ParticleFilterLocalization::initialposeCallback(
  const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  particle_filter_ptr_->setBasePose(utils::convertToVector(msg.pose.pose));
}

void ParticleFilterLocalization::twistCallback(const geometry_msgs::TwistStamped & msg)
{
  twist_ = msg;
}

void ParticleFilterLocalization::update(const double velocity, const double omega, const double dt)
{
  particle_filter_ptr_->update(velocity, omega, dt, motion_noise_std_vec_);
}

void ParticleFilterLocalization::poseCallback(const geometry_msgs::PoseStamped & msg)
{
  current_stamp_ = ros::Time::now();

  // set current position
  particle_filter_ptr_->setBasePose(utils::convertToVector(msg.pose));

  // update particle pose
  update(twist_.twist.linear.x, twist_.twist.angular.z, (current_stamp_-latest_stamp_).toSec());

  // publish particles for visualization
  publishParticles();

  latest_stamp_ = current_stamp_;
}

void ParticleFilterLocalization::publishParticles()
{
  geometry_msgs::PoseArray particle_array;

  particle_array.header.stamp = current_stamp_;
  particle_array.header.frame_id = frame_id_;

  for (std::size_t idx = 0; idx < particle_filter_ptr_->getParticleSize(); ++idx) {
    const auto particle = particle_filter_ptr_->getParticle(idx);

    particle_array.poses.push_back(utils::convertToPose(particle.vec));
  }

  particle_publisher_.publish(particle_array);
}
