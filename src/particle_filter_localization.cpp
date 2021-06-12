#include <particle_filter_localization/particle_filter_localization.h>

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
  particle_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("particle", 1);

  motion_noise_std_vec_(0) = sigma_vv_ * sigma_vv_;
  motion_noise_std_vec_(1) = sigma_vw_ * sigma_vw_;
  motion_noise_std_vec_(2) = sigma_wv_ * sigma_wv_;
  motion_noise_std_vec_(3) = sigma_ww_ * sigma_ww_;
  motion_noise_covariance_ = motion_noise_std_vec_.asDiagonal();
}

void ParticleFilterLocalization::initialposeCallback(
  const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  particle_filter_ptr_->setBasePose(msg.pose.pose);
}

void ParticleFilterLocalization::poseCallback(const geometry_msgs::PoseStamped & msg)
{
  latest_stamp_ = ros::Time::now();

  particle_filter_ptr_->setBasePose(msg.pose);

  publishParticles();
}

void ParticleFilterLocalization::publishParticles()
{
  geometry_msgs::PoseArray particle_array;

  particle_array.header.stamp = latest_stamp_;
  particle_array.header.frame_id = frame_id_;

  for (std::size_t idx = 0; idx < particle_filter_ptr_->getParticleSize(); ++idx) {
    const auto particle = particle_filter_ptr_->getParticle(idx);

    particle_array.poses.push_back(particle.pose);
  }

  particle_publisher_.publish(particle_array);
}
