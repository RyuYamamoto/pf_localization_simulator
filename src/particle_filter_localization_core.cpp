#include <particle_filter_localization/particle_filter_localization.h>

ParticleFilterLocalization::ParticleFilterLocalization()
{
  pnh_.param<int>("particle_num", particle_num_, 100);
  pnh_.param<std::string>("frame_id", frame_id_, "world");

  particle_filter_ptr_ = boost::make_shared<ParticleFilter>(particle_num_);

  initialpose_subscriber_ = pnh_.subscribe("/initialpose", 1, &ParticleFilterLocalization::initialposeCallback, this);
  pose_subscriber_ =
    pnh_.subscribe("/nav_sim/current_pose", 1, &ParticleFilterLocalization::poseCallback, this);
  particle_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("particle", 1);
}

void ParticleFilterLocalization::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  particle_filter_ptr_->setBasePose(msg.pose.pose);
}

void ParticleFilterLocalization::poseCallback(const geometry_msgs::PoseStamped & msg)
{
  latest_stamp_ = ros::Time::now();

  publishParticles();
}

void ParticleFilterLocalization::publishParticles()
{
  geometry_msgs::PoseArray particle_array;

  particle_array.header.stamp = latest_stamp_;
  particle_array.header.frame_id = frame_id_;

  for(std::size_t idx=0;idx<particle_filter_ptr_->getParticleSize();++idx) {
    const auto particle = particle_filter_ptr_->getParticle(idx);

    particle_array.poses.push_back(particle.pose);
  }

  particle_publisher_.publish(particle_array);
}
