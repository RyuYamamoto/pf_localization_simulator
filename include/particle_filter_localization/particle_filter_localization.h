#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core>

#include <particle_filter_localization/particle_filter.h>

class ParticleFilterLocalization
{
public:
  ParticleFilterLocalization();
  ~ParticleFilterLocalization() = default;

private:
  void poseCallback(const geometry_msgs::PoseStamped & msg);
  void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped & msg);

  void publishParticles();

  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Time latest_stamp_;

  ros::Subscriber initialpose_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Publisher particle_publisher_;

  boost::shared_ptr<ParticleFilter> particle_filter_ptr_;

  int particle_num_;
  std::string frame_id_;

  double sigma_vv_;
  double sigma_vw_;
  double sigma_wv_;
  double sigma_ww_;
  Eigen::Vector4d motion_noise_std_vec_;
  Eigen::Matrix4d motion_noise_covariance_;
};
