#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

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
};
