#ifndef _PARTICLE_FILTER_LOCALIZATION_H_
#define _PARTICLE_FILTER_LOCALIZATION_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_sim_msgs/msg/landmark_info_array.hpp>

#include <random>

#include <yaml-cpp/yaml.h>

#include <particle_filter_localization/particle_filter.h>

class ParticleFilterLocalization : public rclcpp::Node
{
public:
  ParticleFilterLocalization();
  ~ParticleFilterLocalization() = default;

private:
  void twistCallback(const geometry_msgs::msg::TwistStamped & msg);
  void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void observationCallback(const nav_sim_msgs::msg::LandmarkInfoArray & msg);

  geometry_msgs::msg::Pose estimatedCurrentPose();

  void publishParticles(const rclcpp::Time stamp);

  void parseYaml(const std::string filename);

private:

  rclcpp::Time latest_stamp_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<nav_sim_msgs::msg::LandmarkInfoArray>::SharedPtr observation_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_publisher_;

  geometry_msgs::msg::TwistStamped twist_;
  geometry_msgs::msg::TwistStamped prev_twist_;

  std::shared_ptr<ParticleFilter> particle_filter_ptr_;

  int particle_num_;
  std::string frame_id_;
  std::string map_file_path_;

  double distance_rate_;
  double direction_rate_;

  double sigma_vv_;
  double sigma_vw_;
  double sigma_wv_;
  double sigma_ww_;
  Eigen::Vector4d motion_noise_std_vec_;
};

#endif
