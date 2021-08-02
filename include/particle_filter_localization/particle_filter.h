#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_sim/LandmarkInfoArray.h>

#include <particle_filter_localization/multi_variate_generator.h>

struct Particle
{
  Eigen::VectorXd vec;
  double weight;
};

class ParticleFilter
{
public:
  ParticleFilter(const int particle_num) : particle_num_(particle_num)
  {
    particle_.resize(particle_num);
  }
  ~ParticleFilter() = default;

  std::size_t getParticleSize() { return particle_.size(); }
  Particle getParticle(std::size_t idx) { return particle_.at(idx); }

  double normalizeDegree(const double degree)
  {
    double normalize_deg = std::fmod((degree + 180.0), 360.0) - 180.0;
    if (normalize_deg < -180.0) normalize_deg += 360.0;
    return normalize_deg;
  }

  void setMap(std::map<std::string, std::pair<double, double>> landmarks)
  {
    landmarks_to_map_ = landmarks;
  }

  void initParticles(const Eigen::VectorXd vec)
  {
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      particle_.at(idx).vec = vec;
      particle_.at(idx).weight = 1.0 / particle_.size();
    }
  }

  void setParticleNum(const int particle_num) { particle_num_ = particle_num; }

  Eigen::VectorXd motion(
    const double vel, const double omega, const double dt, const Eigen::VectorXd pose)
  {
    Eigen::VectorXd diff_pose(pose.rows());
    const double t0 = pose(2);

    diff_pose(2) = omega * dt;
    if (std::fabs(omega) < 1e-10) {
      diff_pose(0) = vel * std::cos(t0) * dt;
      diff_pose(1) = vel * std::sin(t0) * dt;
    } else {
      diff_pose(0) = (vel / omega) * (std::sin(t0 + omega * dt) - std::sin(t0));
      diff_pose(1) = (vel / omega) * (-std::cos(t0 + omega * dt) + std::cos(t0));
    }

    return pose + diff_pose;
  }

  void motionUpdate(
    const double vel, const double omega, const double dt, const Eigen::Vector4d motion_noise)
  {
    MultiVariateNormal multi_variate_normal(
      Eigen::VectorXd::Zero(motion_noise.rows()), motion_noise.asDiagonal());
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      Eigen::VectorXd noise = multi_variate_normal();
      const double noise_vel = vel + noise(0) * std::sqrt(std::fabs(vel) / dt) +
                               noise(1) * std::sqrt(std::fabs(omega) / dt);
      const double noise_omega = omega + noise(2) * std::sqrt(std::fabs(vel) / dt) +
                                 noise(3) * std::sqrt(std::fabs(omega) / dt);

      particle_.at(idx).vec = motion(noise_vel, noise_omega, dt, particle_.at(idx).vec);
    }
  }

  void observationUpdate(
    const nav_sim::LandmarkInfoArray & observations, const double distance_rate,
    const double direction_rate)
  {
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      for (const auto observation : observations.landmark_array) {
        Eigen::VectorXd observation_pos(2);
        observation_pos << observation.length, observation.theta;

        // calculate landmark distance and direction each particle pose
        std::pair<double, double> landmark_to_map = landmarks_to_map_[std::to_string(observation.id)];
        tf2::Transform map_to_particle;
        tf2::Transform map_to_landmark;

        map_to_particle.setOrigin(tf2::Vector3(particle_.at(idx).vec(0), particle_.at(idx).vec(1), 0.0));
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, particle_.at(idx).vec(2));
        map_to_particle.setRotation(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
        map_to_landmark.setOrigin(tf2::Vector3(landmark_to_map.first, landmark_to_map.second, 0.0));

        tf2::Transform particle_to_landmark = map_to_particle.inverse() * map_to_landmark;

        const double diff_landmark_theta =
          std::atan2(
            map_to_landmark.getOrigin().y() - map_to_particle.getOrigin().y(),
            map_to_landmark.getOrigin().x() - map_to_particle.getOrigin().x()) -
          particle_.at(idx).vec(2);

        const double landmark_direction = normalizeDegree(diff_landmark_theta * 180.0 * M_PI) * M_PI / 180.0;
        const double landmark_distance = std::hypot(particle_to_landmark.getOrigin().x(), particle_to_landmark.getOrigin().y());

        const double distance_dev = distance_rate * landmark_distance;
        Eigen::VectorXd mean(2);
        mean << landmark_distance, landmark_direction;
        Eigen::MatrixXd covariance(2,2);
        covariance << std::pow(distance_dev, 2), 0.0, 0.0, std::pow(direction_rate, 2);

        MultiVariateNormal multi_variate_normal(mean, covariance);
        particle_.at(idx).weight *= multi_variate_normal.pdf(observation_pos);
        std::cout << particle_.at(idx).weight << std::endl;
      }
    }
  }

private:
  int particle_num_;

  std::vector<Particle> particle_;
  std::map<std::string, std::pair<double, double>> landmarks_to_map_;
};

#endif
