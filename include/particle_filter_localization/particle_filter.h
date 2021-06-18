#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <particle_filter_localization/multi_variate_generator.h>

struct Particle
{
  Eigen::VectorXd vec;
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

  void setBasePose(const Eigen::VectorXd vec)
  {
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) { particle_.at(idx).vec = vec; }
  }

  void setParticleNum(const int particle_num) { particle_num_ = particle_num; }

  Eigen::VectorXd motion(
    const double vel, const double omega, const double dt, const Eigen::VectorXd pose)
  {
    Eigen::VectorXd diff_pose(pose.rows());
    const double t0 = pose(2);

    diff_pose(2) = omega * dt;
    if (std::fabs(omega) < 1e-06) {
      diff_pose(0) = vel * std::cos(t0) * dt;
      diff_pose(1) = vel * std::sin(t0) * dt;
    } else {
      diff_pose(0) = (vel / omega) * (std::sin(t0 + omega * dt) - std::sin(t0));
      diff_pose(1) = (vel / omega) * (-std::cos(t0 + omega * dt) + std::cos(t0));
    }

    return pose + diff_pose;
  }

  void update(
    const double vel, const double omega, const double dt, const Eigen::Vector4d motion_noise)
  {
    MultiVariateNormal multi_variate_normal(
      Eigen::VectorXd::Zero(motion_noise.rows()), motion_noise.asDiagonal());
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      Eigen::VectorXd noise = multi_variate_normal();
      const double noise_vel = vel + (noise(0) * std::sqrt(std::fabs(vel) / dt) +
                                      noise(1) * std::sqrt(std::fabs(omega) / dt));
      const double noise_omega = omega + (noise(2) * std::sqrt(std::fabs(vel) / dt) +
                                          noise(3) * std::sqrt(std::fabs(omega) / dt));

      particle_.at(idx).vec = motion(noise_vel, noise_omega, dt, particle_.at(idx).vec);
    }
  }

private:
  int particle_num_;

  std::vector<Particle> particle_;
};

#endif
