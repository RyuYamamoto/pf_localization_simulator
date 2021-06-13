#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include <particle_filter_localization/multi_variate_generator.h>

struct Particle
{
  geometry_msgs::Pose pose;
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

  void setBasePose(const geometry_msgs::Pose pose)
  {
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) particle_.at(idx).pose = pose;
  }

  void setParticleNum(const int particle_num) { particle_num_ = particle_num; }

  Eigen::VectorXd convertToVector(const geometry_msgs::Pose pose)
  {
    Eigen::VectorXd vec(3);

    vec(0) = pose.position.x;
    vec(1) = pose.position.y;

    tf::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf::Matrix3x3 m(q);
    double r,p,y;
    m.getRPY(r,p,y);
    vec(2) = y;

    return vec;
  }

  geometry_msgs::Pose convertToPose(const Eigen::VectorXd vec)
  {
    geometry_msgs::Pose pose;
    pose.position.x = vec(0);
    pose.position.y = vec(1);
    pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, vec(2));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
  }

  void update(
    const double velocity, const double omega, const double dt, const Eigen::Vector4d motion_noise)
  {
    MultiVariateNormal multi_variate_normal(Eigen::VectorXd::Zero(motion_noise.rows()), motion_noise.asDiagonal());
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      Eigen::VectorXd noise = multi_variate_normal();
      const double noise_vel = velocity + noise(0) * std::sqrt(std::fabs(velocity)/dt) + noise(1) * std::sqrt(std::fabs(omega)/dt);
      const double noise_omega = omega + noise(2) * std::sqrt(std::fabs(velocity)/dt) + noise(3) * std::sqrt(std::fabs(omega)/dt);

      Eigen::VectorXd vec = convertToVector(particle_.at(idx).pose);

      vec(2) += noise_omega * dt;
      vec(0) += noise_vel * std::cos(vec(2)) * dt;
      vec(1) += noise_vel * std::sin(vec(2)) * dt;

      std::cout << vec << std::endl;

      particle_.at(idx).pose = convertToPose(vec);
    }
  }

private:
  int particle_num_;

  std::vector<Particle> particle_;
};
