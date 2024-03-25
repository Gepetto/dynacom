/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef DYNACOM_CONTACT_6D
#define DYNACOM_CONTACT_6D

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/spatial/se3.hpp"
#include <memory>
#include "contact_base.hpp"
// clang-format on

namespace dynacom {

struct Contact6DSettings {
 public:
  double mu, gu;
  double half_length, half_width;
  Eigen::Matrix<double, 6, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D " << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    gu: " << this->gu << "\n";
    out << "    weights: " << this->weights.transpose() << "\n";
    out << "    Surface half_length: " << this->half_length << "\n";
    out << "    Surface half_width: " << this->half_width << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator!=(const Contact6DSettings &rhs) { return !(*this == rhs); }

  bool operator==(const Contact6DSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->gu == rhs.gu;
    test &= this->half_length == rhs.half_length;
    test &= this->half_width == rhs.half_width;
    test &= this->weights == rhs.weights;
    return test;
  }
};

class Contact6D : public ContactBase {
 private:
  Contact6DSettings settings_;

 public:
  Contact6D();
  Contact6D(const Contact6DSettings &settings);
  void initialize(const Contact6DSettings &settings);

  // setters
  void setMu(const double &mu);
  void setGu(const double &gu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void setTorqueWeights(const Eigen::Vector3d &torque_weights);
  void setSurfaceHalfWidth(const double &half_width);
  void setSurfaceHalfLength(const double &half_length);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);

  // getters
  const Contact6DSettings &getSettings() { return settings_; }
  std::string &getFrameName() { return settings_.frame_name; }
};

}  // namespace dynacom

#endif  // DYNACOM_CONTACT_6D
