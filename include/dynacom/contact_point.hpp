/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef DYNACOM_CONTACT_POINT
#define DYNACOM_CONTACT_POINT

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/spatial/se3.hpp"
#include <memory>
#include "contact_base.hpp"
// clang-format on

namespace dynacom {

struct ContactPointSettings {
 public:
  double mu;
  Eigen::Matrix<double, 3, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D " << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    weights: " << this->weights.transpose() << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator==(const ContactPointSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->weights == rhs.weights;
    return test;
  }

  bool operator!=(const ContactPointSettings &rhs) { return !(*this == rhs); }
};

class ContactPoint : public ContactBase {
 private:
  ContactPointSettings settings_;

 public:
  ContactPoint();
  ContactPoint(const ContactPointSettings &settings);
  void initialize(const ContactPointSettings &settings);

  // setters
  void setMu(const double &mu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);

  // getters
  const ContactPointSettings &getSettings() { return settings_; }
  std::string &getFrameName() { return settings_.frame_name; }
};

}  // namespace dynacom

#endif  // DYNACOM_CONTACT_POINT
