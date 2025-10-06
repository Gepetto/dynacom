/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef DYNACOM_CONTACT_BASE
#define DYNACOM_CONTACT_BASE

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/spatial/se3.hpp"
#include <memory>
// clang-format on

namespace dynacom {

class ContactBase {
 protected:
  pinocchio::SE3 oMs_, cMo_;

  Eigen::VectorXd contactForce_;
  size_t frameID_;
  // matrices
  Eigen::MatrixXd unilaterality_A_;
  Eigen::VectorXd unilaterality_b_;
  Eigen::MatrixXd friction_A_;
  Eigen::VectorXd friction_b_;
  Eigen::VectorXd regularization_A_;
  Eigen::VectorXd regularization_b_;
  Eigen::Matrix<double, 6, -1> newton_euler_A_;

 public:
  // ContactBase() {}
  //  virtual ~Contact() {}

  // setters
  void setPose(pinocchio::SE3& pose) { oMs_ = pose; }
  void setFrameID(const size_t frameID) { frameID_ = frameID; }
  void applyForce(const Eigen::MatrixXd& force) { contactForce_ << force; }
  void deactivate() { contactForce_.setZero(); }

  virtual void updateNewtonEuler(const Eigen::Vector3d& CoM,
                                 const pinocchio::SE3& oMf) = 0;

  // getters
  const pinocchio::SE3& getPose() const { return oMs_; }
  size_t getFrameID() const { return frameID_; }
  size_t uni_rows() const { return unilaterality_A_.rows(); }
  size_t fri_rows() const { return friction_A_.rows(); }
  size_t cols() const { return newton_euler_A_.cols(); }
  const Eigen::VectorXd& appliedForce() { return contactForce_; }

  const Eigen::MatrixXd& uni_A() { return unilaterality_A_; }
  const Eigen::VectorXd& uni_b() { return unilaterality_b_; }
  const Eigen::MatrixXd& fri_A() { return friction_A_; }
  const Eigen::VectorXd& fri_b() { return friction_b_; }
  const Eigen::VectorXd& reg_A() { return regularization_A_; }
  const Eigen::VectorXd& reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, -1>& ne_A() { return newton_euler_A_; }

  const Eigen::Matrix<double, 6, 6> toWorldForces() {
    return oMs_.toActionMatrixInverse().transpose();
  }
  const Eigen::Matrix<double, 6, 6> toCoMForces() {
    return cMo_.act(oMs_).toActionMatrixInverse().transpose();
  }

  virtual std::string& getFrameName() = 0;
};

}  // namespace dynacom

#endif  //// DYNACOM_CONTACT_BASE
