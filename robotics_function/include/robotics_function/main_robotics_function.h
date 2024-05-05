#ifndef MAIN_ROBOTICS_FUNCTION_H
#define MAIN_ROBOTICS_FUNCTION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <cmath>

Eigen::MatrixXd defineJacoDH(const std::vector<double>& jointState);
Eigen::Matrix4d homogeneous(const Eigen::Vector4d& rowDH);
Eigen::Matrix4d directKinematics(const std::vector<double>& jointState, const int& nJoints);
Eigen::MatrixXd jacobian4jaco(const std::vector<double>& jointState);
double raisedCos(const double& sigma, const double& lambda, const double& threshold);
Eigen::MatrixXd pinvDamped(const Eigen::MatrixXd& J);

Eigen::Vector3d quaternionError(const Eigen::Quaterniond& QD, const Eigen::Quaterniond& QE);
Eigen::Quaterniond rot2quat(const Eigen::Matrix3d& R);
Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& Q);
Eigen::AngleAxisd rot2angleaxis(const Eigen::Matrix3d& R);
Eigen::Matrix3d angleaxis2rot(const Eigen::AngleAxisd& AA);
Eigen::Matrix3d rpy2rot(const double& roll, const double& pitch, const double& yaw);

double computePosition(const double& qi, const double& qf, const double& qddot, const double& tf, const double& tc, const double& t);
double computeVelocity(const double& qddot, const double& tf, const double& tc, const double& t);
double computeAcceleration(const double& qddot, const double& tf, const double& tc, const double& t);
Eigen::Vector3d trapezoidalProfile(const double& qi, const double& qf, const double& qdot, const double& tf, const double& t);
Eigen::Vector3d planSegment(const Eigen::Vector3d& pi, const Eigen::Vector3d& pf, const double& tf, const double& velocity, const double& t);
std::vector<Eigen::Vector3d> planSegmentWithVelocity(const Eigen::Vector3d& pi, const Eigen::Vector3d& pf, const double& tf, const double& velocity, const double& t);
Eigen::Quaterniond planOrientation(const Eigen::Quaterniond& QI, const Eigen::Quaterniond& QF, const double& tf, const double& velocity, const double& t);
std::vector<Eigen::Quaterniond> planOrientationWithVelocity(const Eigen::Quaterniond& QI, const double& tf, const double& velocity, const double& t, Eigen::Vector3d rotAxis, double rotAngle);

std::vector<double> convertToJaco2(const std::vector<double>& jointState);
std::vector<double> convertToDH(const std::vector<double>& jointState);

std::vector<std::vector<double>> applyIK(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const std::vector<double>& jointState, const double& Gain);
std::vector<std::vector<double>> applyIK2(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const Eigen::Vector3d& vd, const Eigen::Vector3d& omegad, const std::vector<double>& jointState, const double& Gain);
std::vector<std::vector<double>> applyIK2withNull(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const Eigen::Vector3d& vd, const Eigen::Vector3d& omegad, const std::vector<double>& jointState, const double& Gain, const Eigen::VectorXd& nullVelocity);
double computeManipulabity(const std::vector<double>& jointConfig);
Eigen::VectorXd computeNullVelocity(const std::vector<double>& jointConfig, const double& h, const double& k0);

#endif