/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_hyq/A1leg_inverse_kinematics.h>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include "/home/tianhu/TO/src/towr/towr_ros/math/constants.h"
#include <algorithm>
#include <glog/logging.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>

namespace xpp
{

  A1legInverseKinematics::Vector3d
  A1legInverseKinematics::GetJointAngles(const Vector3d &ee_pos_B, KneeBend bend, int hip_id, int robot_type) const
  {

    double x = ee_pos_B[X];
    double y = ee_pos_B[Y];
    double z = ee_pos_B[Z];
    Eigen::Vector3d pos = Eigen::Vector3d(x, y, z);
    // std::cout
    //     << "ee_pos_B:\n"
    //     << ee_pos_B << std::endl;

    Eigen::Vector3d hip_offset;
    // std::cout
    //     << "robot_type:\n"
    //     << robot_type << std::endl;
    // std::cout
    //     << "pos in here:\n"
    //     << pos << std::endl;
    // robot_type = 5;
    // std::cout
    //     << "hip_id:\n"
    //     << hip_id << std::endl;

    if (robot_type == 5)
    {
      switch (hip_id)
      {
      case 0: // FL Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0], base2hip_LF_[1] - leg_offset_y, base2hip_LF_[2] + leg_offset_x);

        break;
      case 1: // FR Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0], -base2hip_LF_[1] + leg_offset_y, base2hip_LF_[2] + leg_offset_x);
        break;
      case 2: // RL Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0], base2hip_LF_[1] - leg_offset_y, base2hip_LF_[2] + leg_offset_x);

        break;
      case 3: // RR Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0], base2hip_LF_[1] + leg_offset_y, base2hip_LF_[2] + leg_offset_x);

        break;
      default:
        // Invalid hip_id, return zero vector or handle error
        return Eigen::Vector3d::Zero();
      }
    }
    else
    {
      switch (hip_id)
      {
      case 0: // FL Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0] - leg_offset_x, base2hip_LF_[1] - leg_offset_y, 0);

        break;
      case 1: // FR Hip Joint
        hip_offset = Eigen::Vector3d(base2hip_LF_[0] - leg_offset_x, -base2hip_LF_[1] + leg_offset_y, 0);
        break;
      case 2: // RL Hip Joint
        hip_offset = Eigen::Vector3d(-base2hip_LF_[0] + leg_offset_x, base2hip_LF_[1] - leg_offset_y, 0);

        break;
      case 3: // RR Hip Joint
        hip_offset = Eigen::Vector3d(-base2hip_LF_[0] + leg_offset_x, -base2hip_LF_[1] + leg_offset_y, 0);

        break;
      default:
        // Invalid hip_id, return zero vector or handle error
        return Eigen::Vector3d::Zero();
      }
    }

    // Transform foot position from world frame to hip frame
    pos = (pos + hip_offset);

    // if (hip_id == 0)
    // {
    //   std::cout
    //       << "Left_Front:\n"
    //       << std::endl;
    // }
    // else if (hip_id == 1)
    // {
    //   std::cout
    //       << "Right_Front:\n"
    //       << std::endl;
    // }
    // else if (hip_id == 2)
    // {
    //   std::cout
    //       << "Left_Back:\n"
    //       << std::endl;
    // }
    // else if (hip_id == 3)
    // {
    //   std::cout
    //       << "Right_Back:\n"
    //       << std::endl;
    // }

    // std::cout
    //     << "pos:\n"
    //     << pos << std::endl;

    // Define leg parameters
    double l1 = hip_length;
    double l2 = thigh_length;
    double l3 = calf_length;
    double q_Hip, q_Thigh, q_Calf; // Joint angles for hip, thigh, and calf joints
    // target_position = TransformWorldFrameToHipFrame(target_position, base_position, hip_id);
    // Extract target coordinates
    x = pos[0];
    y = pos[1];
    z = pos[2];
    // x = ee_pos_B[X];
    // y = ee_pos_B[Y];
    // z = ee_pos_B[Z];

    int sideSign = (hip_id == 0 || hip_id == 2) ? 1 : -1;

    double dyz = std::sqrt(y * y + z * z);
    double lyz = std::sqrt(dyz * dyz - l1 * l1);
    double gamma_1 = std::atan2(z, y);
    double gamma_2 = std::atan2(lyz, l1);
    // double R_gamma = gamma_1 - gamma_2;
    // double L_gamma = gamma_1 + gamma_2;
    double gamma = gamma_1 + sideSign * gamma_2;
    q_Hip = gamma;
    double lxz = std::sqrt(lyz * lyz + x * x);
    double n = (lxz * lxz - l3 * l3 - l2 * l2) / (2 * l2 * l3);
    // double beta = -std::acos(std::clamp(n, -1.0, 1.0));
    double beta = -std::acos(n);
    q_Calf = beta;
    double m = (l3 * l3 - lxz * lxz - l2 * l2) / (2 * l2 * lxz);
    double alpha_1 = std::atan2(lxz, x);
    double alpha_2 = -std::acos(m);
    q_Thigh = -sideSign * (M_PI_2 - sideSign * (alpha_1 + alpha_2));
    if (q_Thigh > M_PI_2)
    {
      q_Thigh = q_Thigh - M_PI;
    }
    else if (q_Thigh < -M_PI_2)
    {
      q_Thigh = q_Thigh + M_PI;
    }
    if (q_Hip > M_PI_2)
    {
      q_Hip = q_Hip - M_PI;
    }
    else if (q_Hip < -M_PI_2)
    {
      q_Hip = q_Hip + M_PI;
    }
    if (robot_type == 5)
    {
      q_Thigh += M_PI_2;
    }

    // backward knee bend
    // EnforceLimits(q_Hip, Hip);
    // EnforceLimits(q_Thigh, Thigh);
    // EnforceLimits(q_Calf, Calf);
    // q_Hip = 0.1;
    // q_Thigh = M_PI_4 * 3;
    // q_Thigh = 1.57065;
    // q_Calf = 0;
    // q_Calf = -1.5711;
    // 0.748136

    // std::cout
    //     << "q_Hip:\n"
    //     << q_Hip << std::endl;
    // std::cout
    //     << "q_Thigh:\n"
    //     << q_Thigh << std::endl;
    // std::cout
    //     << "q_Calf:\n"
    //     << q_Calf << std::endl;
    Eigen::Vector3d joint_angles;
    joint_angles[0] = q_Hip;
    joint_angles[1] = q_Thigh;
    joint_angles[2] = q_Calf;
    Vector3d test_pos = ForwardKinematics(joint_angles, hip_id, robot_type);
    // std::cout
    //     << "test_pos:\n"
    //     << test_pos << std::endl;
    return Vector3d(q_Hip, q_Thigh, q_Calf);
  }

  void A1legInverseKinematics::EnforceLimits(double &val, A1JointID joint) const
  {
    // totally exaggerated joint angle limits
    const static double hip_min = -46;
    const static double hip_max = 46;

    const static double thigh_min = -60;
    const static double thigh_max = 240;

    const static double calf_min = -52.5;
    const static double calf_max = 154.5;

    // reduced joint angles for optimization
    static const std::map<A1JointID, double> max_range{
        {Hip, hip_max / 180.0 * M_PI},
        {Thigh, thigh_max / 180.0 * M_PI},
        {Calf, calf_max / 180.0 * M_PI}};

    // reduced joint angles for optimization
    static const std::map<A1JointID, double> min_range{
        {Hip, hip_min / 180.0 * M_PI},
        {Thigh, thigh_min / 180.0 * M_PI},
        {Calf, calf_min / 180.0 * M_PI}};

    double max = max_range.at(joint);
    val = val > max ? max : val;

    double min = min_range.at(joint);
    val = val < min ? min : val;
  }
  // Compute the Jacobian
  A1legInverseKinematics::Vector3d
  A1legInverseKinematics::ForwardKinematics(Eigen::Matrix<double, 3, 1> joint_angles, int hip_id, int robot_type) const
  {

    double l1 = hip_length;
    double l2 = thigh_length;
    double l3 = calf_length;

    // Calssify the leg position in left or right
    int sideSign = (hip_id == 0 || hip_id == 2) ? 1 : -1;

    Eigen::Vector3d position;
    double s1 = std::sin(joint_angles[0]); // hip joint
    double s2 = std::sin(joint_angles[1]); // thigh joint
    double s3 = std::sin(joint_angles[2]); // calf joint

    double c1 = std::cos(joint_angles[0]); // hip joint
    double c2 = std::cos(joint_angles[1]); // thigh joint
    double c3 = std::cos(joint_angles[2]); // calf joint

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;
    position[0] = -l3 * s23 - l2 * s2;                                 // x-coordinate
    position[1] = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1; // y-coordinate
    position[2] = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2; // z-coordinate

    // position[0] = std::abs(position[0]);

    return position;
  }

} /* namespace xpp */
