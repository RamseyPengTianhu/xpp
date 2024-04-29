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
  A1legInverseKinematics::GetJointAngles(const Vector3d &ee_pos_B, KneeBend bend, int hip_id) const
  {

    double x = ee_pos_B[X];
    double y = ee_pos_B[Y];
    double z = ee_pos_B[Z];
    Eigen::Vector3d pos = Eigen::Vector3d(x, y, z);

    double leg_offset_x_ = base2hip_LF_[0];
    double leg_offset_y_ = base2hip_LF_[1];
    Eigen::Vector3d FR_Hip_Trans(leg_offset_x_, -leg_offset_y_, 0);
    Eigen::Vector3d FL_Hip_Trans(leg_offset_x_, leg_offset_y_, 0);
    Eigen::Vector3d RR_Hip_Trans(-leg_offset_x_, -leg_offset_y_, 0);
    Eigen::Vector3d RL_Hip_Trans(-leg_offset_x_, leg_offset_y_, 0);
    switch (hip_id)
    {
    case 0:
      // FL Hip Joint
      pos = pos - FL_Hip_Trans;

      break;
    case 1:
      // FR Hip Joint
      pos = pos - FR_Hip_Trans;

      break;
    case 2:
      // RL Hip Joint
      pos = pos - RL_Hip_Trans;

      break;
    case 3:
      // RR Hip Joint
      pos = pos - RR_Hip_Trans;

      break;
    default:
      break;
    }

    // Define leg parameters
    double l1 = hip_length;
    double l2 = thigh_length;
    double l3 = calf_length;
    double q_Hip, q_Thigh, q_Calf; // Joint angles for hip, thigh, and calf joints
    // target_position = TransformWorldFrameToHipFrame(target_position, base_position, hip_id);
    // Extract target coordinates
    // x = pos[0];
    // y = pos[1];
    // z = pos[2];
    x = ee_pos_B[X];
    y = ee_pos_B[Y];
    z = ee_pos_B[Z];

    int sideSign = (hip_id == 0 || hip_id == 2) ? 1 : -1;

    double gamma_1, gamma_2;
    double dyz = std::sqrt(y * y + z * z);
    double lyz = std::sqrt(dyz * dyz - l1 * l1);

    // // Classify the leg position in left or right

    gamma_1 = std::atan2(z, y);
    gamma_2 = std::atan2(lyz, l1);
    q_Hip = gamma_1 + sideSign * gamma_2;

    double lxz = std::sqrt(lyz * lyz + x * x);

    double n = (lxz * lxz - l3 * std::cos(q_Hip) * l3 * std::cos(q_Hip) - l2 * std::cos(q_Hip) * l2 * std::cos(q_Hip)) / (2 * l2 * std::cos(q_Hip) * l3 * std::cos(q_Hip));
    q_Calf = sideSign * std::acos(n);
    double m = (l3 * std::cos(q_Hip) * l3 * std::cos(q_Hip) - lxz * lxz - l2 * std::cos(q_Hip) * l2 * std::cos(q_Hip)) / (2 * l2 * std::cos(q_Hip) * lxz);
    // double n = (lxz * lxz - l3 * l3 - l2 * l2) / (2 * l2 * l3);
    // q_Calf = -std::acos(n);
    // double m = (l3 * l3 - lxz * lxz - l2 * l2) / (2 * l2 * lxz);

    double alpha_1, alpha_2;
    alpha_1 = std::atan2(lyz, x);
    alpha_2 = std::acos(m);
    q_Thigh = -sideSign * (M_PI / 2 - sideSign * (alpha_1 + alpha_2));

    // backward knee bend
    EnforceLimits(q_Hip, Hip);
    EnforceLimits(q_Thigh, Thigh);
    EnforceLimits(q_Calf, Calf);

    return Vector3d(q_Hip, q_Thigh, q_Calf);
  }

  void
  A1legInverseKinematics::EnforceLimits(double &val, A1JointID joint) const
  {
    // totally exaggerated joint angle limits
    const static double haa_min = -180;
    const static double haa_max = 90;

    const static double hfe_min = -90;
    const static double hfe_max = 90;

    const static double kfe_min = -180;
    const static double kfe_max = 0;

    // reduced joint angles for optimization
    static const std::map<A1JointID, double> max_range{
        {Hip, haa_max / 180.0 * M_PI},
        {Thigh, hfe_max / 180.0 * M_PI},
        {Calf, kfe_max / 180.0 * M_PI}};

    // reduced joint angles for optimization
    static const std::map<A1JointID, double> min_range{
        {Hip, haa_min / 180.0 * M_PI},
        {Thigh, hfe_min / 180.0 * M_PI},
        {Calf, kfe_min / 180.0 * M_PI}};

    double max = max_range.at(joint);
    val = val > max ? max : val;

    double min = min_range.at(joint);
    val = val < min ? min : val;
  }

} /* namespace xpp */
