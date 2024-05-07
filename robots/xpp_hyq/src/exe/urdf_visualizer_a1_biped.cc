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
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: for a1 robot
 * Author: Zhu Yijie
 * Create: 2022-1-10
 * Notes: xx
 * Modify:
 */
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>
#include <xpp_hyq/inverse_kinematics_a1.h>
#include <xpp_hyq/inverse_kinematics_a1_biped.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
// using namespace quad;
using namespace biped;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "a1_biped_urdf_visualizer");

  const std::string topic_name = xpp_msgs::robot_state_current;
  const std::string joint_a1_biped = "xpp/joint_a1_des";
  // const std::string joint_desired_a1 = xpp_msgs::joint_desired;

  auto a1_ik = std::make_shared<InverseKinematicsA1Biped>();
  // a1_ik.GetAllJointAngles()
  // CartesianJointConverter inv_kin_converter(a1_ik,
  // 				    xpp_msgs::robot_state_current, // "/xpp/state_des"
  // 				    topic_name);
  CartesianJointConverter inv_kin_converter(a1_ik,
                                            xpp_msgs::robot_state_desired, // "/xpp/state_des"
                                            joint_a1_biped);
  // std::cout<< "joint angle is:" + xpp_msgs::joint_desired<<std::endl;

  // urdf joint names
  int n_ee = a1_ik->GetEECount();

  int n_j = A1legJointCount; // 3
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee * n_j);
  // joint_names.at(n_j * FootIDs::LF + Hip) = "FL_hip_joint";
  // joint_names.at(n_j * FootIDs::LF + Thigh) = "FL_thigh_joint";
  // joint_names.at(n_j * FootIDs::LF + Calf) = "FL_calf_joint";
  // joint_names.at(n_j * FootIDs::RF + Hip) = "FR_hip_joint";
  // joint_names.at(n_j * FootIDs::RF + Thigh) = "FR_thigh_joint";
  // joint_names.at(n_j * FootIDs::RF + Calf) = "FR_calf_joint";
  joint_names.at(n_j * FootIDs::L + Hip) = "RL_hip_joint";
  joint_names.at(n_j * FootIDs::L + Thigh) = "RL_thigh_joint";
  joint_names.at(n_j * FootIDs::L + Calf) = "RL_calf_joint";
  joint_names.at(n_j * FootIDs::R + Hip) = "RR_hip_joint";
  joint_names.at(n_j * FootIDs::R + Thigh) = "RR_thigh_joint";
  joint_names.at(n_j * FootIDs::R + Calf) = "RR_calf_joint";

  std::string urdf = "a1_biped_rviz_urdf_robot_description";
  UrdfVisualizer a1_biped_desired(urdf, joint_names, "base", "world",
                                  joint_a1_biped, "a1_biped_des");

  ::ros::spin();

  return 1;
}
