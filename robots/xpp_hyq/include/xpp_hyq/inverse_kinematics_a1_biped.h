#ifndef XPP_VIS_INVERSEKINEMATICS_A1_BIPED_H_
#define XPP_VIS_INVERSEKINEMATICS_A1_BIPED_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_hyq/A1leg_inverse_kinematics.h>
// #include "/home/tianhu/TO/src/xpp/robots/xpp_hyq/include/xpp_hyq/A1leg_inverse_kinematics.h"
#include "/home/tianhu/TO/src/xpp/robots/xpp_hyq/include/xpp_hyq/hyqleg_inverse_kinematics.h"

namespace xpp
{

  /**
   * @brief Inverse kinematics function for the HyQ robot.
   */
  class InverseKinematicsA1Biped : public InverseKinematics
  {
  public:
    InverseKinematicsA1Biped() = default;
    virtual ~InverseKinematicsA1Biped() = default;

    /**
     * @brief Returns joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B).
     */
    Joints GetAllJointAngles(const EndeffectorsPos &pos_B) const override;

    /**
     * @brief Number of endeffectors (feet, hands) this implementation expects.
     */
    int GetEECount() const override { return 4; };

  private:
    Vector3d base2hip_LF_ = Vector3d(0.1805, 0.135, 0.0);
    int robot_type = 5;

    // HyqlegInverseKinematics leg;
    A1legInverseKinematics leg;
  };

} /* namespace xpp */

#endif /* XPP_VIS_INVERSEKINEMATICS_HYQ4_H_ */
