#include <iostream>
#include <cmath>
#include <array>
#include <Eigen/Dense>

class A1LegIKController {
public:
    A1LegIKController(double hip_length, double thigh_length, double calf_length,
                      double leg_offset_x, double leg_offset_y, double trunk_offset_z,
                      double hip_max, double hip_min,
                      double thigh_max, double thigh_min,
                      double calf_max, double calf_min,
                      double trunk_ixx, double hip_ixx, double thigh_ixx, double calf_ixx)
        : hip_length_(hip_length), thigh_length_(thigh_length), calf_length_(calf_length),
          leg_offset_x_(leg_offset_x), leg_offset_y_(leg_offset_y), trunk_offset_z_(trunk_offset_z),
          hip_max_(hip_max), hip_min_(hip_min),
          thigh_max_(thigh_max), thigh_min_(thigh_min),
          calf_max_(calf_max), calf_min_(calf_min),
          trunk_ixx_(trunk_ixx), hip_ixx_(hip_ixx), thigh_ixx_(thigh_ixx), calf_ixx_(calf_ixx) {}

    // Solve inverse kinematics to obtain joint angles
    std::array<double, 3> solveIK(const std::array<double, 3>& target_position,
                                  const std::array<double, 3>& current_joint_angles,
                                  const std::array<double, 3>& current_cartesian_velocities) {
        // Compute Jacobian matrix
        Eigen::MatrixXd jacobian = computeJacobian(current_joint_angles);

        // Pseudo-inverse of the Jacobian
        Eigen::MatrixXd jacobian_pseudo_inverse = pseudoInverse(jacobian);

        // Cartesian velocities to joint velocities
        Eigen::VectorXd cartesian_velocities(3);
        for (int i = 0; i < 3; ++i) {
            cartesian_velocities(i) = current_cartesian_velocities[i];
        }

        Eigen::VectorXd joint_velocities_eigen = jacobian_pseudo_inverse * cartesian_velocities;

        // Convert Eigen vector to std::array
        std::array<double, 3> joint_velocities;
        for (int i = 0; i < 3; ++i) {
            joint_velocities[i] = joint_velocities_eigen(i);
        }

        // Integrate joint velocities to obtain joint angles (Euler method)
        std::array<double, 3> joint_angles;
        for (int i = 0; i < 3; ++i) {
            joint_angles[i] = current_joint_angles[i] + joint_velocities[i] * time_step_;
        }

        return joint_angles;
    }

private:
    // Dynamics parameters
    double hip_length_, thigh_length_, calf_length_;
    double leg_offset_x_, leg_offset_y_, trunk_offset_z_;
    double hip_max_, hip_min_;
    double thigh_max_, thigh_min_;
    double calf_max_, calf_min_;
    double trunk_ixx_, hip_ixx_, thigh_ixx_, calf_ixx_;

    // Constants for dynamics calculation (replace with actual values)
    double time_step_ = 0.01; // Example time step for integration

    // Calculate Jacobian matrix using numerical differentiation
    Eigen::MatrixXd computeJacobian(const std::array<double, 3>& joint_angles) {
        Eigen::MatrixXd jacobian(3, 3);
        double epsilon = 1e-6; // Small perturbation for numerical differentiation

        // Numerical differentiation to calculate Jacobian
        for (int i = 0; i < 3; ++i) {
            std::array<double, 3> perturbed_angles = joint_angles;
            perturbed_angles[i] += epsilon;

            // Calculate perturbed end-effector position
            std::array<double, 3> perturbed_position = forwardKinematics(perturbed_angles);

            // Numerical differentiation
            for (int j = 0; j < 3; ++j) {
                jacobian(j, i) = (perturbed_position[j] - forwardKinematics(joint_angles)[j]) / epsilon;
            }
        }

        return jacobian;
    }

    // Calculate pseudo-inverse of a matrix
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix) {
        // Use Singular Value Decomposition (SVD) to calculate pseudo-inverse
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Singular values
        Eigen::VectorXd singular_values = svd.singularValues();

        // Pseudo-inverse of singular values
        for (int i = 0; i < singular_values.size(); ++i) {
            if (std::abs(singular_values(i)) > 1e-6) {
                singular_values(i) = 1.0 / singular_values(i);
            }
            // If singular value is close to zero, set pseudo-inverse to zero
            else {
                singular_values(i) = 0.0;
            }
        }

        // Construct pseudo-inverse matrix
        Eigen::MatrixXd pseudo_inverse = svd.matrixV() * singular_values.asDiagonal() * svd.matrixU().transpose();

        return pseudo_inverse;
    }

    // Forward kinematics to calculate end-effector position
    std::array<double, 3> forwardKinematics(const std::array<double, 3>& joint_angles) {
        std::array<double, 3> position;

        double s1 = sin(joint_angles[0]); // hip joint
        double s2 = sin(joint_angles[1]); // thigh joint
        double s3 = sin(joint_angles[2]); // calf joint

        double c1 = cos(joint_angles[0]); // hip joint
        double c2 = cos(joint_angles[1]); // thigh joint
        double c3 = cos(joint_angles[2]); // calf joint

        double c23 = c2 * c3 - s2 * s3;
        double s23 = s2 * c3 + c2 * s3;

        position[0] = hip_length_ + thigh_length_ * c2 + calf_length_ * c23; // x
        position[1] = leg_offset_y_ + thigh_length_ * s2 + calf_length_ * s23; // y
        position[2] = trunk_offset_z_ + thigh_length_ * s1 + calf_length_ * s1; // z

        return position;
    }
};

int main() {
    // Example usage
    A1LegIKController ik_controller(0.04, 0.2, 0.2, 0.183, 0.047, 0.01675,
                                     46, -46, 240, -60, -52.5, -154.5,
                                     0.064713601, 0.000552929, 0.001367788, 0.000032426);

    // Set a target end-effector position
    std::array<double, 3> target_position = {0.1, 0.15, -0.25};

    // Current joint angles and Cartesian velocities (example values)
    std::array<double, 3> current_joint_angles = {0.1, 0.2, 0.3};
    std::array<double, 3> current_cartesian_velocities = {0.0, 0.0, 0.0};

    // Solve inverse kinematics
    std::array<double, 3> joint_angles = ik_controller.solveIK(target_position, current_joint_angles, current_cartesian_velocities);

    // Print the results
    std::cout << "Target Position: " << target_position[0] << " " << target_position[1] << " " << target_position[2] << std::endl;

    std::cout << "Joint angles: ";
    for (double angle : joint_angles) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    return 0;
}
