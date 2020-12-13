#include <iostream>
using namespace std;

#include <ctime>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
// Eigenatrix Exponential
#include <unsupported/Eigen/MatrixFunctions>
using namespace Eigen;

int main() {
    // Initializae an euler_angle vector
    // Roll-Yaw-Pitch: 0 -> -pi/2 -> -pi/4
    Eigen::Vector3d euler_angle(-M_PI/ 4.0, -M_PI/ 2.0, 0.0);
    cout << "euler = " << euler_angle.transpose() << endl;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix =\n" << R << endl;

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
    cout << "quaternion = " << q.coeffs().transpose() << endl;

    // w
    Eigen::Vector3d w(0.01,0.02,0.03);
    cout << "w = " << w.transpose() << endl;
    // w^
    Eigen::Matrix3d w_hat;
    w_hat << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    cout << "w^= \n" << w_hat << endl;

    // R <- Rexp(w^)
    R = R*w_hat.exp();
    cout << "R = \n" << R;

    return 0;
}
