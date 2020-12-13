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

// Declaration
Quaterniond quat_mul(Quaterniond q1,Quaterniond q2);

int main() {
    // Initializae an euler_angle vector
    // euler_angle: Roll-Pitch-Yaw
    //              0 -> -pi/2 -> -pi/4
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
    cout << "new R = \n" << R << endl;

    // q <- q x [1,w].T
    Eigen::Quaterniond w_mul(1,w(0)/2, w(1)/2, w(2)/2);
    q = quat_mul(q, w_mul);
    cout << "new q = " << q.coeffs().transpose() << endl;
    cout << "new R from q = \n" << q.normalized().toRotationMatrix() << endl;

    return 0;
}


Quaterniond quat_mul(Quaterniond q1,Quaterniond q2) {
    double x =  q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y() + q1.w() * q2.x();
    double y = -q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x() + q1.w() * q2.y();
    double z =  q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w() + q1.w() * q2.z();
    double w = -q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z() + q1.w() * q2.w();

    return Quaterniond(w,x,y,z);
}