#include "include/Mahony.hpp"
#include "include/rapidcsv.h"

void Mahony::init_quat(double w, double x, double y, double z) {
    this->est_quat = {w, x, y, z};
    this->est_quat.normalize();
}

Eigen::Quaterniond Mahony::gyro_acc_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, int dt) {
    double acc_norm = acc.norm();
    double gx = gyr[0]; 
    double gy = gyr[1];
    double gz = gyr[2];
    Eigen::Matrix3d dcm = this->est_quat.toRotationMatrix();
    if (acc_norm > 0.0) {
        auto a = acc / acc_norm;
        Eigen::Vector3d v_g;
        if (this->nav_frame == "ENU") {
            v_g = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        else if (this->nav_frame == "NED") {
            v_g = Eigen::Vector3d(0.0, 0.0, -1.0);
        }
        Eigen::Matrix3d dcm = this->est_quat.toRotationMatrix();
        Eigen::Vector3d v_b = dcm.transpose() * v_g; 
        Eigen::Vector3d acc_error = a.cross(v_b);
        this->gyro_bias += this->ki * acc_error * dt;
        gyr += this->gyro_bias + this->kp * acc_error;
        gx = gyr[0];
        gy = gyr[1];
        gz = gyr[2];
    }
    Eigen::Quaterniond origin_q{0.0, gx, gy, gz};
    Eigen::Quaterniond quat_diff = est_quat * origin_q;
    
    Eigen::Quaterniond quat_change;
    quat_change.coeffs() = 0.5 * quat_diff.coeffs();
    
    this->est_quat.coeffs() += quat_change.coeffs() * dt;
    this->est_quat.normalize();
    
    return this->est_quat;
}

int main() {
    Mahony mahony(6, 1.0, 1.0, "ENU");
    rapidcsv::Document doc("../data.csv", rapidcsv::LabelParams(-1, -1));
    size_t rows = doc.GetRowCount();
    for (size_t i = 1; i < rows; ++i) {
        std::vector<long long> row = doc.GetRow<long long>(i);
        long long dt = row[i] - row[i-1];
        Eigen::Vector3d v(1.0, 0.0, 5.0);
        mahony.gyro_acc_fusion(v, v, 1);
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << "\n";
        Eigen::Vector3d acc = {row[1], row[2], row[3]};
        Eigen::Vector3d gyr = {row[4], row[5], row[6]};
        Eigen::Quaterniond filtered = mahony.gyro_acc_fusion(acc, gyr, dt);
    }

    return 0;
}