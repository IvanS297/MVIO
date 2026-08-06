#include "include/Mahony.hpp"
#include "include/rapidcsv.h"
#include "include/Kinematics.hpp"

void Mahony::init_quat(double w, double x, double y, double z) {
    this->est_quat = {w, x, y, z};
    this->est_quat.normalize();
}

Eigen::Quaterniond Mahony::gyro_acc_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, double dt) {
    /*
    DCM - матрица направляющих косинусов, это альтернатива кватернионам (Quaternion)
    */
    double acc_norm = acc.norm();
    Eigen::Matrix3d dcm = this->est_quat.toRotationMatrix(); 
    Eigen::Vector3d corrected_gyr = gyr;
    if (acc_norm > 1e-4) {
        /*
        Смысл числа 1e-4 заключается в том, что число может очен очен маленькое, но все равно не 0.0, это как на 0 делить, поэтому надо 
        защититься от этого.
        
        Деление вектора на его норму необъодимо для того, чтобы вектор имел длину от 0 до 1. Здесь нужно чистое направление, а не его длина,
        его длина может создать жусткие наводки, и смысл фильтра отпадет.

        v_g это вектор истинной гравитации земли, а v_b (vector body) - это вектор гравитации, который ожидает математика.  
        */
        auto a = acc / acc_norm;
        Eigen::Vector3d v_g;
        if (this->nav_frame == "ENU") {
            v_g = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        else if (this->nav_frame == "NED") {
            v_g = Eigen::Vector3d(0.0, 0.0, -1.0);
        }
        Eigen::Vector3d v_b = dcm.transpose() * v_g; 
        Eigen::Vector3d acc_error = a.cross(v_b);
        // векторное произведение покажет ошибку между реальным гравитационным вектором и вычесленным, и тогда bias для гироскопа будет поменян динамически.
        this->gyro_bias += this->ki * acc_error * dt;
        corrected_gyr += this->gyro_bias + this->kp * acc_error;
    }
    Eigen::Quaterniond omega(0.0, corrected_gyr.x(), corrected_gyr.y(), corrected_gyr.z());
    Eigen::Quaterniond quat_derivative;
    quat_derivative.coeffs() = 0.5 * (this->est_quat * omega).coeffs();
    
    this->est_quat.coeffs() += quat_derivative.coeffs() * dt;
    this->est_quat.normalize();
    
    return this->est_quat;
}

EulerAngles quaternionToDegrees(const Eigen::Quaterniond& q) {
    EulerAngles angles;    
    Eigen::Matrix3d m = q.toRotationMatrix();
    angles.pitch = std::asin(-m(2, 0));
    
    if (std::cos(angles.pitch) > 0.001) {
        angles.roll = std::atan2(m(2, 1), m(2, 2));
        angles.yaw = std::atan2(m(1, 0), m(0, 0));
    } else {
        angles.roll = 0.0;
        angles.yaw = std::atan2(-m(0, 1), m(1, 1));
    }

    double rad_to_deg = 180.0 / M_PI;
    angles.roll  *= rad_to_deg;
    angles.pitch *= rad_to_deg;
    angles.yaw   *= rad_to_deg;

    return angles;
}

Eigen::Quaterniond Mahony::gyro_acc_mag_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, Eigen::Vector3d& mag, double dt) {
    double acc_norm = acc.norm();
    double mag_norm = mag.norm();
    Eigen::Vector3d corrected_gyr = gyr;
    Eigen::Vector3d v_g, v_m, b, h;
    
    Eigen::Matrix3d dcm = this->est_quat.toRotationMatrix();
    if (acc_norm > 0.0 && mag_norm > 0.0) {
        Eigen::Vector3d a = acc / acc_norm;
        Eigen::Vector3d g = (this->nav_frame == "ENU") ? Eigen::Vector3d(0.0, 0.0, 1.0) : Eigen::Vector3d(0.0, 0.0, -1.0);
        
        v_g = dcm.transpose() * g;
        Eigen::Vector3d acc_error = a.cross(v_g);
        Eigen::Vector3d m = mag / mag_norm;
        h = dcm * m; 

        if (this->nav_frame == "ENU") {
            b = {0.0, std::sqrt(h.x() * h.x() + h.y() * h.y()), h.z()};
        }
        else if (this->nav_frame == "NED") {
            b = {std::sqrt(h.x() * h.x() + h.y() * h.y()), 0.0, h.z()};
        }

        v_m = dcm.transpose() * b;
        Eigen::Vector3d mag_error = m.cross(v_m);

        Eigen::Vector3d total_error = acc_error + mag_error;
        this->gyro_bias += this->ki * total_error * dt;
        corrected_gyr = gyr + this->gyro_bias + this->kp * total_error; 
    }

    Eigen::Quaterniond omega(0.0, corrected_gyr.x(), corrected_gyr.y(), corrected_gyr.z());
    Eigen::Quaterniond quat_derivative;
    quat_derivative.coeffs() = 0.5 * (this->est_quat * omega).coeffs();
    
    this->est_quat.coeffs() += quat_derivative.coeffs() * dt;
    this->est_quat.normalize();
    
    return this->est_quat;
}


int main() {
    Mahony mahony(9, 0.001, 0.5, "ENU");
    mahony.init_quat(1.0, 0.0, 0.0, 0.0);
    IMU_kinematic kin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    rapidcsv::Document doc("../data.csv", rapidcsv::LabelParams(-1, -1));
    size_t rows = doc.GetRowCount();
    double prev_time = 0.0;

    for (size_t i = 0; i < rows; ++i) {
        std::vector<std::string> row_str = doc.GetRow<std::string>(i);
        if (row_str.size() < 7 || row_str[0].empty()) continue;

        try {
            double current_time = std::stod(row_str[0]);
            double dt = 0.01;
            if (i > 0) {
                dt = (current_time - prev_time) / 1000000.0; 
            }
            prev_time = current_time;

            Eigen::Vector3d acc = { std::stod(row_str[1]), std::stod(row_str[2]), std::stod(row_str[3]) };
            Eigen::Vector3d gyr = { std::stod(row_str[4]), std::stod(row_str[5]), std::stod(row_str[6]) };
            Eigen::Vector3d mag = { std::stod(row_str[7]), std::stod(row_str[8]), std::stod(row_str[9]) };

            //gyr = gyr * (M_PI / 180.0); 
            Eigen::Quaterniond filtered = mahony.gyro_acc_mag_fusion(acc, gyr, mag, dt);
            //Eigen::Quaterniond filtered = mahony.gyro_acc_fusion(acc, gyr, dt);

            EulerAngles deg = quaternionToDegrees(filtered);

            std::cout << "Roll: " << deg.roll << " "
                      << "Pitch: " << deg.pitch << " "
                      << "Yaw: " << deg.yaw << std::endl;

            Eigen::Vector3d a = kin.get_a_lin(acc, filtered);
            Eigen::Vector3d xyz = kin.xyz(acc, filtered, dt);
            std::cout << "X " << xyz[0] << " Y " << xyz[1] << " Z " << xyz[2] << std::endl; 
        } catch (const std::exception& e) {
            std::cerr << "Error parsing " << i << ": " << e.what() << std::endl;
            continue;
        }
    }

    return 0;
}