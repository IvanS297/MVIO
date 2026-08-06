#include <string>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <stdexcept>
#include <vector>
#include <iostream>

struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
};

class Mahony {
    public:
    Mahony(int axis, double ki, double kp, std::string nav_frame) : 
        axis(axis), ki(ki), kp(kp), nav_frame(nav_frame) {
            if (nav_frame != "ENU" && nav_frame != "NED") {throw std::invalid_argument("nav frame must be only ENU or NED"); }
            if (axis != 6 && axis != 9) {throw std::invalid_argument("axis count must be only 6 or 9");}
        };

    void init_quat(double w, double x, double y, double z);
    Eigen::Quaterniond gyro_acc_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, double dt);
    Eigen::Quaterniond gyro_acc_mag_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, Eigen::Vector3d& mag, double dt);
    
    private:
    int axis;
    int ki;
    int kp;
    std::string nav_frame = "ENU";

    Eigen::Quaterniond est_quat{1.0, 0.0, 0.0, 0.0};
    Eigen::MatrixXd gyro_bias = Eigen::MatrixXd::Zero(3, 1);

};

/*
https://medium.com/@k66115704/imu-mahony-filter-explanation-1ae75bf033ab
https://www.olliw.eu/2013/imu-data-fusing/
https://nitinjsanket.github.io/tutorials/attitudeest/mahony.html
https://pev.dev/t/refloat-a-new-vesc-package/1505
*/