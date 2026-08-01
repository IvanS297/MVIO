#include <string>
#include <Eigen/Dense>
#include <stdexcept>
#include <vector>
#include <iostream>

class Mahony {
    public:
    Mahony(int axis, int ki, int kp, std::string nav_frame) : 
        axis(axis), ki(ki), kp(kp), nav_frame(nav_frame) {
            if (nav_frame != "ENU" && nav_frame != "NED") {throw std::invalid_argument("nav frame must be only ENU or NED"); }
            if (axis != 6 && axis != 9) {throw std::invalid_argument("axis count must be only 6 or 9");}
        };

    void init_quat(double w, double x, double y, double z);
    Eigen::Quaterniond gyro_acc_fusion(Eigen::Vector3d& acc, Eigen::Vector3d& gyr, int dt);
    
    private:
    int axis = 6;
    int ki = 1.0;
    int kp = 1.0;
    std::string nav_frame = "ENU";

    Eigen::Quaterniond est_quat{1.0, 0.0, 0.0, 0.0};
    Eigen::MatrixXd gyro_bias = Eigen::MatrixXd::Zero(3, 1);

};