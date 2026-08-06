#include <Eigen/Dense>
#include <Eigen/Eigen>

class IMU_kinematic {
    public:
        IMU_kinematic(double roll, double pitch, double yaw, double x, double y, double z) {
            // https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen/21414609
            Eigen::AngleAxisd roll_a(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch_a(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw_a(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yaw_a * pitch_a * roll_a;
            R = q.matrix();
            T = {x, y, z};
        }

        Eigen::Vector3d get_a_lin(const Eigen::Vector3d& a_l, const Eigen::Quaterniond& q_l);
        Eigen::Vector3d xyz(const Eigen::Vector3d& a_lin, const Eigen::Quaterniond& q_l, double dt);

        Eigen::Matrix3d get_r_mat();
        Eigen::Vector3d get_t_vec();
        void reset();
    private:
        Eigen::Vector3d p_imu = {0.0, 0.0, 0.0};
        Eigen::Vector3d v_imu = {0.0, 0.0, 0.0};
        const Eigen::Vector3d g = {0.0, 0.0, 9.81};
        Eigen::Matrix3d R; // матрица поворота относительно base_link тела
        Eigen::Vector3d T; // вектор смещения относительно base_link тела
};

Eigen::Vector3d IMU_kinematic::get_a_lin(const Eigen::Vector3d& a_l, const Eigen::Quaterniond& q_l) {
    Eigen::Vector3d a_body = R * a_l;
    Eigen::Vector3d a_g = q_l * a_body;
    return a_g - g;
}

Eigen::Vector3d IMU_kinematic::xyz(const Eigen::Vector3d& a_lin, const Eigen::Quaterniond& q_l, double dt) {
    Eigen::Vector3d v_old = v_imu;
    v_imu += a_lin * dt;
    p_imu += (v_imu + v_old) * 0.5 * dt; 
    Eigen::Vector3d t_global = q_l * T;
    Eigen::Vector3d p_base_link = p_imu - t_global;
    return p_base_link;
}

void IMU_kinematic::reset() {
    p_imu.setZero();
    v_imu.setZero();
}

Eigen::Matrix3d IMU_kinematic::get_r_mat() {
    return R;
}
Eigen::Vector3d IMU_kinematic::get_t_vec() {
    return T;
}