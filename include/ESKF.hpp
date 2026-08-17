#include <Eigen/Eigen>
#include <Eigen/Dense>

/*
Error-Satte Kalman Filter.
Здесь 13 мерный Error-state: ox = [op, ov, ob_a, ob_g, oλ] - 13 чисел
16 мерный Nominal-State: x = [p, v, b_a, b_g, λ] - 16 мерный, кватернион q из Махони

Производные с текужим состоянием тела
dp = v
dv = R(q) * (a_m - b_a) - g
db_a = 0
db_g = 0
λ = 0

Поскольку IMU имеет динамический bias, помимо статического, то a_m - b_a - это то, что ESKF пытается приблизить к реальности

*/

using namespace Eigen;

// Nominal state
struct nominal_state {
    Vector3d p = Vector3d::Zero();
    Vector3d v = Vector3d::Zero();
    Vector3d b_a = Vector3d::Zero();
    Vector3d b_g = Vector3d::Zero();
    double lambda = 1.0;   // масштаб VO — забыли добавить в прошлый раз, вот тут ему самое место

    MatrixXd P = MatrixXd::Identity(13, 13);  // ковариация ошибки, живёт вместе с nominal state
};

struct imu_sample {
    Vector3d a;
    Vector3d w;
    Matrix3d R;
    double dt;
};


class ESKF {
    public:
        ESKF(Vector3d oa2, Vector3d oba2, Vector3d og2, Vector3d obg2);
        void predict(const nominal_state& s, const imu_sample& imu_data, double dt);

    private:
        const Vector3d g = {0.0, 0.0, 9.81};
        nominal_state predict_state;

        Vector3d oa2_, oba2_, og2_, obg2_; // дисперсии для акселерометра м гироскопа и их bias`ов
};