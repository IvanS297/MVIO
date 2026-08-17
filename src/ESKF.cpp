#include "include/ESKF.hpp"

// Мои дисперсии IMU (mpu6050)
// Acc x: 0.00086 Acc y: 0.00077 Acc z: 0.00375 Gyr x: 0.00016 Gyr y: 0.00099 Gyr z: 7e-05 Mag x: 0.00135 Mag y: 0.00675 Mag z: 0.00018 

ESKF::ESKF(Vector3d oa2, Vector3d oba2, Vector3d og2, Vector3d obg2) {
    oa2_ = oa2;
    oba2_ = oba2;
    og2_ = og2;
    obg2_ = obg2;
}

void ESKF::predict(const nominal_state& s, const imu_sample& d, double dt) {
    Vector3d a_corrected = d.R * (d.a - s.b_a) - g;

    // Пропагация nominal state
    predict_state.p = s.p + s.v * dt + 0.5 * a_corrected * dt * dt;
    predict_state.v = s.v + a_corrected * dt;
    predict_state.b_a = s.b_a;
    predict_state.b_g = s.b_g;
    // λ не входит в state как отдельное поле сейчас — стоит добавить, если ещё не добавлено
    predict_state.lambda = s.lambda; // масштаб всегда переносится между предсказанием и входом ESKF по той приине, что IMU + Камера всегда жестко закреплены между собой

    // Матрица F - якобиан динамики 
    MatrixXd F = MatrixXd::Identity(13, 13);
    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 6) = -d.R * dt;

    // Якобиан G по шуму с размером 13x10
    MatrixXd G = MatrixXd::Zero(13, 10);
    G.block<3, 3>(3, 0) = -d.R;
    G.block<3, 3>(6, 3) = Matrix3d::Identity();
    G.block<3, 3>(9, 6) = Matrix3d::Identity();
    G(12, 9) = 1.0;

    // Q_c и Q
    Matrix<double, 10, 1> dm;
    dm << oa2_, oba2_, obg2_, predict_state.lambda;
    MatrixXd Q_c = dm.asDiagonal();
    MatrixXd Q = G * Q_c * G.transpose() * dt;

    // Пропагация ковариации
    predict_state.P = F * s.P * F.transpose() + Q;
}