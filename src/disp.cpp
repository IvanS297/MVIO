#include "include/rapidcsv.h"
#include <iostream>
#include <vector>

/*
Программа нужна, чтобы найти дисперсию машего IMU для рассчетов в ESKF
Нужен датасет data.csv 100+ значений, чтобы точнее ее найти, также сделайте так, чтобы тачик был в покое.
*/

double calc_disppersion(std::vector<double>& data) {
    double o2 = 0.0;
    double avg = 0.0;
    double sum = 0.0;
    for (int i = 0; i < data.size(); ++i) {
        avg += data[i];
    }
    avg /= data.size();
    for (int i = 0; i < data.size(); ++i) {
        sum = (data[i] - avg) * (data[i] - avg);
    }
    sum /= data.size();
    o2 = sqrt(sum);
    return o2;
}

double calc_bias_disp(double disp) {
    
}

int main() {
    rapidcsv::Document doc("../data.csv", rapidcsv::LabelParams(-1, -1));
    size_t rows = doc.GetRowCount();
    std::vector<double> accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz;
    for (size_t i = 0; i < rows; ++i) {
        std::vector<std::string> row_str = doc.GetRow<std::string>(i);
        if (row_str.size() < 7 || row_str[0].empty()) continue;
        accx.push_back(std::stod(row_str[1]));
        accy.push_back(std::stod(row_str[2]));
        accz.push_back(std::stod(row_str[3]));
        gyrx.push_back(std::stod(row_str[4]));
        gyry.push_back(std::stod(row_str[5]));
        gyrz.push_back(std::stod(row_str[6]));
        magx.push_back(std::stod(row_str[7]));
        magy.push_back(std::stod(row_str[8]));
        magz.push_back(std::stod(row_str[9]));
    }
    std::cout << "Dsipersions" << std::endl;
    std::cout << "Acc x: " << calc_disppersion(accx) << " "
    << "Acc y: " << calc_disppersion(accy) << " "
    << "Acc z: " << calc_disppersion(accz) << " "
    << "Gyr x: " << calc_disppersion(gyrx) << " "
    << "Gyr y: " << calc_disppersion(gyry) << " "
    << "Gyr z: " << calc_disppersion(gyrz) << " "
    << "Mag x: " << calc_disppersion(magx) << " "
    << "Mag y: " << calc_disppersion(magy) << " "
    << "Mag z: " << calc_disppersion(magz) << " " << std::endl;
    return 0;
}