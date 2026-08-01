#include <iostream>
#include "include/serialib.h"
#include <include/rapidcsv.h>
#include <sstream>

using namespace std;

int main() {
    serialib serial;
    char error = serial.openDevice("/dev/ttyUSB0", 115200);
    if (error != 1) return error;
    cout << "opened" << endl;

    std::stringstream ss;
    char buffer[73] = "";
    for (int i = 0; i < 100; ++i) {
        serial.readString(buffer, '\n', 72, 2000);
        cout << buffer << endl;
        ss << buffer;
        buffer[0] = '\0';
    }
    serial.closeDevice();
    cout << "closed" << endl;
    rapidcsv::Document doc(ss, rapidcsv::LabelParams(-1, -1));
    doc.Save("data.csv");
    return 0;
}