#include <iostream>
#include "include/serialib.h"
#include <include/rapidcsv.h>
#include <sstream>

using namespace std;

int main() {
    serialib serial;
    char error = serial.openDevice("/dev/ttyUSB0", 115200);
    if (error != 1) {
        cout << "Error opening device: " << (int)error << endl;
        return error;
    }
    cout << "opened" << endl;

    std::stringstream ss;
    char buffer[83] = "";
    
    for (int i = 0; i < 250; ++i) {
        int bytes_read = serial.readString(buffer, '\n', 83, 2000);
            
        if (i > 0) {
            if (bytes_read > 0) {
                std::string line(buffer);
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();  // убираем хвостовой \r, если он есть
                }
                cout << line << endl;
                if (i < 2) continue;    
                ss << line << "\n"; 
            }
        }
        
        buffer[0] = '\0';
    }
    
    serial.closeDevice();
    cout << "closed" << endl;

    rapidcsv::Document doc(ss, rapidcsv::LabelParams(-1, -1));
    doc.Save("data.csv");
    
    cout << "Dataset saved to data.csv" << endl;
    return 0;
}
