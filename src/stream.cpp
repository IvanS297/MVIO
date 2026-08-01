#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        cout << "Error: Cannot open camera" << endl;
        return -1;
    }
    
    Mat frame;
    while (true) {
        cap >> frame;
        
        if (frame.empty()) {
            cout << "Can't receive frame" << endl;
            break;
        }
        
        imshow("Camera", frame);
        
        if (waitKey(1) == 27)
            break;
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}