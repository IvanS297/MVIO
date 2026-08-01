#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

int main() {
    // Проверяем версию OpenCV
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

    // Попытка открыть стандартную вебкамеру
    cv::VideoCapture cap(0); 
    if (!cap.isOpened()) {
        std::cout << "Не удалось открыть камеру (это нормально, если её нет)." << std::endl;
        return 0;
    }

    std::cout << "Камера успешно инициализирована!" << std::endl;
    return 0;
}
