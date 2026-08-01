#include <iostream>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <numeric>

#include "include/VisualOdometry.hpp"

using namespace cv;
using namespace std;

VO::VO(camera_info info) {
    cam_info = info;

    cur_pose = cv::Mat::eye(4, 4, CV_64F);
    orb = cv::ORB::create(2000);

    cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
    cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>();
    flann = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
}

cv::Mat VO::form_transf(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    
    cv::Mat R_64;
    R.convertTo(R_64, CV_64F);
    R_64.copyTo(T(cv::Rect(0, 0, 3, 3)));
    
    cv::Mat t_64;
    t.convertTo(t_64, CV_64F);
    
    cv::Mat t_reshaped = t_64.reshape(1, 3);
    T.at<double>(0, 3) = t_reshaped.at<double>(0, 0);
    T.at<double>(1, 3) = t_reshaped.at<double>(1, 0);
    T.at<double>(2, 3) = t_reshaped.at<double>(2, 0);
    
    return T;
}

void VO::sum_z_cal_relative_scale(const cv::Mat& R, const cv::Mat& t, 
                                const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2,
                                int& z_sum, double& relative_scale) {
    
    if (q1.size() < 2 || q2.size() < 2) {
        z_sum = 0;
        relative_scale = 1.0;
        return;
    }

    cv::Mat T = form_transf(R, t);
    cv::Mat K_aug = cv::Mat::zeros(3, 4, CV_64F);
    cam_info.k.convertTo(K_aug(cv::Rect(0, 0, 3, 3)), CV_64F);

    cv::Mat P = K_aug * T;

    cv::Mat hom_Q1_raw;
    cv::triangulatePoints(cam_info.p, P, q1, q2, hom_Q1_raw);

    if (hom_Q1_raw.empty() || hom_Q1_raw.cols < 2) {
        z_sum = 0;
        relative_scale = 1.0;
        return;
    }

    cv::Mat hom_Q1;
    hom_Q1_raw.convertTo(hom_Q1, CV_64F);

    cv::Mat hom_Q2 = T * hom_Q1;

    cv::Mat uhom_Q1 = cv::Mat(3, hom_Q1.cols, CV_64F);
    cv::Mat uhom_Q2 = cv::Mat(3, hom_Q2.cols, CV_64F);

    int pos_z_count = 0;
    for (int i = 0; i < hom_Q1.cols; ++i) {
        // Теперь здесь СТРОГО double и .at<double>
        double w1 = hom_Q1.at<double>(3, i);
        double w2 = hom_Q2.at<double>(3, i);
        
        // Защита от деления на ноль, если w близко к нулю
        if (std::abs(w1) < 1e-6) w1 = 1e-6;
        if (std::abs(w2) < 1e-6) w2 = 1e-6;

        double z1 = hom_Q1.at<double>(2, i) / w1;
        double z2 = hom_Q2.at<double>(2, i) / w2;

        uhom_Q1.at<double>(0, i) = hom_Q1.at<double>(0, i) / w1;
        uhom_Q1.at<double>(1, i) = hom_Q1.at<double>(1, i) / w1;
        uhom_Q1.at<double>(2, i) = z1;

        uhom_Q2.at<double>(0, i) = hom_Q2.at<double>(0, i) / w2;
        uhom_Q2.at<double>(1, i) = hom_Q2.at<double>(1, i) / w2;
        uhom_Q2.at<double>(2, i) = z2;

        if (z1 > 0) pos_z_count++;
        if (z2 > 0) pos_z_count++;
    }
    z_sum = pos_z_count;
 
    double scale_sum = 0.0;
    int valid_pairs = 0;
    for (int i = 0; i < hom_Q1.cols - 1; ++i) {
        double norm1 = cv::norm(uhom_Q1.col(i) - uhom_Q1.col(i + 1));
        double norm2 = cv::norm(uhom_Q2.col(i) - uhom_Q2.col(i + 1));
        if (norm2 > 1e-6) {
            scale_sum += (norm1 / norm2);
            valid_pairs++;
        }
    }
    relative_scale = (valid_pairs > 0) ? (scale_sum / valid_pairs) : 1.0;
}

void VO::decomp_essential_mat(const cv::Mat& E, const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2,
                            cv::Mat& out_R, cv::Mat& out_t) {
    cv::Mat R1, R2, t;
    cv::decomposeEssentialMat(E, R1, R2, t);

    std::vector<std::pair<cv::Mat, cv::Mat>> pairs = {
        {R1, t}, {R1, -t}, {R2, t}, {R2, -t}
    };

    int max_z_sum = -1;
    int best_idx = 0;
    double best_scale = 1.0;

    for (size_t i = 0; i < pairs.size(); ++i) {
        int z_sum = 0;
        double scale = 1.0;
        sum_z_cal_relative_scale(pairs[i].first, pairs[i].second, q1, q2, z_sum, scale);

        if (z_sum > max_z_sum) {
            max_z_sum = z_sum;
            best_idx = i;
            best_scale = scale;
        }
    }

    out_R = pairs[best_idx].first.clone();
    out_t = pairs[best_idx].second * best_scale; // Применяем вычисленный масштаб к вектору сдвига
}

cv::Mat VO::get_pose(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2) {
    if (q1.size() < 8 || q2.size() < 8) return cv::Mat();

    cv::Mat E = cv::findEssentialMat(q1, q2, cam_info.k, cv::RANSAC, 0.999, 1.0);
    if (E.empty() || E.rows != 3 || E.cols != 3) return cv::Mat();

    cv::Mat R, t;
    decomp_essential_mat(E, q1, q2, R, t);

    return form_transf(R, t);
}

void VO::image_callback(Mat &image)
{
    Mat dst, gray;

    cv::Mat K_fixed, D_fixed;
    cam_info.k.convertTo(K_fixed, CV_64F);
    cam_info.d.convertTo(D_fixed, CV_64F); 

    try {
        cv::undistort(image, dst, cam_info.k, cam_info.d);
    } catch (const cv::Exception& e) {
        std::cerr << "unsidtorrt error: " << e.what() << std::endl;
        return;
    }
    
    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    image = gray;
    if (!prev_img.empty())
    {
        try
        {
            std::vector<cv::Point2f> q1, q2;
            get_matches(image, q1, q2);
            if (q1.size() >= 8 && q2.size() >= 8)
            {
                std::cout << "[INFO] Found " << q1.size() << " matches" << std::endl;
                cv::Mat transf;
                try {
                    transf = get_pose(q1, q2);
                } catch (const cv::Exception& e) {
                    std::cerr << "get pose error: " << e.what() << std::endl;
                    return;
                }
                if (!transf.empty())
                {
                    cv::Mat transf_inv = transf.inv();
                    try {
                        cur_pose = cur_pose * transf_inv;
                        double x = cur_pose.at<double>(0, 3);
                        double y = cur_pose.at<double>(1, 3);
                        double z = cur_pose.at<double>(2, 3);
                        std::cout << "Robot Position -> X: " << x << " | Y: " << y << " | Z: " << z << std::endl;                    
                    } catch (const cv::Exception& e) {
                        std::cerr << "cur_pose error: " << e.what() << std::endl;
                        return;
                    }
                }
                
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Общая ошибка: " << e.what() << '\n';
        }
    }
    prev_img = image;
}

void VO::get_matches(const cv::Mat& current_image, std::vector<cv::Point2f>& q1, std::vector<cv::Point2f>& q2) {
    q1.clear();
    q2.clear();

    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    orb->detectAndCompute(prev_img, cv::noArray(), kp1, des1);
    orb->detectAndCompute(current_image, cv::noArray(), kp2, des2);

    if (des1.empty() || des2.empty()) return;

    std::vector<std::vector<cv::DMatch>> matches;
    flann->knnMatch(des1, des2, matches, 2);

    std::vector<cv::DMatch> good;
    for (const auto& match_pair : matches) {
        if (match_pair.size() >= 2) {
            if (match_pair[0].distance < 0.95f * match_pair[1].distance) {
                good.push_back(match_pair[0]);
            }
        }
    }

    cv::Mat img3;
    cv::drawMatches(current_image, kp1, prev_img, kp2, good, img3, 
                    cv::Scalar::all(-1), cv::Scalar::all(-1), 
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    cv::imshow("image", img3);
    cv::waitKey(1);

    for (const auto& m : good) {
        q1.push_back(kp1[m.queryIdx].pt);
        q2.push_back(kp2[m.trainIdx].pt);
    }
}

cv::Mat k = (cv::Mat_<double>(3,3) << 810.88756426, 0.0, 291.91410185,
         0.0, 810.24169473, 171.78701343,
         0.0, 0.0, 1.0);
cv::Mat d = (cv::Mat_<double>(1,5) << 0.30756359, -4.66079417, -0.01652856, -0.01069219, 10.05040356);
cv::Mat p = (cv::Mat_<double>(3,4) << -4.42494188e+01, -8.39080550e+02,  1.91672388e+02,  2.39839033e+05,
         5.30478544e+02,  5.69766726e+01,  6.33520761e+02,  4.11870112e+04,
         -6.01533592e-01, -1.36372086e-01,  7.87121332e-01,  6.17549839e+02);

cv::Mat r{1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};
camera_info cam_info = {640, 360, k, d, r, p};
VO vo(cam_info);

int main()
{
    VideoCapture cap("http://192.168.137.39:5000");
    auto now = chrono::system_clock::now();
    auto prev = now;

    if (!cap.isOpened())
    {
        cout << "Error: Cannot open camera" << endl;
        return -1;
    }

    Mat frame;
    while (true)
    {
        if (now == prev)
        {
            now = chrono::system_clock::now();
            continue;
        }
        now = chrono::system_clock::now();
        cap >> frame;
        
        if (frame.empty())
        {
            cout << "Can't receive frame" << endl;
            break;
        }

        imshow("Camera", frame);
        vo.image_callback(frame);

        // float fps = 1000.0f / chrono::duration_cast<chrono::milliseconds>(now - prev).count();
        // cv::putText(frame, "FPS: " + to_string(fps), cv::Point(50, 350), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

        if (waitKey(1) == 27)
            break;
        prev = now;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
