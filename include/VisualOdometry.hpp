#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/multicalib.hpp>
#include <opencv2/features2d.hpp>

struct camera_info {
    int width;
    int height;
    cv::Mat k;
    cv::Mat d;
    cv::Mat r;
    cv::Mat p;
};


class VO {
    public:
        VO(camera_info info, int min_features, float parallax_thresh, float ratio_test, int orb_features);
        
        void image_callback(cv::Mat& image);
        
        void get_matches(const cv::Mat& current_image, std::vector<cv::Point2f>& q1, std::vector<cv::Point2f>& q2);
        
        cv::Mat get_pose(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2);
        
        cv::Mat form_transf(const cv::Mat& R, const cv::Mat& t);
        
        bool shouldCreateNewKeyframe(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2);
        
        void filter_by_mask(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2, const cv::Mat& mask,
                    std::vector<cv::Point2f>& q1_inliers, std::vector<cv::Point2f>& q2_inliers);
        
    private:
        cv::Ptr<cv::ORB> orb;
        cv::Ptr<cv::DescriptorMatcher> flann;
        std::vector<cv::KeyPoint> q1, q2;
        cv::Mat d1, d2;

        cv::Mat prev_img;
        camera_info cam_info;
        cv::Mat cur_pose;

        int MIN_FEATURES;
        float PARALLAX_THRESHOLD;
        float RATIO_TEST;

        void decomp_essential_mat(const cv::Mat& E, const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2, cv::Mat& out_R, cv::Mat& out_t);
        void sum_z_cal_relative_scale(const cv::Mat& R, const cv::Mat& t, 
                                const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2,
                                int& z_sum, double& relative_scale);
};