/*
 * Copyright (C) 2024 - Rewritten for new camera/param format
 * Based on auto_calib_fisheye/test_bev_simple.cpp
 */
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <json/json.h>
#include <fstream>

using namespace std;
using namespace cv;

// Camera configuration structure (similar to auto_calib_fisheye)
struct CameraConfig {
    string name;
    string image_file;
    string ocam_param_file;
    string json_name;  // e.g., "park_front"
    
    // Ocam parameters
    struct OcamParams {
        double cx, cy;
        double c, d, e;
        vector<double> cam2world_poly;
        vector<double> world2cam_poly;
        Eigen::VectorXd world2cam_coeffs;  // For Horner's method
        int width, height;
    } ocam;
    
    // Traditional pinhole intrinsics (for pinhole model)
    Eigen::Matrix3d K;
    vector<double> dist;
    
    // Extrinsics (T_vehicle_to_camera = body-to-camera)
    Eigen::Matrix4d extrinsic;
    Eigen::Matrix4d extrinsic_initial;
    
    // Height (camera height above ground)
    double height;
    
    // Images
    Mat img;
    Mat img_bev;
    
    // Tail size for blending
    int tail_size;
};

// Global camera configs
CameraConfig cameras[4];  // 0=front, 1=left, 2=back, 3=right
const string camera_names[4] = {"front", "left", "behind", "right"};
const string json_names[4] = {"park_front", "park_left", "park_back", "park_right"};

// Calibration state
bool cali_stiching_mode = false;
double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
int cali_frame = 0;
int camera_model = 1;  // 0=fisheye, 1=ocam, 2=pinhole

// BEV parameters
int bev_rows = 1000, bev_cols = 1000;
double metric_ratio_m = 0.01;  // meters per pixel (1cm/pixel)
double wheel_base = 3.01;      // Default wheel base in meters
Eigen::Matrix3d KG;

// Modification matrices for calibration
vector<Eigen::Matrix4d> modification_list_[4];

// Keyboard detection
bool kbhit() {
    termios term;
    tcgetattr(0, &term);
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    tcsetattr(0, TCSANOW, &term);
    return byteswaiting > 0;
}

// Convert Rodrigues vector to rotation matrix
Eigen::Matrix3d rodriguesToRotationMatrix(const vector<double> &rvec) {
    double theta = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    
    if (theta < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }
    
    double r[3] = {rvec[0] / theta, rvec[1] / theta, rvec[2] / theta};
    
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double one_minus_cos = 1.0 - cos_theta;
    
    Eigen::Matrix3d R;
    R << cos_theta + one_minus_cos * r[0] * r[0],
         one_minus_cos * r[0] * r[1] - sin_theta * r[2],
         one_minus_cos * r[0] * r[2] + sin_theta * r[1],
         one_minus_cos * r[1] * r[0] + sin_theta * r[2],
         cos_theta + one_minus_cos * r[1] * r[1],
         one_minus_cos * r[1] * r[2] - sin_theta * r[0],
         one_minus_cos * r[2] * r[0] - sin_theta * r[1],
         one_minus_cos * r[2] * r[1] + sin_theta * r[0],
         cos_theta + one_minus_cos * r[2] * r[2];
    
    return R;
}

// Load Ocam parameters from JSON
void loadOcamParams(CameraConfig &cam, const string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open Ocam calibration file: " << filename << endl;
        exit(1);
    }
    
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(file, root)) {
        cerr << "Failed to parse JSON from " << filename << endl;
        exit(1);
    }
    
    const Json::Value &intrinsic = root["intrinsic_param"];
    
    cam.ocam.width = intrinsic["camera_width"].asInt();
    cam.ocam.height = intrinsic["camera_height"].asInt();
    cam.ocam.cx = intrinsic["principal_point"][0].asDouble();
    cam.ocam.cy = intrinsic["principal_point"][1].asDouble();
    cam.ocam.c = intrinsic["affine_c"].asDouble();
    cam.ocam.d = intrinsic["affine_d"].asDouble();
    cam.ocam.e = intrinsic["affine_e"].asDouble();
    
    const Json::Value &cam2world = intrinsic["cam2world"];
    const Json::Value &world2cam = intrinsic["world2cam"];
    
    cam.ocam.cam2world_poly.clear();
    cam.ocam.world2cam_poly.clear();
    
    for (int i = 0; i < (int)cam2world.size(); i++) {
        cam.ocam.cam2world_poly.push_back(cam2world[i].asDouble());
    }
    for (int i = 0; i < (int)world2cam.size(); i++) {
        cam.ocam.world2cam_poly.push_back(world2cam[i].asDouble());
    }
    
    // Pre-compute polynomial coefficients for Horner's method
    cam.ocam.world2cam_coeffs.resize(cam.ocam.world2cam_poly.size());
    for (size_t i = 0; i < cam.ocam.world2cam_poly.size(); i++) {
        cam.ocam.world2cam_coeffs(i) = cam.ocam.world2cam_poly[cam.ocam.world2cam_poly.size() - 1 - i];
    }
    
    cout << "Loaded Ocam params for " << cam.name << ": " 
         << cam.ocam.width << "x" << cam.ocam.height << endl;
}

// Load extrinsics from new JSON format
void loadExtrinsics(const string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open extrinsics file: " << filename << endl;
        exit(1);
    }
    
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(file, root)) {
        cerr << "Failed to parse extrinsics JSON from " << filename << endl;
        exit(1);
    }
    
    const Json::Value &extrinsics = root["extrinsic_param"];
    
    for (int i = 0; i < 4; i++) {
        const string &json_name = json_names[i];
        
        if (!extrinsics.isMember(json_name)) {
            cerr << "Missing extrinsics for " << json_name << endl;
            exit(1);
        }
        
        const Json::Value &cam_ext = extrinsics[json_name];
        
        // Get rotation vector and translation
        vector<double> rvec(3);
        for (int j = 0; j < 3; j++) {
            rvec[j] = cam_ext["rotation"][j].asDouble();
        }
        
        Eigen::Vector3d translation_vec;
        for (int j = 0; j < 3; j++) {
            translation_vec[j] = cam_ext["translation"][j].asDouble();
        }
        
        // Convert Rodrigues vector to rotation matrix (camera-to-body rotation)
        Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
        
        // Build T_cam_to_body (4x4)
        Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
        T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
        T_cam_to_body.block<3, 1>(0, 3) = translation_vec;
        
        // Take inverse to get T_body_to_cam (vehicle-to-camera)
        cameras[i].extrinsic = T_cam_to_body.inverse();
        cameras[i].extrinsic_initial = cameras[i].extrinsic;
        cameras[i].height = fabs(translation_vec[2]);
        
        cout << "Loaded extrinsics for " << cameras[i].name << ":" << endl;
        cout << "  Translation: [" << cameras[i].extrinsic.block<3,1>(0,3).transpose() << "]" << endl;
        cout << "  Height: " << cameras[i].height << " m" << endl;
    }
}

// Load pinhole intrinsics from old format (for compatibility)
void loadPinholeIntrinsic(const string &filename, Eigen::Matrix3d &K, vector<double> &dist) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open intrinsic file: " << filename << endl;
        return;
    }
    
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(file, root)) {
        cerr << "Failed to parse intrinsic JSON from " << filename << endl;
        return;
    }
    
    Json::Value::Members name = root.getMemberNames();
    string id = *(name.begin());
    
    Json::Value intri = root[id]["param"]["cam_K"]["data"];
    Json::Value d = root[id]["param"]["cam_dist"]["data"];
    int dist_col = root[id]["param"]["cam_dist"]["cols"].asInt();
    
    K << intri[0][0].asDouble(), intri[0][1].asDouble(), intri[0][2].asDouble(),
         intri[1][0].asDouble(), intri[1][1].asDouble(), intri[1][2].asDouble(),
         intri[2][0].asDouble(), intri[2][1].asDouble(), intri[2][2].asDouble();
    
    dist.clear();
    for (int i = 0; i < dist_col; i++) {
        dist.push_back(d[0][i].asDouble());
    }
}

// Ocam projection: distortPointsOcamFull (from auto_calib_fisheye)
void distortPointsOcamFull(Mat &P_GC1, Mat &p_GC, const CameraConfig::OcamParams &ocam) {
    for (int i = 0; i < P_GC1.cols; i++) {
        // P_GC1 contains 3D camera coordinates [x, y, z]
        double x = P_GC1.at<Vec3d>(0, i)[0];
        double y = P_GC1.at<Vec3d>(0, i)[1];
        double z = P_GC1.at<Vec3d>(0, i)[2];
        
        // Check if point is visible by camera
        if (z < 0) {
            p_GC.at<Vec2d>(0, i)[0] = -1;
            p_GC.at<Vec2d>(0, i)[1] = -1;
            continue;
        }
        
        // CPAC formula: norm = sqrt(x*x + y*y)
        double norm = sqrt(x * x + y * y);
        if (norm < 1e-14) norm = 1e-14;
        
        // CPAC: theta = atan(-z / norm)
        double theta = atan(-z / norm);
        
        // Evaluate polynomial using Horner's method
        double rho = 0.0;
        for (int k = 0; k < ocam.world2cam_coeffs.size(); k++) {
            rho = rho * theta + ocam.world2cam_coeffs(k);
        }
        
        // CPAC: uu = x / norm * rho, vv = y / norm * rho
        double uu = x / norm * rho;
        double vv = y / norm * rho;
        
        // CPAC affine transformation
        double u = uu + vv * ocam.e + ocam.cx;
        double v = uu * ocam.d + vv * ocam.c + ocam.cy;
        
        p_GC.at<Vec2d>(0, i)[0] = u;
        p_GC.at<Vec2d>(0, i)[1] = v;
    }
}

// Helper: Convert Eigen matrix to OpenCV Mat
void eigenToCv(const Eigen::Matrix3d& eigen_mat, cv::Mat& cv_mat) {
    cv_mat = cv::Mat(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cv_mat.at<double>(i, j) = eigen_mat(i, j);
        }
    }
}

// Project image to ground (BEV) - new CPAC-based approach
Mat project_on_ground(Mat img, const CameraConfig &cam, const string &camera_idx) {
    // Build 4xN matrix of ground points in body/vehicle coordinates
    Mat P_G = Mat::ones(4, bev_rows * bev_cols, CV_64FC1);
    
    for (int j = 0; j < bev_rows; j++) {
        for (int i = 0; i < bev_cols; i++) {
            int idx = j * bev_cols + i;
            
            double wheel_base_pixel = wheel_base / metric_ratio_m;
            double veh_x = (wheel_base_pixel / 2.0 + bev_rows / 2.0 - j) * metric_ratio_m;
            double veh_y = (bev_cols / 2.0 - i) * metric_ratio_m;
            
            P_G.at<double>(0, idx) = veh_x;  // x_forward
            P_G.at<double>(1, idx) = veh_y;  // y_left
            P_G.at<double>(2, idx) = 0.0;    // z_up (ground)
        }
    }
    
    // Transform from body to camera coordinates
    Eigen::MatrixXd P_G_eigen(4, bev_rows * bev_cols);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < bev_rows * bev_cols; j++) {
            P_G_eigen(i, j) = P_G.at<double>(i, j);
        }
    }
    
    Eigen::MatrixXd P_GC_eigen = cam.extrinsic * P_G_eigen;
    
    // Convert to 3D points for projection
    Mat P_GC1 = Mat::zeros(1, bev_rows * bev_cols, CV_64FC3);
    for (int i = 0; i < bev_rows * bev_cols; i++) {
        P_GC1.at<Vec3d>(0, i) = Vec3d(P_GC_eigen(0, i), P_GC_eigen(1, i), P_GC_eigen(2, i));
    }
    
    Mat p_GC = Mat::zeros(1, bev_rows * bev_cols, CV_64FC2);
    
    // Project using appropriate camera model
    if (camera_model == 0) {
        // OpenCV fisheye model
        Mat K_mat;
        eigenToCv(cam.K, K_mat);
        fisheye::distortPoints(P_GC1, p_GC, K_mat, cam.dist);
    } else if (camera_model == 1) {
        // Ocam model
        distortPointsOcamFull(P_GC1, p_GC, cam.ocam);
    } else {
        // Pinhole model (simple projection)
        for (int i = 0; i < P_GC1.cols; i++) {
            double x = P_GC1.at<Vec3d>(0, i)[0];
            double y = P_GC1.at<Vec3d>(0, i)[1];
            double z = P_GC1.at<Vec3d>(0, i)[2];
            
            if (z <= 0) {
                p_GC.at<Vec2d>(0, i)[0] = -1;
                p_GC.at<Vec2d>(0, i)[1] = -1;
            } else {
                double u = x / z * cam.K(0, 0) + cam.K(0, 2);
                double v = y / z * cam.K(1, 1) + cam.K(1, 2);
                p_GC.at<Vec2d>(0, i)[0] = u;
                p_GC.at<Vec2d>(0, i)[1] = v;
            }
        }
    }
    
    // Reshape and remap
    Mat p_GC_table = p_GC.reshape(0, bev_rows);
    Mat p_GC_table_32F;
    p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);
    
    Mat img_GC;
    remap(img, img_GC, p_GC_table_32F, Mat(), INTER_LINEAR);
    
    return img_GC;
}

// Tail function for blending
Mat tail(Mat img, const string &index) {
    int size = 600;  // Default tail size

    if (index == "f" || index == "front") {
        Rect m_select = Rect(0, 0, img.cols, size);
        Mat cropped = img(m_select);
        Mat border(img.rows - size, img.cols, cropped.type(), Scalar(0, 0, 0));
        Mat dst;
        vconcat(cropped, border, dst);
        return dst;
    } else if (index == "l" || index == "left") {
        Rect m_select = Rect(0, 0, size, img.rows);
        Mat cropped = img(m_select);
        Mat border(img.rows, img.cols - size, cropped.type(), Scalar(0, 0, 0));
        Mat dst;
        hconcat(cropped, border, dst);
        return dst;
    } else if (index == "b" || index == "behind" || index == "back") {
        Rect m_select = Rect(0, img.rows - size, img.cols, size);
        Mat cropped = img(m_select);
        Mat border(img.rows - size, img.cols, cropped.type(), Scalar(0, 0, 0));
        Mat dst;
        vconcat(border, cropped, dst);
        return dst;
    } else if (index == "r" || index == "right") {
        Rect m_select = Rect(img.cols - size, 0, size, img.rows);
        Mat cropped = img(m_select);
        Mat border(img.rows, img.cols - size, cropped.type(), Scalar(0, 0, 0));
        Mat dst;
        hconcat(border, cropped, dst);
        return dst;
    }
    return img.clone();
}

// Generate surround view from 4 BEV images
/**
 * @brief 生成全景环视（Surround View）图像
 * 
 * 将四个方向的BEV图像（前、后、左、右）拼接成一张全景图像，
 * 在重叠区域进行图像融合，并在中心区域创建黑色遮罩。
 * 
 * @param img_GF 前方（Front）BEV投影图像
 * @param img_GL 左侧（Left）BEV投影图像
 * @param img_GB 后方（Back）BEV投影图像
 * @param img_GR 右侧（Right）BEV投影图像
 * 
 * @return 拼接后的全景环视图像
 * 
 * @note 融合策略：
 *       - 四角区域：相邻方向图像按0.5权重混合
 *       - 边缘区域：单一方向图像
 *       - 中心区域：黑色遮罩（blend_start到blend_end范围）
 * @note blend_start和blend_end参数定义了中心黑色区域的大小
 */
Mat generate_surround_view(Mat img_GF, Mat img_GL, Mat img_GB, Mat img_GR) {
    Mat img_G(bev_rows, bev_cols, CV_8UC3);
    
    int blend_start = 400;
    int blend_end = 600;
    
    for (int i = 0; i < blend_start; i++) {
        for (int j = 0; j < blend_start; j++) {
            img_G.at<Vec3b>(i, j) = 0.5 * img_GL.at<Vec3b>(i, j) + 0.5 * img_GF.at<Vec3b>(i, j);
        }
        for (int j = blend_end; j < bev_cols; j++) {
            img_G.at<Vec3b>(i, j) = 0.5 * img_GR.at<Vec3b>(i, j) + 0.5 * img_GF.at<Vec3b>(i, j);
        }
    }
    for (int i = blend_end; i < bev_rows; i++) {
        for (int j = 0; j < blend_start; j++) {
            img_G.at<Vec3b>(i, j) = 0.5 * img_GL.at<Vec3b>(i, j) + 0.5 * img_GB.at<Vec3b>(i, j);
        }
        for (int j = blend_end; j < bev_cols; j++) {
            img_G.at<Vec3b>(i, j) = 0.5 * img_GR.at<Vec3b>(i, j) + 0.5 * img_GB.at<Vec3b>(i, j);
        }
    }
    for (int i = blend_start; i < blend_end; i++) {
        for (int j = 0; j < blend_start; j++) {
            img_G.at<Vec3b>(i, j) = img_GL.at<Vec3b>(i, j);
        }
        for (int j = blend_end; j < bev_cols; j++) {
            img_G.at<Vec3b>(i, j) = img_GR.at<Vec3b>(i, j);
        }
    }
    for (int j = blend_start; j < blend_end; j++) {
        for (int i = 0; i < blend_start; i++) {
            img_G.at<Vec3b>(i, j) = img_GF.at<Vec3b>(i, j);
        }
        for (int i = blend_end; i < bev_rows; i++) {
            img_G.at<Vec3b>(i, j) = img_GB.at<Vec3b>(i, j);
        }
    }
    
    // Center black region
    for (int i = blend_start; i < blend_end; i++) {
        for (int j = blend_start; j < blend_end; j++) {
            img_G.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
        }
    }
    
    return img_G;
}

// Initialize calibration modification matrices
void CalibrationInit() {
    for (int cam_idx = 0; cam_idx < 4; cam_idx++) {
        modification_list_[cam_idx].reserve(12);
        for (int32_t j = 0; j < 12; j++) {
            std::vector<int> transform_flag(6, 0);
            transform_flag[j / 2] = (j % 2) ? (-1) : 1;
            
            Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rot_tmp;
            rot_tmp = Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                                        Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                                        Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                                        Eigen::Vector3d::UnitZ());
            tmp.block(0, 0, 3, 3) = rot_tmp;
            tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
            tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
            tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
            modification_list_[cam_idx].push_back(tmp);
        }
    }
    cout << "=>Calibration scale Init!" << endl;
}

// Update calibration scale
void CalibrationScaleChange(int frame) {
    modification_list_[frame].clear();
    for (int32_t i = 0; i < 12; i++) {
        std::vector<int> transform_flag(6, 0);
        transform_flag[i / 2] = (i % 2) ? (-1) : 1;
        
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rot_tmp;
        rot_tmp = Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                                    Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                                    Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                                    Eigen::Vector3d::UnitZ());
        tmp.block(0, 0, 3, 3) = rot_tmp;
        tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
        tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
        tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
        modification_list_[frame].push_back(tmp);
    }
    cout << "=>Calibration scale update done!" << endl;
}

// Convert rotation matrix to Rodrigues vector
void rotationMatrixToRodrigues(const Eigen::Matrix3d &R, Eigen::Vector3d &rvec) {
    double theta = acos((R.trace() - 1.0) / 2.0);
    
    if (theta < 1e-8) {
        rvec = Eigen::Vector3d::Zero();
    } else {
        rvec << R(2, 1) - R(1, 2),
                R(0, 2) - R(2, 0),
                R(1, 0) - R(0, 1);
        rvec *= theta / (2.0 * sin(theta));
    }
}

// Save calibration result - output in input format (T_cam_to_body)
void saveResult(const int &frame_id) {
    std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
    std::ofstream fCalib(file_name);
    if (!fCalib.is_open()) {
        cerr << "open file " << file_name << " failed." << endl;
        return;
    }
    
    const CameraConfig &cam = cameras[frame_id];
    
    // Convert T_body_to_cam to T_cam_to_body (inverse)
    Eigen::Matrix4d T_cam_to_body = cam.extrinsic.inverse();
    Eigen::Matrix3d R_cam_to_body = T_cam_to_body.block<3, 3>(0, 0);
    Eigen::Vector3d t_cam_in_body = T_cam_to_body.block<3, 1>(0, 3);
    
    // Convert rotation matrix to Rodrigues vector
    Eigen::Vector3d rvec;
    rotationMatrixToRodrigues(R_cam_to_body, rvec);
    
    // Text format output
    fCalib << "Extrinsic (T_body_to_cam):" << endl;
    fCalib << "R:\n"
           << cam.extrinsic(0, 0) << " " << cam.extrinsic(0, 1) << " " << cam.extrinsic(0, 2) << "\n"
           << cam.extrinsic(1, 0) << " " << cam.extrinsic(1, 1) << " " << cam.extrinsic(1, 2) << "\n"
           << cam.extrinsic(2, 0) << " " << cam.extrinsic(2, 1) << " " << cam.extrinsic(2, 2) << endl;
    fCalib << "t: " << cam.extrinsic(0, 3) << " " << cam.extrinsic(1, 3) << " " << cam.extrinsic(2, 3) << endl;
    fCalib << "height: " << cam.height << endl;
    
    // JSON format - same as input format (T_cam_to_body)
    fCalib << "\n************* Output JSON format (T_cam_to_body) *************" << endl;
    fCalib << "\"" << json_names[frame_id] << "\": {" << endl;
    fCalib << "    \"rotation\": [" << rvec(0) << ", " << rvec(1) << ", " << rvec(2) << "]," << endl;
    fCalib << "    \"translation\": [" << t_cam_in_body(0) << ", " << t_cam_in_body(1) << ", " << t_cam_in_body(2) << "]" << endl;
    fCalib << "}" << endl;
    
    fCalib.close();
    cout << "Saved calibration to " << file_name << endl;
    cout << "  Rotation (Rodrigues): [" << rvec.transpose() << "]" << endl;
    cout << "  Translation (cam in body): [" << t_cam_in_body.transpose() << "]" << endl;
}

// Save all extrinsics to a single JSON file (same format as input)
void saveAllExtrinsics(const string &filename) {
    std::ofstream f(filename);
    if (!f.is_open()) {
        cerr << "Failed to open " << filename << " for writing" << endl;
        return;
    }
    
    f << "{" << endl;
    f << "    \"extrinsic_param\": {" << endl;
    
    for (int i = 0; i < 4; i++) {
        const CameraConfig &cam = cameras[i];
        
        // Convert T_body_to_cam to T_cam_to_body (inverse)
        Eigen::Matrix4d T_cam_to_body = cam.extrinsic.inverse();
        Eigen::Matrix3d R_cam_to_body = T_cam_to_body.block<3, 3>(0, 0);
        Eigen::Vector3d t_cam_in_body = T_cam_to_body.block<3, 1>(0, 3);
        
        // Convert rotation matrix to Rodrigues vector
        Eigen::Vector3d rvec;
        rotationMatrixToRodrigues(R_cam_to_body, rvec);
        
        f << "        \"" << json_names[i] << "\": {" << endl;
        f << "            \"rotation\": [" << rvec(0) << ", " << rvec(1) << ", " << rvec(2) << "]," << endl;
        f << "            \"translation\": [" << t_cam_in_body(0) << ", " << t_cam_in_body(1) << ", " << t_cam_in_body(2) << "]" << endl;
        f << "        }";
        if (i < 3) f << ",";
        f << endl;
    }
    
    f << "    }" << endl;
    f << "}" << endl;
    f.close();
    
    cout << "\n==========================================" << endl;
    cout << "Saved all extrinsics to: " << filename << endl;
    cout << "==========================================" << endl;
}

// Manual calibration via keyboard
bool ManualCalibration(int key_input, int frame) {
    char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
    bool real_hit = false;
    for (int32_t i = 0; i < 12; i++) {
        if (key_input == table[i]) {
            cameras[frame].extrinsic = cameras[frame].extrinsic * modification_list_[frame][i];
            real_hit = true;
        }
    }
    return real_hit;
}

// Initialize KG matrix
void initializeKG() {
    KG = Eigen::Matrix3d::Zero();
    KG(0, 0) = 1.0 / metric_ratio_m;
    KG(1, 1) = 1.0 / metric_ratio_m;
    KG(0, 2) = bev_cols / 2.0;
    KG(1, 2) = bev_rows / 2.0;
    KG(2, 2) = 1.0;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cout << "Usage: ./run_avm_new <image_path> <ocam_param_path> <extrinsic_json> [camera_model]" << endl;
        cout << "  camera_model: 0=fisheye, 1=ocam(default), 2=pinhole" << endl;
        cout << "\nExample (Ocam model):" << endl;
        cout << "  ./run_avm_new ./imgs ./ocam_params ./extrinsics.json 1" << endl;
        cout << "\nExample (Pinhole model):" << endl;
        cout << "  ./run_avm_new ./imgs ./intrinsics ./extrinsics.json 2" << endl;
        return 0;
    }
    
    string image_path = argv[1];
    string param_path = argv[2];
    string extrinsic_file = argv[3];
    if (argc >= 5) {
        camera_model = stoi(argv[4]);
    }
    
    cout << "==========================================" << endl;
    cout << "AVM Manual Calibration (New Format)" << endl;
    cout << "==========================================" << endl;
    cout << "Camera model: " << (camera_model == 0 ? "Fisheye" : (camera_model == 1 ? "Ocam" : "Pinhole")) << endl;
    
    // Initialize camera configs
    for (int i = 0; i < 4; i++) {
        cameras[i].name = camera_names[i];
        cameras[i].json_name = json_names[i];
        cameras[i].image_file = image_path + "/" + (i == 0 ? "Front" : (i == 1 ? "Left" : (i == 2 ? "Back" : "Right"))) + ".png";
    }
    
    // Load images
    cout << "\nLoading images..." << endl;
    for (int i = 0; i < 4; i++) {
        cameras[i].img = imread(cameras[i].image_file);
        if (cameras[i].img.empty()) {
            cerr << "Failed to load image: " << cameras[i].image_file << endl;
            return 1;
        }
        cout << "  " << cameras[i].name << ": " << cameras[i].img.cols << "x" << cameras[i].img.rows << endl;
    }
    
    // Load intrinsics
    cout << "\nLoading intrinsics..." << endl;
    if (camera_model == 1) {
        // Ocam model
        for (int i = 0; i < 4; i++) {
            string ocam_file = param_path + "/" + json_names[i] + ".json";
            loadOcamParams(cameras[i], ocam_file);
        }
    } else {
        // Pinhole/fisheye model
        for (int i = 0; i < 4; i++) {
            string int_file = param_path + "/" + (i == 0 ? "Front" : (i == 1 ? "Left" : (i == 2 ? "Back" : "Right"))) + ".json";
            loadPinholeIntrinsic(int_file, cameras[i].K, cameras[i].dist);
        }
    }
    
    // Load extrinsics
    cout << "\nLoading extrinsics..." << endl;
    loadExtrinsics(extrinsic_file);
    
    // Initialize KG
    initializeKG();
    
    // Initialize calibration
    CalibrationInit();
    
    // Generate initial BEV images
    cout << "\nGenerating initial BEV images..." << endl;
    for (int i = 0; i < 4; i++) {
        cameras[i].img_bev = project_on_ground(cameras[i].img, cameras[i], cameras[i].name);
        cameras[i].img_bev = tail(cameras[i].img_bev, cameras[i].name);
        cout << "  " << cameras[i].name << " BEV generated" << endl;
    }
    
    // Generate initial surround view
    Mat img_surround = generate_surround_view(
        cameras[0].img_bev, cameras[1].img_bev,
        cameras[2].img_bev, cameras[3].img_bev);
    
    cout << "\n==========================================" << endl;
    cout << "Starting GUI..." << endl;
    cout << "==========================================" << endl;
    
    // Setup Pangolin
    int width = 1000, height = 1000;
    pangolin::CreateWindowAndBind("AVM Manual Calibration", width, height);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::View &project_image = pangolin::Display("project")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0, 1.0 * width / height)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    unsigned char *imageArray = new unsigned char[3 * width * height];
    pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    
    // Control panel
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(0), 1.0, 0.0, pangolin::Attach::Pix(150));
    pangolin::Var<bool> displayMode("cp.stitching", false, true);
    pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 5);
    pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 200);
    pangolin::Var<int> frameStep("cp.frame", 0, 0, 3);
    
    pangolin::Var<bool> addXdegree("cp.+ x deg", false, false);
    pangolin::Var<bool> minusXdegree("cp.- x deg", false, false);
    pangolin::Var<bool> addYdegree("cp.+ y deg", false, false);
    pangolin::Var<bool> minusYdegree("cp.- y deg", false, false);
    pangolin::Var<bool> addZdegree("cp.+ z deg", false, false);
    pangolin::Var<bool> minusZdegree("cp.- z deg", false, false);
    pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
    pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
    pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
    pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
    pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
    pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);
    
    pangolin::Var<bool> resetButton("cp.Reset", false, false);
    pangolin::Var<bool> saveButton("cp.Save", false, false);
    
    std::vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(addXdegree);
    mat_calib_box.push_back(minusXdegree);
    mat_calib_box.push_back(addYdegree);
    mat_calib_box.push_back(minusYdegree);
    mat_calib_box.push_back(addZdegree);
    mat_calib_box.push_back(minusZdegree);
    mat_calib_box.push_back(addXtrans);
    mat_calib_box.push_back(minusXtrans);
    mat_calib_box.push_back(addYtrans);
    mat_calib_box.push_back(minusYtrans);
    mat_calib_box.push_back(addZtrans);
    mat_calib_box.push_back(minusZtrans);
    
    Mat current_frame = cameras[0].img_bev.clone();
    
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        if (displayMode.GuiChanged()) {
            cali_stiching_mode = displayMode.Get();
        }
        
        if (degreeStep.GuiChanged()) {
            cali_scale_degree_ = degreeStep.Get();
            CalibrationScaleChange(cali_frame);
            cout << "Degree calib scale: " << cali_scale_degree_ << " degree" << endl;
        }
        if (tStep.GuiChanged()) {
            cali_scale_trans_ = tStep.Get() / 100.0;
            CalibrationScaleChange(cali_frame);
            cout << "Trans calib scale: " << cali_scale_trans_ * 100 << " cm" << endl;
        }
        
        if (frameStep.GuiChanged()) {
            cali_frame = frameStep.Get();
            cout << "Frame changed to " << cali_frame << " (" << cameras[cali_frame].name << ")" << endl;
            CalibrationScaleChange(cali_frame);
            cameras[cali_frame].img_bev = project_on_ground(
                cameras[cali_frame].img, cameras[cali_frame], cameras[cali_frame].name);
            cameras[cali_frame].img_bev = tail(cameras[cali_frame].img_bev, cameras[cali_frame].name);
        }
        
        // Handle calibration buttons
        for (int i = 0; i < 12; i++) {
            if (pangolin::Pushed(mat_calib_box[i])) {
                if (i == 10) {
                    cameras[cali_frame].height -= cali_scale_trans_;
                } else if (i == 11) {
                    cameras[cali_frame].height += cali_scale_trans_;
                }
                cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic * modification_list_[cali_frame][i];
                cout << "Calibrated " << cameras[cali_frame].name << ", height: " << cameras[cali_frame].height << endl;
                cameras[cali_frame].img_bev = project_on_ground(
                    cameras[cali_frame].img, cameras[cali_frame], cameras[cali_frame].name);
                cameras[cali_frame].img_bev = tail(cameras[cali_frame].img_bev, cameras[cali_frame].name);
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            cameras[cali_frame].height = cameras[cali_frame].extrinsic_initial.block<3,1>(0,3).norm();
            cameras[cali_frame].img_bev = project_on_ground(
                cameras[cali_frame].img, cameras[cali_frame], cameras[cali_frame].name);
            cameras[cali_frame].img_bev = tail(cameras[cali_frame].img_bev, cameras[cali_frame].name);
            cout << "Reset " << cameras[cali_frame].name << endl;
        }
        
        if (pangolin::Pushed(saveButton)) {
            if (cali_stiching_mode) {
                for (int i = 0; i < 4; i++) {
                    saveResult(i);
                }
                imwrite("stitching.png", img_surround);
                saveAllExtrinsics("camera_extrinsics_calibrated.json");
                cout << "Saved all calibrations" << endl;
            } else {
                saveResult(cali_frame);
                string img_name = "calibimg_" + to_string(cali_frame) + ".png";
                imwrite(img_name, current_frame);
                cout << "Saved " << cameras[cali_frame].name << endl;
            }
        }
        
        // Keyboard input
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                cameras[cali_frame].img_bev = project_on_ground(
                    cameras[cali_frame].img, cameras[cali_frame], cameras[cali_frame].name);
                cameras[cali_frame].img_bev = tail(cameras[cali_frame].img_bev, cameras[cali_frame].name);
                cout << "Key calibration applied to " << cameras[cali_frame].name << endl;
            }
        }
        
        // Update display
        if (cali_stiching_mode) {
            img_surround = generate_surround_view(
                cameras[0].img_bev, cameras[1].img_bev,
                cameras[2].img_bev, cameras[3].img_bev);
            current_frame = img_surround.clone();
        } else {
            current_frame = cameras[cali_frame].img_bev.clone();
        }
        
        imageArray = current_frame.data;
        imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);
        
        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewportFlipY();
        
        pangolin::FinishFrame();
        glFinish();
    }
    
    delete[] imageArray;
    return 0;
}
