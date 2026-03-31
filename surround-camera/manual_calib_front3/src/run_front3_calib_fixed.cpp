/*
 * Front 3 Cameras Manual Calibration - FIXED VERSION
 * FL(Fisheye) + F(Ocam) + FR(Fisheye) BEV projection with proper 3D ground projection
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
#include <sstream>
#include <iomanip>
#include "calibration_metrics.hpp"

using namespace std;
using namespace cv;
using namespace CalibrationMetrics;

// Camera types
enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1, PINHOLE = 2 };

// BEV configuration
float metric_ratio_m = 0.02;  // 每个像素代表的实际距离 (米)
int bev_rows = 1500;          // BEV图像高度
int bev_cols = 3000;          // BEV图像宽度
float wheel_base = 2.8;       // 轴距 (米)

// Camera configuration
struct FrontCameraConfig {
    string name;
    string json_name;
    string image_file;
    CameraType type;
    
    // Fisheye intrinsics (OpenCV 4 param)
    Mat camera_matrix;
    Mat dist_coeffs;
    Eigen::Matrix3d K;
    vector<double> dist;
    
    // Ocam intrinsics
    struct OcamParams {
        double cx, cy, c, d, e;
        vector<double> world2cam_poly;
        Eigen::VectorXd world2cam_coeffs;
        int width, height;
    } ocam;
    
    int width, height;
    float height_above_ground;  // 相机离地面高度
    
    // Extrinsics
    Eigen::Matrix4d extrinsic;
    Eigen::Matrix4d extrinsic_initial;
    
    // Images
    Mat img_original;
    Mat img_bev;  // BEV投影图像
};

FrontCameraConfig cameras[3];
const string camera_names[3] = {"front_left", "front", "front_right"};
const string json_names[3] = {"front_left", "park_front", "front_right"};
const CameraType camera_types[3] = {FISHEYE_OPENCV, OCAM, FISHEYE_OPENCV};

int cali_frame = 0;
double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
float cali_scale_height_ = 0.01;  // 高度调整步长

vector<Eigen::Matrix4d> modification_list_[3];

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

Eigen::Matrix3d rodriguesToRotationMatrix(const vector<double> &rvec) {
    double theta = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    if (theta < 1e-8) return Eigen::Matrix3d::Identity();
    double r[3] = {rvec[0]/theta, rvec[1]/theta, rvec[2]/theta};
    double cos_t = cos(theta), sin_t = sin(theta), omc = 1.0 - cos_t;
    Eigen::Matrix3d R;
    R << cos_t + omc*r[0]*r[0], omc*r[0]*r[1] - sin_t*r[2], omc*r[0]*r[2] + sin_t*r[1],
         omc*r[1]*r[0] + sin_t*r[2], cos_t + omc*r[1]*r[1], omc*r[1]*r[2] - sin_t*r[0],
         omc*r[2]*r[0] - sin_t*r[1], omc*r[2]*r[1] + sin_t*r[0], cos_t + omc*r[2]*r[2];
    return R;
}

// Load fisheye intrinsics
void loadFisheyeIntrinsic(FrontCameraConfig &cam, const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) { cerr << "Cannot open: " << filename << endl; exit(1); }
    Json::Value root; Json::Reader reader;
    if (!reader.parse(file, root)) { cerr << "Failed to parse: " << filename << endl; exit(1); }
    
    const Json::Value &intrinsic = root["intrinsic_param"];
    cam.width = intrinsic["camera_width"].asInt();
    cam.height = intrinsic["camera_height"].asInt();
    
    const Json::Value &K = intrinsic["camera_matrix"];
    cam.camera_matrix = Mat::eye(3, 3, CV_64FC1);
    cam.K = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double val = K[i][j].asDouble();
        cam.camera_matrix.at<double>(i, j) = val;
        cam.K(i, j) = val;
    }
    
    const Json::Value &D = intrinsic["distortion_coeffcients"];
    cam.dist_coeffs = Mat::zeros(1, 4, CV_64FC1);
    cam.dist.clear();
    for (int i = 0; i < 4 && i < (int)D.size(); i++) {
        double val = D[i].asDouble();
        cam.dist_coeffs.at<double>(0, i) = val;
        cam.dist.push_back(val);
    }
    
    cout << "Loaded fisheye " << cam.name << ": " << cam.width << "x" << cam.height << endl;
}

// Load Ocam intrinsics
void loadOcamIntrinsic(FrontCameraConfig &cam, const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) { cerr << "Cannot open: " << filename << endl; exit(1); }
    Json::Value root; Json::Reader reader;
    if (!reader.parse(file, root)) { cerr << "Failed to parse: " << filename << endl; exit(1); }
    
    const Json::Value &intrinsic = root["intrinsic_param"];
    cam.ocam.width = intrinsic["camera_width"].asInt();
    cam.ocam.height = intrinsic["camera_height"].asInt();
    cam.width = cam.ocam.width; cam.height = cam.ocam.height;
    cam.ocam.cx = intrinsic["principal_point"][0].asDouble();
    cam.ocam.cy = intrinsic["principal_point"][1].asDouble();
    cam.ocam.c = intrinsic["affine_c"].asDouble();
    cam.ocam.d = intrinsic["affine_d"].asDouble();
    cam.ocam.e = intrinsic["affine_e"].asDouble();
    
    const Json::Value &world2cam = intrinsic["world2cam"];
    cam.ocam.world2cam_poly.clear();
    for (int i = 0; i < (int)world2cam.size(); i++)
        cam.ocam.world2cam_poly.push_back(world2cam[i].asDouble());
    
    cam.ocam.world2cam_coeffs.resize(cam.ocam.world2cam_poly.size());
    for (size_t i = 0; i < cam.ocam.world2cam_poly.size(); i++)
        cam.ocam.world2cam_coeffs(i) = cam.ocam.world2cam_poly[cam.ocam.world2cam_poly.size() - 1 - i];
    
    cout << "Loaded Ocam " << cam.name << ": " << cam.width << "x" << cam.height << endl;
}

// Load extrinsics
void loadExtrinsics(const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) { cerr << "Cannot open: " << filename << endl; exit(1); }
    Json::Value root; Json::Reader reader;
    if (!reader.parse(file, root)) { cerr << "Failed to parse: " << filename << endl; exit(1); }
    
    const Json::Value &extrinsics = root["extrinsic_param"];
    for (int i = 0; i < 3; i++) {
        if (!extrinsics.isMember(json_names[i])) {
            cerr << "Missing extrinsics for " << json_names[i] << endl;
            exit(1);
        }
        const Json::Value &cam_ext = extrinsics[json_names[i]];
        vector<double> rvec(3);
        for (int j = 0; j < 3; j++) rvec[j] = cam_ext["rotation"][j].asDouble();
        Eigen::Vector3d translation_vec;
        for (int j = 0; j < 3; j++) translation_vec[j] = cam_ext["translation"][j].asDouble();
        
        Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
        Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
        T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
        T_cam_to_body.block<3, 1>(0, 3) = translation_vec;
        
        cameras[i].extrinsic = T_cam_to_body.inverse();
        cameras[i].extrinsic_initial = cameras[i].extrinsic;
        
        // 计算相机高度
        cameras[i].height_above_ground = cameras[i].extrinsic.block<3,1>(0,3).norm();
        
        cout << "Loaded " << cameras[i].name << " extrinsics, height: " 
             << cameras[i].height_above_ground << " m" << endl;
    }
}

// Ocam point projection - world2cam polynomial evaluation
void distortPointsOcamFull(const Mat& P_GC1, Mat& p_GC, const FrontCameraConfig::OcamParams& ocam) {
    for (int i = 0; i < P_GC1.cols; i++) {
        double x = P_GC1.at<Vec3d>(0, i)[0];
        double y = P_GC1.at<Vec3d>(0, i)[1];
        double z = P_GC1.at<Vec3d>(0, i)[2];
        
        double norm = sqrt(x*x + y*y);
        
        if (norm < 1e-14 || z >= 0) {
            p_GC.at<Vec2d>(0, i)[0] = -1;
            p_GC.at<Vec2d>(0, i)[1] = -1;
            continue;
        }
        
        // 计算入射角 theta (z为负，指向前方)
        double theta = atan(-z / norm);
        
        // 评估world2cam多项式
        double rho = 0.0;
        for (int k = 0; k < ocam.world2cam_coeffs.size(); k++) {
            rho = rho * theta + ocam.world2cam_coeffs(k);
        }
        
        // 归一化坐标
        double uu = x / norm * rho;
        double vv = y / norm * rho;
        
        // 应用仿射变换
        double u = uu * ocam.c + vv * ocam.d + ocam.cx;
        double v = uu * ocam.e + vv + ocam.cy;
        
        p_GC.at<Vec2d>(0, i)[0] = u;
        p_GC.at<Vec2d>(0, i)[1] = v;
    }
}

// Project image to ground (BEV) - proper 3D projection approach
Mat project_on_ground(const Mat& img, const FrontCameraConfig& cam, int cam_idx) {
    // 构建车身坐标系下的地面点矩阵 (4 x N)
    Mat P_G = Mat::ones(4, bev_rows * bev_cols, CV_64FC1);
    
    // 前视3相机的BEV配置 - 只关注前方区域
    // 坐标系: x向前，y向左，z向上
    for (int j = 0; j < bev_rows; j++) {
        for (int i = 0; i < bev_cols; i++) {
            int idx = j * bev_cols + i;
            
            // BEV图像映射到车身坐标系（前方区域）
            double veh_x = (bev_rows - j) * metric_ratio_m;  // 向前为正
            double veh_y = (i - bev_cols / 2.0) * metric_ratio_m;  // 向左为正
            
            P_G.at<double>(0, idx) = veh_x;
            P_G.at<double>(1, idx) = veh_y;
            P_G.at<double>(2, idx) = 0.0;  // 地面 z=0
        }
    }
    
    // 从车身坐标系转换到相机坐标系
    Eigen::MatrixXd P_G_eigen(4, bev_rows * bev_cols);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < bev_rows * bev_cols; j++) {
            P_G_eigen(i, j) = P_G.at<double>(i, j);
        }
    }
    
    Eigen::MatrixXd P_GC_eigen = cam.extrinsic * P_G_eigen;
    
    // 转换为3D点用于投影
    Mat P_GC1 = Mat::zeros(1, bev_rows * bev_cols, CV_64FC3);
    for (int i = 0; i < bev_rows * bev_cols; i++) {
        P_GC1.at<Vec3d>(0, i) = Vec3d(P_GC_eigen(0, i), P_GC_eigen(1, i), P_GC_eigen(2, i));
    }
    
    Mat p_GC = Mat::zeros(1, bev_rows * bev_cols, CV_64FC2);
    
    // 使用相应的相机模型投影
    if (cam.type == FISHEYE_OPENCV) {
        // OpenCV fisheye模型
        fisheye::distortPoints(P_GC1, p_GC, cam.camera_matrix, cam.dist_coeffs);
    } else if (cam.type == OCAM) {
        // Ocam模型
        distortPointsOcamFull(P_GC1, p_GC, cam.ocam);
    } else {
        // 针孔模型
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
    
    // reshape并remap
    Mat p_GC_table = p_GC.reshape(0, bev_rows);
    Mat p_GC_table_32F;
    p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);
    
    Mat img_GC;
    remap(img, img_GC, p_GC_table_32F, Mat(), INTER_LINEAR);
    
    return img_GC;
}

// Tail function for front cameras - keep only the front portion
Mat tail_front(const Mat& img, const string& name, int size = 600) {
    // 对于前视相机，只保留BEV图像的上部（前方区域）
    if (name.find("front") != string::npos) {
        Rect m_select = Rect(0, 0, img.cols, size);
        Mat cropped = img(m_select);
        Mat border(img.rows - size, img.cols, cropped.type(), Scalar(0, 0, 0));
        Mat dst;
        vconcat(cropped, border, dst);
        return dst;
    }
    return img.clone();
}

// Generate front 3 stitching view
Mat generateFront3StitchingView() {
    Mat stitched(bev_rows, bev_cols, CV_8UC3, Scalar(0, 0, 0));
    
    // 计算融合区域
    int left_blend_start = bev_cols * 0.28;
    int left_blend_end = bev_cols * 0.38;
    int right_blend_start = bev_cols * 0.62;
    int right_blend_end = bev_cols * 0.72;
    
    // 三列布局: FL | F | FR
    int col1_end = bev_cols / 3;
    int col2_end = 2 * bev_cols / 3;
    
    for (int y = 0; y < bev_rows; y++) {
        for (int x = 0; x < bev_cols; x++) {
            Vec3b pixel(0, 0, 0);
            bool has_pixel = false;
            
            if (x < left_blend_start) {
                // 纯FL区域
                if (x < cameras[0].img_bev.cols && y < cameras[0].img_bev.rows) {
                    pixel = cameras[0].img_bev.at<Vec3b>(y, x);
                    has_pixel = true;
                }
            } else if (x < left_blend_end) {
                // FL-F融合区域
                float alpha = (float)(x - left_blend_start) / (left_blend_end - left_blend_start);
                Vec3b p1(0,0,0), p2(0,0,0);
                bool valid1 = false, valid2 = false;
                
                if (x < cameras[0].img_bev.cols && y < cameras[0].img_bev.rows) {
                    p1 = cameras[0].img_bev.at<Vec3b>(y, x);
                    if (p1 != Vec3b(0,0,0)) valid1 = true;
                }
                int x2 = x - col1_end + bev_cols/6;
                if (x2 >= 0 && x2 < cameras[1].img_bev.cols && y < cameras[1].img_bev.rows) {
                    p2 = cameras[1].img_bev.at<Vec3b>(y, x2);
                    if (p2 != Vec3b(0,0,0)) valid2 = true;
                }
                
                if (valid1 && valid2) {
                    pixel = (1-alpha) * p1 + alpha * p2;
                    has_pixel = true;
                } else if (valid1) {
                    pixel = p1;
                    has_pixel = true;
                } else if (valid2) {
                    pixel = p2;
                    has_pixel = true;
                }
            } else if (x < right_blend_start) {
                // 纯F区域（需要调整x坐标）
                int x_f = x - col1_end + bev_cols/6;
                if (x_f >= 0 && x_f < cameras[1].img_bev.cols && y < cameras[1].img_bev.rows) {
                    pixel = cameras[1].img_bev.at<Vec3b>(y, x_f);
                    has_pixel = true;
                }
            } else if (x < right_blend_end) {
                // F-FR融合区域
                float alpha = (float)(x - right_blend_start) / (right_blend_end - right_blend_start);
                Vec3b p1(0,0,0), p2(0,0,0);
                bool valid1 = false, valid2 = false;
                
                int x1 = x - col1_end + bev_cols/6;
                if (x1 >= 0 && x1 < cameras[1].img_bev.cols && y < cameras[1].img_bev.rows) {
                    p1 = cameras[1].img_bev.at<Vec3b>(y, x1);
                    if (p1 != Vec3b(0,0,0)) valid1 = true;
                }
                int x2 = x - col2_end + bev_cols/6;
                if (x2 >= 0 && x2 < cameras[2].img_bev.cols && y < cameras[2].img_bev.rows) {
                    p2 = cameras[2].img_bev.at<Vec3b>(y, x2);
                    if (p2 != Vec3b(0,0,0)) valid2 = true;
                }
                
                if (valid1 && valid2) {
                    pixel = (1-alpha) * p1 + alpha * p2;
                    has_pixel = true;
                } else if (valid1) {
                    pixel = p1;
                    has_pixel = true;
                } else if (valid2) {
                    pixel = p2;
                    has_pixel = true;
                }
            } else {
                // 纯FR区域
                int x_fr = x - col2_end + bev_cols/6;
                if (x_fr >= 0 && x_fr < cameras[2].img_bev.cols && y < cameras[2].img_bev.rows) {
                    pixel = cameras[2].img_bev.at<Vec3b>(y, x_fr);
                    has_pixel = true;
                }
            }
            
            if (has_pixel) {
                stitched.at<Vec3b>(y, x) = pixel;
            }
        }
    }
    
    return stitched;
}

// Initialize calibration
void CalibrationInit() {
    for (int cam_idx = 0; cam_idx < 3; cam_idx++) {
        modification_list_[cam_idx].reserve(12);
        for (int32_t j = 0; j < 12; j++) {
            vector<int> transform_flag(6, 0);
            transform_flag[j / 2] = (j % 2) ? (-1) : 1;
            Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rot_tmp;
            rot_tmp = Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
            tmp.block(0, 0, 3, 3) = rot_tmp;
            tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
            tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
            tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
            modification_list_[cam_idx].push_back(tmp);
        }
    }
    cout << "=>Calibration scale Init!" << endl;
}

void CalibrationScaleChange(int frame) {
    modification_list_[frame].clear();
    for (int32_t i = 0; i < 12; i++) {
        vector<int> transform_flag(6, 0);
        transform_flag[i / 2] = (i % 2) ? (-1) : 1;
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rot_tmp;
        rot_tmp = Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
        tmp.block(0, 0, 3, 3) = rot_tmp;
        tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
        tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
        tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
        modification_list_[frame].push_back(tmp);
    }
}

// Convert rotation matrix to Rodrigues
void rotationMatrixToRodrigues(const Eigen::Matrix3d &R, Eigen::Vector3d &rvec) {
    double theta = acos(max(-1.0, min(1.0, (R.trace() - 1.0) / 2.0)));
    if (theta < 1e-8) { rvec = Eigen::Vector3d::Zero(); }
    else {
        rvec << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        rvec *= theta / (2.0 * sin(theta));
    }
}

// Save result
void saveResult(const int &frame_id) {
    string file_name = "calibration_" + camera_names[frame_id] + ".txt";
    ofstream fCalib(file_name);
    if (!fCalib.is_open()) { cerr << "open file " << file_name << " failed." << endl; return; }
    
    const FrontCameraConfig &cam = cameras[frame_id];
    Eigen::Matrix4d T_cam_to_body = cam.extrinsic.inverse();
    Eigen::Matrix3d R_cam_to_body = T_cam_to_body.block<3, 3>(0, 0);
    Eigen::Vector3d t_cam_in_body = T_cam_to_body.block<3, 1>(0, 3);
    Eigen::Vector3d rvec; rotationMatrixToRodrigues(R_cam_to_body, rvec);
    
    fCalib << "Extrinsic (T_body_to_cam):" << endl;
    fCalib << "R:\n" << cam.extrinsic(0, 0) << " " << cam.extrinsic(0, 1) << " " << cam.extrinsic(0, 2) << "\n"
           << cam.extrinsic(1, 0) << " " << cam.extrinsic(1, 1) << " " << cam.extrinsic(1, 2) << "\n"
           << cam.extrinsic(2, 0) << " " << cam.extrinsic(2, 1) << " " << cam.extrinsic(2, 2) << endl;
    fCalib << "t: " << cam.extrinsic(0, 3) << " " << cam.extrinsic(1, 3) << " " << cam.extrinsic(2, 3) << endl;
    fCalib << "height: " << cam.height_above_ground << endl;
    fCalib << "\n************* Output JSON format (T_cam_to_body) *************" << endl;
    fCalib << "\"" << json_names[frame_id] << "\": {" << endl;
    fCalib << "    \"rotation\": [" << rvec(0) << ", " << rvec(1) << ", " << rvec(2) << "]," << endl;
    fCalib << "    \"translation\": [" << t_cam_in_body(0) << ", " << t_cam_in_body(1) << ", " << t_cam_in_body(2) << "]" << endl;
    fCalib << "}" << endl;
    fCalib.close();
    cout << "Saved calibration to " << file_name << endl;
}

// Save all extrinsics
void saveAllExtrinsics(const string &filename) {
    ofstream f(filename);
    if (!f.is_open()) { cerr << "Failed to open " << filename << endl; return; }
    
    f << "{" << endl << "    \"extrinsic_param\": {" << endl;
    for (int i = 0; i < 3; i++) {
        const FrontCameraConfig &cam = cameras[i];
        Eigen::Matrix4d T_cam_to_body = cam.extrinsic.inverse();
        Eigen::Matrix3d R_cam_to_body = T_cam_to_body.block<3, 3>(0, 0);
        Eigen::Vector3d t_cam_in_body = T_cam_to_body.block<3, 1>(0, 3);
        Eigen::Vector3d rvec; rotationMatrixToRodrigues(R_cam_to_body, rvec);
        
        f << "        \"" << json_names[i] << "\": {" << endl;
        f << "            \"rotation\": [" << rvec(0) << ", " << rvec(1) << ", " << rvec(2) << "]," << endl;
        f << "            \"translation\": [" << t_cam_in_body(0) << ", " << t_cam_in_body(1) << ", " << t_cam_in_body(2) << "]" << endl;
        f << "        }";
        if (i < 2) f << ",";
        f << endl;
    }
    f << "    }" << endl << "}" << endl;
    f.close();
    cout << "\n==========================================" << endl;
    cout << "Saved all extrinsics to: " << filename << endl;
    cout << "==========================================" << endl;
}

// Manual calibration
bool ManualCalibration(int key_input, int frame) {
    char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
    for (int32_t i = 0; i < 12; i++) {
        if (key_input == table[i]) {
            cameras[frame].extrinsic = cameras[frame].extrinsic * modification_list_[frame][i];
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cout << "Usage: ./run_front3_calib <image_path> <intrinsic_path> <extrinsic_json>" << endl;
        cout << "\nExample:" << endl;
        cout << "  ./run_front3_calib ./2_dog/imgs ./2_dog/param ./2_dog/param/camera_extrinsics_ikalibr.json5" << endl;
        cout << "\nNote: Images should be named: front_left.png, front.png, front_right.png" << endl;
        return 0;
    }
    
    string image_path = argv[1];
    string intrinsic_path = argv[2];
    string extrinsic_file = argv[3];
    
    cout << "==========================================" << endl;
    cout << "Front 3 Cameras Manual Calibration" << endl;
    cout << "==========================================" << endl;
    
    // Initialize camera configs
    for (int i = 0; i < 3; i++) {
        cameras[i].name = camera_names[i];
        cameras[i].json_name = json_names[i];
        cameras[i].type = camera_types[i];
        cameras[i].image_file = image_path + "/" + camera_names[i] + ".png";
    }
    
    // Load images
    cout << "\nLoading images..." << endl;
    for (int i = 0; i < 3; i++) {
        cameras[i].img_original = imread(cameras[i].image_file);
        if (cameras[i].img_original.empty()) {
            cerr << "Failed to load: " << cameras[i].image_file << endl;
            // Try alternative names
            if (i == 0) cameras[i].image_file = image_path + "/front_left.png";
            else if (i == 1) cameras[i].image_file = image_path + "/front.png";
            else cameras[i].image_file = image_path + "/front_right.png";
            
            cameras[i].img_original = imread(cameras[i].image_file);
            if (cameras[i].img_original.empty()) {
                cerr << "Also failed: " << cameras[i].image_file << endl;
                return 1;
            }
        }
        cout << "  " << cameras[i].name << ": " << cameras[i].img_original.cols << "x" 
             << cameras[i].img_original.rows << endl;
    }
    
    // Load intrinsics
    cout << "\nLoading intrinsics..." << endl;
    loadFisheyeIntrinsic(cameras[0], intrinsic_path + "/front_left.json");
    loadOcamIntrinsic(cameras[1], intrinsic_path + "/park_front.json");
    loadFisheyeIntrinsic(cameras[2], intrinsic_path + "/front_right.json");
    
    // Load extrinsics
    cout << "\nLoading extrinsics..." << endl;
    loadExtrinsics(extrinsic_file);
    
    // Initialize calibration
    CalibrationInit();
    
    // Generate initial BEV images
    cout << "\nGenerating initial BEV images..." << endl;
    for (int i = 0; i < 3; i++) {
        cameras[i].img_bev = project_on_ground(cameras[i].img_original, cameras[i], i);
        cameras[i].img_bev = tail_front(cameras[i].img_bev, cameras[i].name);
        cout << "  " << cameras[i].name << " BEV generated" << endl;
    }
    
    Mat stitched = generateFront3StitchingView();
    
    cout << "\n==========================================" << endl;
    cout << "Starting GUI..." << endl;
    cout << "==========================================" << endl;
    
    // Setup Pangolin
    int width = 1400, height = 800;
    pangolin::CreateWindowAndBind("Front 3 Cameras Calibration", width, height);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::View &project_image = pangolin::Display("project")
        .SetBounds(0.2, 1.0, pangolin::Attach::Pix(150), 1.0, (double)bev_cols / bev_rows)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    unsigned char *imageArray = new unsigned char[3 * bev_cols * bev_rows];
    pangolin::GlTexture imageTexture(bev_cols, bev_rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    
    // Control panel
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(0), 1.0, 0.0, pangolin::Attach::Pix(150));
    pangolin::Var<int> frameStep("cp.frame", 0, 0, 2);
    pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 5);
    pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 200);
    
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
    
    vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(addXdegree); mat_calib_box.push_back(minusXdegree);
    mat_calib_box.push_back(addYdegree); mat_calib_box.push_back(minusYdegree);
    mat_calib_box.push_back(addZdegree); mat_calib_box.push_back(minusZdegree);
    mat_calib_box.push_back(addXtrans); mat_calib_box.push_back(minusXtrans);
    mat_calib_box.push_back(addYtrans); mat_calib_box.push_back(minusYtrans);
    mat_calib_box.push_back(addZtrans); mat_calib_box.push_back(minusZtrans);
    
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        if (frameStep.GuiChanged()) {
            cali_frame = frameStep.Get();
            cout << "Frame: " << cali_frame << " (" << cameras[cali_frame].name << ")" << endl;
            CalibrationScaleChange(cali_frame);
        }
        
        if (degreeStep.GuiChanged()) {
            cali_scale_degree_ = degreeStep.Get();
            CalibrationScaleChange(cali_frame);
            cout << "Degree step: " << cali_scale_degree_ << endl;
        }
        
        if (tStep.GuiChanged()) {
            cali_scale_trans_ = tStep.Get() / 100.0;
            CalibrationScaleChange(cali_frame);
            cout << "Trans step: " << cali_scale_trans_ * 100 << " cm" << endl;
        }
        
        // Handle calibration buttons
        for (int i = 0; i < 12; i++) {
            if (pangolin::Pushed(mat_calib_box[i])) {
                cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic * modification_list_[cali_frame][i];
                cout << "Calibrated " << cameras[cali_frame].name << endl;
                
                // Regenerate BEV
                cameras[cali_frame].img_bev = project_on_ground(
                    cameras[cali_frame].img_original, cameras[cali_frame], cali_frame);
                cameras[cali_frame].img_bev = tail_front(cameras[cali_frame].img_bev, cameras[cali_frame].name);
                stitched = generateFront3StitchingView();
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            cameras[cali_frame].height_above_ground = cameras[cali_frame].extrinsic_initial.block<3,1>(0,3).norm();
            cameras[cali_frame].img_bev = project_on_ground(
                cameras[cali_frame].img_original, cameras[cali_frame], cali_frame);
            cameras[cali_frame].img_bev = tail_front(cameras[cali_frame].img_bev, cameras[cali_frame].name);
            stitched = generateFront3StitchingView();
            cout << "Reset " << cameras[cali_frame].name << endl;
        }
        
        if (pangolin::Pushed(saveButton)) {
            for (int i = 0; i < 3; i++) saveResult(i);
            saveAllExtrinsics("front3_extrinsics_calibrated.json");
            imwrite("front3_stitched.png", stitched);
            cout << "Saved all calibrations" << endl;

            // 计算并显示评估指标
            cout << "\n========================================" << endl;
            cout << "  Computing Calibration Metrics..." << endl;
            cout << "========================================" << endl;

            // 计算掩码
            Mat mask_fl = Utils::computeValidMask(cameras[0].img_bev);
            Mat mask_f  = Utils::computeValidMask(cameras[1].img_bev);
            Mat mask_fr = Utils::computeValidMask(cameras[2].img_bev);

            // 评估 FL-F 对
            EvaluationReport report_fl_f = CalibrationEvaluator::evaluateCameraPair(
                cameras[0].img_bev, cameras[1].img_bev,
                mask_fl, mask_f, "FL-F");
            cout << CalibrationEvaluator::generateReportText(report_fl_f) << endl;

            // 评估 F-FR 对
            EvaluationReport report_f_fr = CalibrationEvaluator::evaluateCameraPair(
                cameras[1].img_bev, cameras[2].img_bev,
                mask_f, mask_fr, "F-FR");
            cout << CalibrationEvaluator::generateReportText(report_f_fr) << endl;

            // 综合得分
            double overall = (report_fl_f.overall_score + report_f_fr.overall_score) / 2.0;
            cout << "\n========================================" << endl;
            cout << "  FRONT-3 OVERALL SCORE: " << fixed << setprecision(1) << overall << "/100" << endl;
            if (overall >= 80) cout << "  Status: EXCELLENT - Calibration is optimal!" << endl;
            else if (overall >= 60) cout << "  Status: GOOD - Minor adjustments may help" << endl;
            else if (overall >= 40) cout << "  Status: FAIR - Further calibration needed" << endl;
            else cout << "  Status: POOR - Significant adjustment required" << endl;
            cout << "========================================" << endl;

            // 保存评估报告
            string report_file = "front3_calibration_metrics.txt";
            ofstream f(report_file);
            f << "Front 3 Cameras Calibration Quality Report\n";
            f << "========================================\n\n";
            f << "Overall Score: " << fixed << setprecision(1) << overall << "/100\n\n";
            f << CalibrationEvaluator::generateReportText(report_fl_f);
            f << "\n\n";
            f << CalibrationEvaluator::generateReportText(report_f_fr);
            f.close();
            cout << "\nMetrics report saved to: " << report_file << endl;
        }
        
        // Keyboard input
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                cameras[cali_frame].img_bev = project_on_ground(
                    cameras[cali_frame].img_original, cameras[cali_frame], cali_frame);
                cameras[cali_frame].img_bev = tail_front(cameras[cali_frame].img_bev, cameras[cali_frame].name);
                stitched = generateFront3StitchingView();
                cout << "Key calibration applied to " << cameras[cali_frame].name << endl;
            }
        }
        
        // Update display
        memcpy(imageArray, stitched.data, 3 * bev_cols * bev_rows);
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
