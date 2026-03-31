/*
 * Front 3 Cameras Manual Calibration
 * FL(Fisheye) + F(Ocam) + FR(Fisheye) stitching with perspective transform
 * With Calibration Quality Metrics
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
#include "calibration_metrics.hpp"

using namespace std;
using namespace cv;
using namespace CalibrationMetrics;

// Camera types
enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1 };

// Camera configuration
struct FrontCameraConfig {
    string name;
    string json_name;
    string image_file;
    CameraType type;
    
    // Fisheye intrinsics (OpenCV 4 param)
    Mat camera_matrix;
    Mat dist_coeffs;
    
    // Ocam intrinsics
    struct OcamParams {
        double cx, cy, c, d, e;
        vector<double> world2cam_poly;
        vector<double> cam2world_poly;
        Eigen::VectorXd world2cam_coeffs;
        int width, height;
    } ocam;
    
    int width, height;
    
    // Extrinsics
    Eigen::Matrix4d extrinsic;
    Eigen::Matrix4d extrinsic_initial;
    
    // Images
    Mat img_original;
    Mat img_undistorted;
    Mat img_warped;
    Mat perspective_transform;
};

FrontCameraConfig cameras[3];
const string camera_names[3] = {"front_left", "front", "front_right"};
const string json_names[3] = {"front_left", "park_front", "front_right"};
const CameraType camera_types[3] = {FISHEYE_OPENCV, OCAM, FISHEYE_OPENCV};

int cali_frame = 0;
double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
int output_width = 3000, output_height = 1000;
vector<Eigen::Matrix4d> modification_list_[3];

// Front wall projection parameters
// 前方墙面距离和尺寸参数 (单位: 米)
double wall_distance = 10.0;      // 墙面距离相机10米
double wall_width = 15.0;         // 墙面总宽度15米
double wall_height = 5.0;         // 墙面高度5米
double wall_center_y = 0.0;       // 墙面中心高度 (相对于相机高度)

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
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
        cam.camera_matrix.at<double>(i, j) = K[i][j].asDouble();
    
    const Json::Value &D = intrinsic["distortion_coeffcients"];
    cam.dist_coeffs = Mat::zeros(1, 4, CV_64FC1);
    for (int i = 0; i < 4 && i < (int)D.size(); i++)
        cam.dist_coeffs.at<double>(0, i) = D[i].asDouble();
    
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
    
    const Json::Value &cam2world = intrinsic["cam2world"];
    cam.ocam.cam2world_poly.clear();
    for (int i = 0; i < (int)cam2world.size(); i++)
        cam.ocam.cam2world_poly.push_back(cam2world[i].asDouble());
    
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
        cout << "Loaded " << cameras[i].name << " extrinsics" << endl;
    }
}

// Undistort fisheye
void undistortFisheye(FrontCameraConfig &cam) {
    Mat R = Mat::eye(3, 3, CV_32FC1);
    Mat new_K = getOptimalNewCameraMatrix(cam.camera_matrix, cam.dist_coeffs,
                                          Size(cam.width, cam.height), 1.0, Size(cam.width, cam.height));
    Mat map1, map2;
    fisheye::initUndistortRectifyMap(cam.camera_matrix, cam.dist_coeffs, R, new_K,
                                     Size(cam.width, cam.height), CV_16SC2, map1, map2);
    remap(cam.img_original, cam.img_undistorted, map1, map2, INTER_LINEAR);
    cam.camera_matrix = new_K;
}

// Undistort Ocam - correct implementation
// Maps from pinhole (output) coordinates to Ocam (input) coordinates
void undistortOcam(FrontCameraConfig &cam) {
    Mat map_x(cam.height, cam.width, CV_32FC1);
    Mat map_y(cam.height, cam.width, CV_32FC1);
    
    // Pinhole camera parameters for output image
    // Use Ocam cx as focal length estimate, center of image as principal point
    double f = cam.ocam.cx;  
    double cx = cam.width / 2.0;
    double cy = cam.height / 2.0;
    
    for (int v = 0; v < cam.height; v++) {
        for (int u = 0; u < cam.width; u++) {
            // 1. Output image (pinhole) coordinate -> normalized coordinate
            double x = (u - cx) / f;
            double y = (v - cy) / f;
            
            // 2. Compute angle theta from optical axis
            double r = sqrt(x*x + y*y);
            double theta = atan(r);  // theta = atan(r) for pinhole projection
            
            // 3. Use world2cam polynomial to get rho (radius in Ocam image)
            // world2cam maps theta -> rho
            double rho = 0.0;
            for (int k = 0; k < cam.ocam.world2cam_coeffs.size(); k++) {
                rho = rho * theta + cam.ocam.world2cam_coeffs(k);
            }
            
            // 4. Compute (uu, vv) in Ocam sensor coordinates (before affine transform)
            double uu, vv;
            if (r < 1e-10) {
                uu = 0;
                vv = 0;
            } else {
                // Scale by rho/r to get Ocam sensor coordinates
                uu = x / r * rho;
                vv = y / r * rho;
            }
            
            // 5. Apply affine transformation to get Ocam image coordinates
            // Forward affine: u = uu + vv*e + cx, v = uu*d + vv*c + cy
            double u_ocam = uu + vv * cam.ocam.e + cam.ocam.cx;
            double v_ocam = uu * cam.ocam.d + vv * cam.ocam.c + cam.ocam.cy;
            
            map_x.at<float>(v, u) = (float)u_ocam;
            map_y.at<float>(v, u) = (float)v_ocam;
        }
    }
    
    remap(cam.img_original, cam.img_undistorted, map_x, map_y, INTER_LINEAR);
}

// Compute perspective transform
void computePerspectiveTransform(FrontCameraConfig &cam, int cam_idx) {
    vector<Point2f> src_pts = {
        Point2f(0, 0), Point2f((float)cam.img_undistorted.cols, 0),
        Point2f((float)cam.img_undistorted.cols, (float)cam.img_undistorted.rows),
        Point2f(0, (float)cam.img_undistorted.rows)
    };
    
    float x_scale = output_width / 3.0f;
    float x_offset = cam_idx * x_scale * 0.7f;
    float y_offset = output_height * 0.1f;
    float y_scale = output_height * 0.8f;
    
    // Apply extrinsic adjustment
    Eigen::Matrix3d R = cam.extrinsic.block<3, 3>(0, 0);
    double yaw = atan2(R(0, 2), R(2, 2));
    float yaw_offset = (float)(yaw * 100);
    
    vector<Point2f> dst_pts = {
        Point2f(x_offset + yaw_offset, y_offset),
        Point2f(x_offset + x_scale + yaw_offset, y_offset),
        Point2f(x_offset + x_scale + yaw_offset, y_offset + y_scale),
        Point2f(x_offset + yaw_offset, y_offset + y_scale)
    };
    
    cam.perspective_transform = getPerspectiveTransform(src_pts, dst_pts);
}

// Apply perspective transform
void applyPerspectiveTransform(FrontCameraConfig &cam) {
    warpPerspective(cam.img_undistorted, cam.img_warped,
                    cam.perspective_transform, Size(output_width, output_height));
}

// Generate stitched view
Mat generateFrontStitchingView() {
    Mat stitched(output_height, output_width, CV_8UC3, Scalar(0, 0, 0));
    float x_scale = output_width / 3.0f;
    int blend_width = (int)(x_scale * 0.3f);
    
    // Blend and stitch
    for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
            if (x < x_scale - blend_width) {
                stitched.at<Vec3b>(y, x) = cameras[0].img_warped.at<Vec3b>(y, x);
            } else if (x < x_scale + blend_width) {
                float alpha = (x - (x_scale - blend_width)) / (2.0f * blend_width);
                stitched.at<Vec3b>(y, x) = (1 - alpha) * cameras[0].img_warped.at<Vec3b>(y, x)
                                         + alpha * cameras[1].img_warped.at<Vec3b>(y, x);
            } else if (x < 2 * x_scale - blend_width) {
                stitched.at<Vec3b>(y, x) = cameras[1].img_warped.at<Vec3b>(y, x);
            } else if (x < 2 * x_scale + blend_width) {
                float alpha = (x - (2 * x_scale - blend_width)) / (2.0f * blend_width);
                stitched.at<Vec3b>(y, x) = (1 - alpha) * cameras[1].img_warped.at<Vec3b>(y, x)
                                         + alpha * cameras[2].img_warped.at<Vec3b>(y, x);
            } else {
                stitched.at<Vec3b>(y, x) = cameras[2].img_warped.at<Vec3b>(y, x);
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
    double theta = acos((R.trace() - 1.0) / 2.0);
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
        cout << "  ./run_front3_calib ./imgs ./intrinsics ./extrinsics.json" << endl;
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
            return 1;
        }
        cout << "  " << cameras[i].name << ": " << cameras[i].img_original.cols << "x" << cameras[i].img_original.rows << endl;
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
    
    // Generate initial images
    cout << "\nGenerating initial images..." << endl;
    for (int i = 0; i < 3; i++) {
        if (cameras[i].type == FISHEYE_OPENCV) undistortFisheye(cameras[i]);
        else undistortOcam(cameras[i]);
        computePerspectiveTransform(cameras[i], i);
        applyPerspectiveTransform(cameras[i]);
        cout << "  " << cameras[i].name << " done" << endl;
    }
    
    Mat stitched = generateFrontStitchingView();
    
    cout << "\n==========================================" << endl;
    cout << "Starting GUI..." << endl;
    cout << "==========================================" << endl;
    
    // Setup Pangolin
    int width = 1200, height = 600;
    pangolin::CreateWindowAndBind("Front 3 Cameras Calibration", width, height);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::View &project_image = pangolin::Display("project")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0, 1.0 * width / height)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    unsigned char *imageArray = new unsigned char[3 * output_width * output_height];
    pangolin::GlTexture imageTexture(output_width, output_height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    
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
                
                // Recompute perspective transform
                computePerspectiveTransform(cameras[cali_frame], cali_frame);
                applyPerspectiveTransform(cameras[cali_frame]);
                stitched = generateFrontStitchingView();
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            computePerspectiveTransform(cameras[cali_frame], cali_frame);
            applyPerspectiveTransform(cameras[cali_frame]);
            stitched = generateFrontStitchingView();
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
            Mat mask_fl = Utils::computeValidMask(cameras[0].img_warped);
            Mat mask_f  = Utils::computeValidMask(cameras[1].img_warped);
            Mat mask_fr = Utils::computeValidMask(cameras[2].img_warped);

            // 评估 FL-F 对
            EvaluationReport report_fl_f = CalibrationEvaluator::evaluateCameraPair(
                cameras[0].img_warped, cameras[1].img_warped,
                mask_fl, mask_f, "FL-F");
            cout << CalibrationEvaluator::generateReportText(report_fl_f) << endl;

            // 评估 F-FR 对
            EvaluationReport report_f_fr = CalibrationEvaluator::evaluateCameraPair(
                cameras[1].img_warped, cameras[2].img_warped,
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
                computePerspectiveTransform(cameras[cali_frame], cali_frame);
                applyPerspectiveTransform(cameras[cali_frame]);
                stitched = generateFrontStitchingView();
                cout << "Key calibration applied to " << cameras[cali_frame].name << endl;
            }
        }
        
        // Update display
        imageArray = stitched.data;
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
