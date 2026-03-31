/*
 * Front 3 Cameras Manual Calibration - Version 2
 * FL(Fisheye) + F(Ocam) + FR(Fisheye) with Front Wall Projection
 * 
 * Projects all cameras to a virtual front wall for alignment
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
        Eigen::VectorXd world2cam_coeffs;
        int width, height;
    } ocam;
    
    int width, height;
    
    // Extrinsics
    Eigen::Matrix4d extrinsic;
    Eigen::Matrix4d extrinsic_initial;
    
    // Images
    Mat img_original;
    Mat img_projected;
};

FrontCameraConfig cameras[3];
const string camera_names[3] = {"front_left", "front", "front_right"};
const string json_names[3] = {"front_left", "park_front", "front_right"};
const CameraType camera_types[3] = {FISHEYE_OPENCV, OCAM, FISHEYE_OPENCV};

int cali_frame = 0;
double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;

// View mode: 0=stitching view, 1-3=single camera view
int view_mode = 0;  // Default to stitching view

// Front wall projection parameters (单位: 米)
double wall_distance = 8.0;       // 墙面距离相机8米
double wall_width = 12.0;         // 墙面宽度12米  
double wall_height = 6.0;         // 墙面高度6米
double wall_center_y = 0.0;       // 墙面中心高度 (相对于相机)

// Output image parameters
int output_width = 2400, output_height = 1200;
double meter_per_pixel = wall_width / output_width;  // 米/像素

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

// Project fisheye point to front wall
// Returns true if point is visible
bool projectFisheyeToWall(double u, double v, const FrontCameraConfig &cam,
                          double &wall_x, double &wall_y) {
    // 1. Undistort to normalized coordinates
    Mat src_pt = (Mat_<double>(1, 2) << u, v);
    Mat dst_pt;
    
    // Use OpenCV fisheye undistortPoints
    Mat K = cam.camera_matrix.clone();
    Mat D = cam.dist_coeffs.clone();
    
    // Convert to float for fisheye::undistortPoints
    Mat src_f, dst_f;
    src_pt.convertTo(src_f, CV_32FC2);
    fisheye::undistortPoints(src_f, dst_f, K, D);
    dst_f.convertTo(dst_pt, CV_64FC2);
    
    double xn = dst_pt.at<double>(0, 0);
    double yn = dst_pt.at<double>(0, 1);
    
    // 2. Ray from camera center: P = t * (xn, yn, 1)
    // 3. Intersect with front wall plane: z = wall_distance (in camera coordinates)
    
    // Transform wall plane to camera coordinates
    Eigen::Matrix4d T_body_to_cam = cam.extrinsic;
    Eigen::Vector4d wall_pt_body;
    wall_pt_body << 0, 0, wall_distance, 1;  // Point on wall in body frame
    Eigen::Vector4d wall_pt_cam = T_body_to_cam * wall_pt_body;
    
    // Wall normal in camera frame (pointing backward, i.e., negative z)
    Eigen::Vector4d wall_normal_body;
    wall_normal_body << 0, 0, -1, 0;
    Eigen::Vector4d wall_normal_cam = T_body_to_cam * wall_normal_body;
    
    // Ray direction in camera frame
    Eigen::Vector3d ray_dir(xn, yn, 1.0);
    ray_dir.normalize();
    
    // Intersect ray with wall plane
    double denom = ray_dir.dot(wall_normal_cam.head<3>());
    if (fabs(denom) < 1e-10) return false;  // Ray parallel to wall
    
    double t = (wall_pt_cam.head<3>().dot(wall_normal_cam.head<3>())) / denom;
    if (t <= 0) return false;  // Wall behind camera
    
    Eigen::Vector3d intersection_cam = t * ray_dir;
    
    // Transform back to body frame
    Eigen::Vector4d intersection_cam_h;
    intersection_cam_h << intersection_cam, 1.0;
    Eigen::Vector4d intersection_body = T_body_to_cam.inverse() * intersection_cam_h;
    
    // Convert to wall coordinates
    wall_x = intersection_body(1);  // y_body -> wall_x (left-right)
    wall_y = -intersection_body(2); // -z_body -> wall_y (up-down, negate because z is down)
    
    return true;
}

// Project Ocam point to front wall using CPAC model
bool projectOcamToWall(double u, double v, const FrontCameraConfig &cam,
                       double &wall_x, double &wall_y) {
    // 1. Inverse affine transformation to get (uu, vv)
    double det = cam.ocam.c - cam.ocam.d * cam.ocam.e;
    if (fabs(det) < 1e-10) det = 1e-10;
    
    double du = u - cam.ocam.cx;
    double dv = v - cam.ocam.cy;
    
    double uu = (cam.ocam.c * du - cam.ocam.e * dv) / det;
    double vv = (-cam.ocam.d * du + dv) / det;
    
    // 2. Compute theta from rho using world2cam polynomial
    double rho = sqrt(uu * uu + vv * vv);
    
    double theta = 0.0;
    for (int k = 0; k < cam.ocam.world2cam_coeffs.size(); k++)
        theta = theta * rho + cam.ocam.world2cam_coeffs(k);
    
    // 3. Compute 3D ray direction
    // theta is angle from optical axis (z)
    // For points in front of camera, theta < 0 in Ocam convention
    double norm = sqrt(uu * uu + vv * vv);
    if (norm < 1e-14) norm = 1e-14;
    
    // Ray direction (in camera frame, z forward)
    double factor = tan(-theta) / norm;  // -theta because Ocam uses negative for forward
    double xc = uu * factor;
    double yc = vv * factor;
    double zc = 1.0;
    
    Eigen::Vector3d ray_dir(xc, yc, zc);
    ray_dir.normalize();
    
    // 4. Intersect with wall plane
    Eigen::Matrix4d T_body_to_cam = cam.extrinsic;
    Eigen::Vector4d wall_pt_body;
    wall_pt_body << 0, 0, wall_distance, 1;
    Eigen::Vector4d wall_pt_cam = T_body_to_cam * wall_pt_body;
    
    Eigen::Vector4d wall_normal_body;
    wall_normal_body << 0, 0, -1, 0;
    Eigen::Vector4d wall_normal_cam = T_body_to_cam * wall_normal_body;
    
    double denom = ray_dir.dot(wall_normal_cam.head<3>());
    if (fabs(denom) < 1e-10) return false;
    
    double t = (wall_pt_cam.head<3>().dot(wall_normal_cam.head<3>())) / denom;
    if (t <= 0) return false;
    
    Eigen::Vector3d intersection_cam = t * ray_dir;
    
    Eigen::Vector4d intersection_cam_h;
    intersection_cam_h << intersection_cam, 1.0;
    Eigen::Vector4d intersection_body = T_body_to_cam.inverse() * intersection_cam_h;
    
    wall_x = intersection_body(1);
    wall_y = -intersection_body(2);
    
    return true;
}

// Project camera image to front wall
void projectCameraToWall(FrontCameraConfig &cam) {
    // Create output image
    cam.img_projected = Mat::zeros(output_height, output_width, CV_8UC3);
    
    // Wall coordinate ranges
    double wall_x_min = -wall_width / 2.0;
    double wall_x_max = wall_width / 2.0;
    double wall_y_min = -wall_height / 2.0;
    double wall_y_max = wall_height / 2.0;
    
    // For each pixel in output image, find corresponding camera pixel
    Mat map_x(output_height, output_width, CV_32FC1);
    Mat map_y(output_height, output_width, CV_32FC1);
    
    for (int v = 0; v < output_height; v++) {
        for (int u = 0; u < output_width; u++) {
            // Convert output pixel to wall coordinates
            double wall_x = wall_x_min + (u + 0.5) * meter_per_pixel;
            double wall_y = wall_y_max - (v + 0.5) * meter_per_pixel;  // Flip y
            
            // Transform wall point to camera coordinates
            Eigen::Vector4d wall_pt_body;
            wall_pt_body << wall_distance, wall_x, -wall_y, 1.0;  // x=forward, y=left, z=up
            
            Eigen::Matrix4d T_body_to_cam = cam.extrinsic;
            Eigen::Vector4d wall_pt_cam = T_body_to_cam * wall_pt_body;
            
            // Project to image
            double src_u, src_v;
            bool valid = false;
            
            if (cam.type == FISHEYE_OPENCV) {
                // Project using fisheye model
                double xc = wall_pt_cam(0);
                double yc = wall_pt_cam(1);
                double zc = wall_pt_cam(2);
                
                if (zc > 0) {  // In front of camera
                    // Normalize
                    double xn = xc / zc;
                    double yn = yc / zc;
                    
                    // Distort using fisheye model
                    Mat src_pt = (Mat_<double>(1, 2) << xn, yn);
                    Mat dst_pt;
                    
                    Mat K = cam.camera_matrix.clone();
                    Mat D = cam.dist_coeffs.clone();
                    
                    Mat src_f, dst_f;
                    src_pt.convertTo(src_f, CV_32FC2);
                    
                    // Use projectPoints to get distorted coordinates
                    vector<Point3d> obj_pts;
                    obj_pts.push_back(Point3d(xn, yn, 1.0));
                    vector<Point2d> img_pts;
                    
                    // Simple fisheye projection
                    double r = sqrt(xn * xn + yn * yn);
                    double theta = atan(r);
                    
                    double k1 = D.at<double>(0, 0);
                    double k2 = D.at<double>(0, 1);
                    double k3 = D.at<double>(0, 2);
                    double k4 = D.at<double>(0, 3);
                    
                    double theta2 = theta * theta;
                    double theta4 = theta2 * theta2;
                    double theta6 = theta4 * theta2;
                    double theta8 = theta6 * theta2;
                    
                    double theta_d = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
                    
                    if (r > 1e-10) {
                        double scale = theta_d / r;
                        double xd = xn * scale;
                        double yd = yn * scale;
                        
                        src_u = K.at<double>(0, 0) * xd + K.at<double>(0, 2);
                        src_v = K.at<double>(1, 1) * yd + K.at<double>(1, 2);
                        valid = true;
                    }
                }
            } else {
                // Project using Ocam model
                double xc = wall_pt_cam(0);
                double yc = wall_pt_cam(1);
                double zc = wall_pt_cam(2);
                
                if (zc > 0) {  // In front of camera
                    double norm = sqrt(xc * xc + yc * yc);
                    if (norm > 1e-14) {
                        double theta = atan(-zc / norm);  // Ocam convention
                        
                        double rho = 0.0;
                        for (int k = 0; k < cam.ocam.world2cam_coeffs.size(); k++)
                            rho = rho * theta + cam.ocam.world2cam_coeffs(k);
                        
                        double uu = xc / norm * rho;
                        double vv = yc / norm * rho;
                        
                        src_u = uu + vv * cam.ocam.e + cam.ocam.cx;
                        src_v = uu * cam.ocam.d + vv * cam.ocam.c + cam.ocam.cy;
                        valid = true;
                    }
                }
            }
            
            if (valid && src_u >= 0 && src_u < cam.width - 1 && 
                src_v >= 0 && src_v < cam.height - 1) {
                map_x.at<float>(v, u) = (float)src_u;
                map_y.at<float>(v, u) = (float)src_v;
            } else {
                map_x.at<float>(v, u) = -1.0f;
                map_y.at<float>(v, u) = -1.0f;
            }
        }
    }
    
    remap(cam.img_original, cam.img_projected, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
}

// Generate stitched view from front wall projections
Mat generateFrontStitchingView() {
    Mat stitched(output_height, output_width, CV_8UC3, Scalar(0, 0, 0));
    
    // Simple stitching with overlap blending
    // Divide output into 3 horizontal bands with overlap
    int x1_end = output_width * 0.35;
    int x2_start = output_width * 0.3;
    int x2_end = output_width * 0.7;
    int x3_start = output_width * 0.65;
    
    for (int y = 0; y < output_height; y++) {
        // Left part: only FL
        for (int x = 0; x < x2_start; x++) {
            stitched.at<Vec3b>(y, x) = cameras[0].img_projected.at<Vec3b>(y, x);
        }
        
        // Overlap 1: FL + F
        for (int x = x2_start; x < x1_end; x++) {
            float alpha = (float)(x - x2_start) / (x1_end - x2_start);
            stitched.at<Vec3b>(y, x) = (1 - alpha) * cameras[0].img_projected.at<Vec3b>(y, x)
                                     + alpha * cameras[1].img_projected.at<Vec3b>(y, x);
        }
        
        // Middle part: only F
        for (int x = x1_end; x < x3_start; x++) {
            stitched.at<Vec3b>(y, x) = cameras[1].img_projected.at<Vec3b>(y, x);
        }
        
        // Overlap 2: F + FR
        for (int x = x3_start; x < x2_end; x++) {
            float alpha = (float)(x - x3_start) / (x2_end - x3_start);
            stitched.at<Vec3b>(y, x) = (1 - alpha) * cameras[1].img_projected.at<Vec3b>(y, x)
                                     + alpha * cameras[2].img_projected.at<Vec3b>(y, x);
        }
        
        // Right part: only FR
        for (int x = x2_end; x < output_width; x++) {
            stitched.at<Vec3b>(y, x) = cameras[2].img_projected.at<Vec3b>(y, x);
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
        cout << "Usage: ./run_front3_calib_v2 <image_path> <intrinsic_path> <extrinsic_json>" << endl;
        cout << "\nExample:" << endl;
        cout << "  ./run_front3_calib_v2 ./imgs ./intrinsics ./extrinsics.json" << endl;
        return 0;
    }
    
    string image_path = argv[1];
    string intrinsic_path = argv[2];
    string extrinsic_file = argv[3];
    
    cout << "==========================================" << endl;
    cout << "Front 3 Cameras Manual Calibration (v2)" << endl;
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
    
    // Generate initial wall projections
    cout << "\nGenerating initial wall projections..." << endl;
    for (int i = 0; i < 3; i++) {
        projectCameraToWall(cameras[i]);
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
                
                // Recompute projection
                projectCameraToWall(cameras[cali_frame]);
                stitched = generateFrontStitchingView();
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            projectCameraToWall(cameras[cali_frame]);
            stitched = generateFrontStitchingView();
            cout << "Reset " << cameras[cali_frame].name << endl;
        }
        
        if (pangolin::Pushed(saveButton)) {
            for (int i = 0; i < 3; i++) saveResult(i);
            saveAllExtrinsics("front3_extrinsics_calibrated.json");
            imwrite("front3_stitched.png", stitched);
            cout << "Saved all calibrations" << endl;
        }
        
        // Keyboard input
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                projectCameraToWall(cameras[cali_frame]);
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
