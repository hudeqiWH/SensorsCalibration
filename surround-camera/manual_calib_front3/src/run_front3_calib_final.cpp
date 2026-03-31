/*
 * Front 3 Cameras Manual Calibration - Final Version
 * FL(Fisheye) + F(Ocam) + FR(Fisheye) with Front Wall Projection
 * 
 * Features:
 * - Fixed image flip (horizontal mirror)
 * - View mode toggle (Stitching / Single Camera)
 * - Adjustable wall distance for better projection
 * - Real-time extrinsic calibration
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
int view_mode = 0;  // 0=stitching, 1-3=single camera
bool use_tail_effect = true;  // Enable tail effect by default

// Front wall projection parameters (单位: 米)
double wall_distance = 8.0;       // 墙面距离相机
double wall_width = 12.0;         // 墙面宽度  
double wall_height = 6.0;         // 墙面高度
double wall_center_y = 0.0;       // 墙面中心高度

// Output image parameters
int output_width = 2400, output_height = 1200;
double meter_per_pixel = wall_width / output_width;

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

// Project camera image to front wall
void projectCameraToWall(FrontCameraConfig &cam) {
    cam.img_projected = Mat::zeros(output_height, output_width, CV_8UC3);
    
    double wall_x_min = -wall_width / 2.0;
    double wall_x_max = wall_width / 2.0;
    double wall_y_min = -wall_height / 2.0;
    double wall_y_max = wall_height / 2.0;
    
    Mat map_x(output_height, output_width, CV_32FC1);
    Mat map_y(output_height, output_width, CV_32FC1);
    
    for (int v = 0; v < output_height; v++) {
        for (int u = 0; u < output_width; u++) {
            // Convert output pixel to wall coordinates
            // wall_x: left(-) to right(+), maps to output u: 0(left) to width(right)
            // wall_y: down(-) to up(+), maps to output v: 0(top) to height(bottom)
            double wall_x = wall_x_min + u * meter_per_pixel + meter_per_pixel / 2;
            double wall_y = wall_y_max - v * meter_per_pixel - meter_per_pixel / 2;
            
            // Transform wall point to camera coordinates
            Eigen::Vector4d wall_pt_body;
            wall_pt_body << wall_distance, wall_x, -wall_y, 1.0;
            
            Eigen::Matrix4d T_body_to_cam = cam.extrinsic;
            Eigen::Vector4d wall_pt_cam = T_body_to_cam * wall_pt_body;
            
            double src_u, src_v;
            bool valid = false;
            
            if (cam.type == FISHEYE_OPENCV) {
                double xc = wall_pt_cam(0);
                double yc = wall_pt_cam(1);
                double zc = wall_pt_cam(2);
                
                if (zc > 0) {
                    double xn = xc / zc;
                    double yn = yc / zc;
                    
                    Mat K = cam.camera_matrix.clone();
                    Mat D = cam.dist_coeffs.clone();
                    
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
                double xc = wall_pt_cam(0);
                double yc = wall_pt_cam(1);
                double zc = wall_pt_cam(2);
                
                if (zc > 0) {
                    double norm = sqrt(xc * xc + yc * yc);
                    if (norm > 1e-14) {
                        double theta = atan(-zc / norm);
                        
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

// Check if pixel is valid (not black/empty)
bool isValidPixel(const Vec3b& pixel) {
    return pixel[0] > 10 || pixel[1] > 10 || pixel[2] > 10;
}

// Apply tail mask to projected image - keeps only the center region of each camera
// This creates the "bowtie" effect like in the reference image
void applyTailMask(Mat& img, int cam_idx) {
    int width = img.cols;
    int height = img.rows;
    
    // Define tail sizes - how much to keep from each camera
    // FL: keep right side (overlaps with F on the right)
    // F: keep center (overlaps with both FL and FR)
    // FR: keep left side (overlaps with F on the left)
    int tail_size = width * 0.4;  // Keep 40% of each camera's width
    
    Mat mask = Mat::zeros(height, width, CV_8UC1);
    
    if (cam_idx == 0) {  // FL - keep right side
        // Create gradient mask: full on right, fading to left
        for (int x = 0; x < width; x++) {
            if (x >= width - tail_size) {
                // Rightmost 40%: full opacity
                mask.col(x).setTo(255);
            } else if (x >= width - tail_size * 1.5) {
                // Fade region
                double alpha = (double)(x - (width - tail_size * 1.5)) / (tail_size * 0.5);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    } else if (cam_idx == 1) {  // F - keep center
        int center_start = (width - tail_size) / 2;
        int center_end = center_start + tail_size;
        
        for (int x = 0; x < width; x++) {
            if (x >= center_start && x <= center_end) {
                // Center region: full opacity
                mask.col(x).setTo(255);
            } else if (x >= center_start - tail_size * 0.25 && x < center_start) {
                // Left fade
                double alpha = (double)(x - (center_start - tail_size * 0.25)) / (tail_size * 0.25);
                mask.col(x).setTo((uchar)(alpha * 255));
            } else if (x > center_end && x <= center_end + tail_size * 0.25) {
                // Right fade
                double alpha = 1.0 - (double)(x - center_end) / (tail_size * 0.25);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    } else if (cam_idx == 2) {  // FR - keep left side
        for (int x = 0; x < width; x++) {
            if (x < tail_size) {
                // Leftmost 40%: full opacity
                mask.col(x).setTo(255);
            } else if (x < tail_size * 1.5) {
                // Fade region
                double alpha = 1.0 - (double)(x - tail_size) / (tail_size * 0.5);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    }
    
    // Apply mask to image
    Mat masked_img;
    img.copyTo(masked_img, mask);
    img = masked_img;
}

// Generate stitched view with optional tail effect (bowtie shape)
Mat generateFrontStitchingView() {
    Mat stitched(output_height, output_width, CV_8UC3, Scalar(0, 0, 0));
    
    // Create temporary copies
    Mat img0 = cameras[0].img_projected.clone();
    Mat img1 = cameras[1].img_projected.clone();
    Mat img2 = cameras[2].img_projected.clone();
    
    // Apply tail masks if enabled
    if (use_tail_effect) {
        applyTailMask(img0, 0);  // FL
        applyTailMask(img1, 1);  // F
        applyTailMask(img2, 2);  // FR
    }
    
    // Blend the images
    for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
            Vec3b pix0 = img0.at<Vec3b>(y, x);
            Vec3b pix1 = img1.at<Vec3b>(y, x);
            Vec3b pix2 = img2.at<Vec3b>(y, x);
            
            bool valid0 = isValidPixel(pix0);
            bool valid1 = isValidPixel(pix1);
            bool valid2 = isValidPixel(pix2);
            
            // Count valid cameras at this pixel
            int valid_count = (valid0 ? 1 : 0) + (valid1 ? 1 : 0) + (valid2 ? 1 : 0);
            
            if (valid_count == 0) {
                // No valid pixels, keep black
                continue;
            } else if (valid_count == 1) {
                // Only one camera valid
                if (valid0) stitched.at<Vec3b>(y, x) = pix0;
                else if (valid1) stitched.at<Vec3b>(y, x) = pix1;
                else stitched.at<Vec3b>(y, x) = pix2;
            } else if (valid_count == 2) {
                // Two cameras overlap - average them
                if (valid0 && valid1) {
                    stitched.at<Vec3b>(y, x) = 0.5 * pix0 + 0.5 * pix1;
                } else if (valid1 && valid2) {
                    stitched.at<Vec3b>(y, x) = 0.5 * pix1 + 0.5 * pix2;
                } else {
                    stitched.at<Vec3b>(y, x) = 0.5 * pix0 + 0.5 * pix2;
                }
            } else {
                // All three valid - use weighted average (prefer center)
                stitched.at<Vec3b>(y, x) = 0.25 * pix0 + 0.5 * pix1 + 0.25 * pix2;
            }
        }
    }
    
    // Add text labels
    putText(stitched, "FL", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 255, 0), 2);
    putText(stitched, "F", Point(output_width/2 - 20, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(255, 0, 0), 2);
    putText(stitched, "FR", Point(output_width - 100, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255), 2);
    
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

void rotationMatrixToRodrigues(const Eigen::Matrix3d &R, Eigen::Vector3d &rvec) {
    double theta = acos((R.trace() - 1.0) / 2.0);
    if (theta < 1e-8) { rvec = Eigen::Vector3d::Zero(); }
    else {
        rvec << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        rvec *= theta / (2.0 * sin(theta));
    }
}

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

Mat getDisplayImage() {
    if (view_mode == 0) {
        return generateFrontStitchingView();
    } else {
        Mat img = cameras[view_mode - 1].img_projected.clone();
        // Add camera name overlay
        string camera_label;
        Scalar label_color;
        if (view_mode == 1) {
            camera_label = "FRONT LEFT (FL)";
            label_color = Scalar(0, 255, 0); // Green
        } else if (view_mode == 2) {
            camera_label = "FRONT (F)";
            label_color = Scalar(255, 0, 0); // Blue
        } else {
            camera_label = "FRONT RIGHT (FR)";
            label_color = Scalar(0, 0, 255); // Red
        }
        
        // Draw background rectangle for text
        int baseline;
        Size text_size = getTextSize(camera_label, FONT_HERSHEY_SIMPLEX, 1.2, 2, &baseline);
        rectangle(img, Point(10, 10), Point(10 + text_size.width + 20, 10 + text_size.height + 20), Scalar(0, 0, 0), -1);
        putText(img, camera_label, Point(20, 45), FONT_HERSHEY_SIMPLEX, 1.2, label_color, 2);
        
        return img;
    }
}

void updateAllProjections() {
    for (int i = 0; i < 3; i++) {
        projectCameraToWall(cameras[i]);
    }
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cout << "Usage: ./run_front3_calib_final <image_path> <intrinsic_path> <extrinsic_json>" << endl;
        cout << "\nExample:" << endl;
        cout << "  ./run_front3_calib_final ./imgs ./intrinsics ./extrinsics.json" << endl;
        cout << "\nControls:" << endl;
        cout << "  - View buttons: Switch between Stitching/Single camera views" << endl;
        cout << "  - Wall Dist slider: Adjust wall distance for better projection" << endl;
        cout << "  - 6DOF buttons: Adjust extrinsic parameters" << endl;
        cout << "  - Reset: Reset current camera to initial extrinsics" << endl;
        cout << "  - Save: Save all calibrations" << endl;
        cout << "\nNote: If projection looks skewed, adjust Yaw (z deg) and Pitch (y deg)" << endl;
        return 0;
    }
    
    string image_path = argv[1];
    string intrinsic_path = argv[2];
    string extrinsic_file = argv[3];
    
    cout << "==========================================" << endl;
    cout << "Front 3 Cameras Manual Calibration (Final)" << endl;
    cout << "==========================================" << endl;
    
    for (int i = 0; i < 3; i++) {
        cameras[i].name = camera_names[i];
        cameras[i].json_name = json_names[i];
        cameras[i].type = camera_types[i];
        cameras[i].image_file = image_path + "/" + camera_names[i] + ".png";
    }
    
    cout << "\nLoading images..." << endl;
    for (int i = 0; i < 3; i++) {
        cameras[i].img_original = imread(cameras[i].image_file);
        if (cameras[i].img_original.empty()) {
            cerr << "Failed to load: " << cameras[i].image_file << endl;
            return 1;
        }
        cout << "  " << cameras[i].name << ": " << cameras[i].img_original.cols << "x" << cameras[i].img_original.rows << endl;
    }
    
    cout << "\nLoading intrinsics..." << endl;
    loadFisheyeIntrinsic(cameras[0], intrinsic_path + "/front_left.json");
    loadOcamIntrinsic(cameras[1], intrinsic_path + "/park_front.json");
    loadFisheyeIntrinsic(cameras[2], intrinsic_path + "/front_right.json");
    
    cout << "\nLoading extrinsics..." << endl;
    loadExtrinsics(extrinsic_file);
    
    CalibrationInit();
    
    cout << "\nGenerating wall projections..." << endl;
    updateAllProjections();
    cout << "  Done" << endl;
    
    Mat display_image = getDisplayImage();
    
    cout << "\n==========================================" << endl;
    cout << "Starting GUI..." << endl;
    cout << "==========================================" << endl;
    
    int width = 1200, height = 600;
    pangolin::CreateWindowAndBind("Front 3 Cameras Calibration - Final", width, height);
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
    pangolin::Var<double> wallDist("cp.wall dist(m)", wall_distance, 1, 20);
    
    // View mode buttons
    pangolin::Var<bool> viewStitching("cp.View Stitch", true, true);
    pangolin::Var<bool> viewFL("cp.View FL", false, true);
    pangolin::Var<bool> viewF("cp.View F", false, true);
    pangolin::Var<bool> viewFR("cp.View FR", false, true);
    pangolin::Var<bool> toggleTail("cp.Toggle Tail", true, true);
    
    // Calibration buttons
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
        
        // Handle view mode changes
        if (pangolin::Pushed(viewStitching)) {
            view_mode = 0;
            viewFL = false; viewF = false; viewFR = false;
            display_image = getDisplayImage();
            cout << "\n========================================" << endl;
            cout << "  [STITCHING VIEW]" << endl;
            cout << "  Showing all 3 cameras with overlap regions" << endl;
            cout << "  Yellow lines = camera boundaries" << endl;
            cout << "  FL(green) | F(blue) | FR(red)" << endl;
            cout << "========================================" << endl;
        }
        if (pangolin::Pushed(viewFL)) {
            view_mode = 1;
            viewStitching = false; viewF = false; viewFR = false;
            display_image = getDisplayImage();
            cout << "\n========================================" << endl;
            cout << "  [FRONT LEFT CAMERA VIEW]" << endl;
            cout << "  Calibrating: " << cameras[0].name << endl;
            cout << "========================================" << endl;
        }
        if (pangolin::Pushed(viewF)) {
            view_mode = 2;
            viewStitching = false; viewFL = false; viewFR = false;
            display_image = getDisplayImage();
            cout << "\n========================================" << endl;
            cout << "  [FRONT CENTER CAMERA VIEW]" << endl;
            cout << "  Calibrating: " << cameras[1].name << endl;
            cout << "========================================" << endl;
        }
        if (pangolin::Pushed(viewFR)) {
            view_mode = 3;
            viewStitching = false; viewFL = false; viewF = false;
            display_image = getDisplayImage();
            cout << "\n========================================" << endl;
            cout << "  [FRONT RIGHT CAMERA VIEW]" << endl;
            cout << "  Calibrating: " << cameras[2].name << endl;
            cout << "========================================" << endl;
        }
        
        // Handle tail effect toggle
        if (pangolin::Pushed(toggleTail)) {
            use_tail_effect = !use_tail_effect;
            if (view_mode == 0) {
                display_image = getDisplayImage();
            }
            cout << "\n========================================" << endl;
            cout << "  Tail Effect: " << (use_tail_effect ? "ON" : "OFF") << endl;
            if (use_tail_effect) {
                cout << "  Showing bowtie-shaped camera overlap" << endl;
            } else {
                cout << "  Showing full camera projection" << endl;
            }
            cout << "========================================" << endl;
        }
        
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
        
        if (wallDist.GuiChanged()) {
            wall_distance = wallDist.Get();
            updateAllProjections();
            display_image = getDisplayImage();
            cout << "Wall distance: " << wall_distance << " m" << endl;
        }
        
        // Handle calibration buttons
        for (int i = 0; i < 12; i++) {
            if (pangolin::Pushed(mat_calib_box[i])) {
                cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic * modification_list_[cali_frame][i];
                cout << "Calibrated " << cameras[cali_frame].name << endl;
                projectCameraToWall(cameras[cali_frame]);
                display_image = getDisplayImage();
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            projectCameraToWall(cameras[cali_frame]);
            display_image = getDisplayImage();
            cout << "Reset " << cameras[cali_frame].name << endl;
        }
        
        if (pangolin::Pushed(saveButton)) {
            for (int i = 0; i < 3; i++) saveResult(i);
            saveAllExtrinsics("front3_extrinsics_calibrated.json");
            imwrite("front3_stitched.png", generateFrontStitchingView());
            cout << "Saved all calibrations" << endl;
        }
        
        // Keyboard input
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                projectCameraToWall(cameras[cali_frame]);
                display_image = getDisplayImage();
                cout << "Key calibration applied to " << cameras[cali_frame].name << endl;
            }
        }
        
        // Update display - use RenderToViewport (no flip)
        imageArray = display_image.data;
        imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);
        
        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewport();
        
        pangolin::FinishFrame();
        glFinish();
    }
    
    delete[] imageArray;
    return 0;
}
