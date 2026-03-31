/*
 * Front 3 Cameras Manual Calibration - Fixed Version
 * Fixes: Horizontal and vertical flip issues
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

enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1 };

struct FrontCameraConfig {
    string name, json_name, image_file;
    CameraType type;
    Mat camera_matrix, dist_coeffs;
    struct OcamParams {
        double cx, cy, c, d, e;
        vector<double> world2cam_poly;
        Eigen::VectorXd world2cam_coeffs;
        int width, height;
    } ocam;
    int width, height;
    Eigen::Matrix4d extrinsic, extrinsic_initial;
    Mat img_original, img_projected;
};

FrontCameraConfig cameras[3];
const string camera_names[3] = {"front_left", "front", "front_right"};
const string json_names[3] = {"front_left", "park_front", "front_right"};
const CameraType camera_types[3] = {FISHEYE_OPENCV, OCAM, FISHEYE_OPENCV};

int cali_frame = 0, view_mode = 0;
double cali_scale_degree_ = 0.3, cali_scale_trans_ = 0.06;
bool use_tail_effect = true;

// Wall parameters (meters)
double wall_distance = 8.0, wall_width = 12.0, wall_height = 6.0;
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

void loadExtrinsics(const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) { cerr << "Cannot open: " << filename << endl; exit(1); }
    Json::Value root; Json::Reader reader;
    if (!reader.parse(file, root)) { cerr << "Failed to parse: " << filename << endl; exit(1); }
    const Json::Value &extrinsics = root["extrinsic_param"];
    for (int i = 0; i < 3; i++) {
        if (!extrinsics.isMember(json_names[i])) { cerr << "Missing extrinsics for " << json_names[i] << endl; exit(1); }
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

// FIXED: Correct coordinate mapping to avoid horizontal and vertical flips
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
            // FIXED: Correct wall coordinate calculation
            // Image: u=0(left) -> u=width(right)
            // Body Y: points LEFT, so image left = body left (positive Y)
            //        image right = body right (negative Y)
            // Therefore: wall_x should decrease as u increases
            double wall_x = wall_x_max - u * meter_per_pixel - meter_per_pixel / 2;
            
            // Image: v=0(top) -> v=height(bottom)  
            // Body Z: points UP, so image top = body up (positive Z)
            //         image bottom = body down (negative Z)
            // Therefore: wall_y should decrease as v increases
            double wall_y = wall_y_max - v * meter_per_pixel - meter_per_pixel / 2;
            
            // Transform wall point to camera coordinates
            // Body frame: x=forward, y=left, z=up
            Eigen::Vector4d wall_pt_body;
            wall_pt_body << wall_distance, wall_x, wall_y, 1.0;
            
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
                    double k1 = D.at<double>(0, 0), k2 = D.at<double>(0, 1);
                    double k3 = D.at<double>(0, 2), k4 = D.at<double>(0, 3);
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
            
            if (valid && src_u >= 0 && src_u < cam.width - 1 && src_v >= 0 && src_v < cam.height - 1) {
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

bool isValidPixel(const Vec3b& pixel) {
    return pixel[0] > 10 || pixel[1] > 10 || pixel[2] > 10;
}

void applyTailMask(Mat& img, int cam_idx) {
    int width = img.cols;
    int height = img.rows;
    int tail_size = width * 0.4;
    Mat mask = Mat::zeros(height, width, CV_8UC1);
    
    if (cam_idx == 0) {  // FL - keep right side
        for (int x = 0; x < width; x++) {
            if (x >= width - tail_size) {
                mask.col(x).setTo(255);
            } else if (x >= width - tail_size * 1.5) {
                double alpha = (double)(x - (width - tail_size * 1.5)) / (tail_size * 0.5);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    } else if (cam_idx == 1) {  // F - keep center
        int center_start = (width - tail_size) / 2;
        int center_end = center_start + tail_size;
        for (int x = 0; x < width; x++) {
            if (x >= center_start && x <= center_end) {
                mask.col(x).setTo(255);
            } else if (x >= center_start - tail_size * 0.25 && x < center_start) {
                double alpha = (double)(x - (center_start - tail_size * 0.25)) / (tail_size * 0.25);
                mask.col(x).setTo((uchar)(alpha * 255));
            } else if (x > center_end && x <= center_end + tail_size * 0.25) {
                double alpha = 1.0 - (double)(x - center_end) / (tail_size * 0.25);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    } else if (cam_idx == 2) {  // FR - keep left side
        for (int x = 0; x < width; x++) {
            if (x < tail_size) {
                mask.col(x).setTo(255);
            } else if (x < tail_size * 1.5) {
                double alpha = 1.0 - (double)(x - tail_size) / (tail_size * 0.5);
                mask.col(x).setTo((uchar)(alpha * 255));
            }
        }
    }
    
    Mat masked_img;
    img.copyTo(masked_img, mask);
    img = masked_img;
}

Mat generateFrontStitchingView() {
    Mat stitched(output_height, output_width, CV_8UC3, Scalar(0, 0, 0));
    Mat img0 = cameras[0].img_projected.clone();
    Mat img1 = cameras[1].img_projected.clone();
    Mat img2 = cameras[2].img_projected.clone();
    
    if (use_tail_effect) {
        applyTailMask(img0, 0);
        applyTailMask(img1, 1);
        applyTailMask(img2, 2);
    }
    
    for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
            Vec3b pix0 = img0.at<Vec3b>(y, x);
            Vec3b pix1 = img1.at<Vec3b>(y, x);
            Vec3b pix2 = img2.at<Vec3b>(y, x);
            bool valid0 = isValidPixel(pix0);
            bool valid1 = isValidPixel(pix1);
            bool valid2 = isValidPixel(pix2);
            int valid_count = (valid0 ? 1 : 0) + (valid1 ? 1 : 0) + (valid2 ? 1 : 0);
            
            if (valid_count == 0) continue;
            else if (valid_count == 1) {
                if (valid0) stitched.at<Vec3b>(y, x) = pix0;
                else if (valid1) stitched.at<Vec3b>(y, x) = pix1;
                else stitched.at<Vec3b>(y, x) = pix2;
            } else if (valid_count == 2) {
                if (valid0 && valid1) stitched.at<Vec3b>(y, x) = 0.5 * pix0 + 0.5 * pix1;
                else if (valid1 && valid2) stitched.at<Vec3b>(y, x) = 0.5 * pix1 + 0.5 * pix2;
                else stitched.at<Vec3b>(y, x) = 0.5 * pix0 + 0.5 * pix2;
            } else {
                stitched.at<Vec3b>(y, x) = 0.25 * pix0 + 0.5 * pix1 + 0.25 * pix2;
            }
        }
    }
    
    putText(stitched, "FL", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 255, 0), 2);
    putText(stitched, "F", Point(output_width/2 - 20, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(255, 0, 0), 2);
    putText(stitched, "FR", Point(output_width - 100, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255), 2);
    return stitched;
}

Mat getDisplayImage() {
    if (view_mode == 0) return generateFrontStitchingView();
    else {
        Mat img = cameras[view_mode - 1].img_projected.clone();
        string labels[] = {"FRONT LEFT (FL)", "FRONT (F)", "FRONT RIGHT (FR)"};
        Scalar colors[] = {Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(0, 0, 255)};
        int baseline;
        Size text_size = getTextSize(labels[view_mode-1], FONT_HERSHEY_SIMPLEX, 1.2, 2, &baseline);
        rectangle(img, Point(10, 10), Point(10 + text_size.width + 20, 10 + text_size.height + 20), Scalar(0, 0, 0), -1);
        putText(img, labels[view_mode-1], Point(20, 45), FONT_HERSHEY_SIMPLEX, 1.2, colors[view_mode-1], 2);
        return img;
    }
}

void updateAllProjections() {
    for (int i = 0; i < 3; i++) projectCameraToWall(cameras[i]);
}

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
        cout << "Usage: ./run_front3_calib_fixed <image_path> <intrinsic_path> <extrinsic_json>" << endl;
        return 0;
    }
    
    string image_path = argv[1], intrinsic_path = argv[2], extrinsic_file = argv[3];
    cout << "Front 3 Cameras Manual Calibration (FIXED)" << endl;
    
    for (int i = 0; i < 3; i++) {
        cameras[i].name = camera_names[i];
        cameras[i].json_name = json_names[i];
        cameras[i].type = camera_types[i];
        cameras[i].image_file = image_path + "/" + camera_names[i] + ".png";
    }
    
    for (int i = 0; i < 3; i++) {
        cameras[i].img_original = imread(cameras[i].image_file);
        if (cameras[i].img_original.empty()) { cerr << "Failed to load: " << cameras[i].image_file << endl; return 1; }
    }
    
    loadFisheyeIntrinsic(cameras[0], intrinsic_path + "/front_left.json");
    loadOcamIntrinsic(cameras[1], intrinsic_path + "/park_front.json");
    loadFisheyeIntrinsic(cameras[2], intrinsic_path + "/front_right.json");
    loadExtrinsics(extrinsic_file);
    CalibrationInit();
    updateAllProjections();
    
    Mat display_image = getDisplayImage();
    
    int width = 1200, height = 600;
    pangolin::CreateWindowAndBind("Front 3 Cameras Calibration - FIXED", width, height);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::View &project_image = pangolin::Display("project")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0, 1.0 * width / height)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    unsigned char *imageArray = new unsigned char[3 * output_width * output_height];
    pangolin::GlTexture imageTexture(output_width, output_height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(0), 1.0, 0.0, pangolin::Attach::Pix(150));
    pangolin::Var<int> frameStep("cp.frame", 0, 0, 2);
    pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 5);
    pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 200);
    pangolin::Var<double> wallDist("cp.wall dist(m)", wall_distance, 1, 20);
    
    pangolin::Var<bool> viewStitching("cp.View Stitch", true, true);
    pangolin::Var<bool> viewFL("cp.View FL", false, true);
    pangolin::Var<bool> viewF("cp.View F", false, true);
    pangolin::Var<bool> viewFR("cp.View FR", false, true);
    pangolin::Var<bool> toggleTail("cp.Toggle Tail", true, true);
    
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
        
        if (pangolin::Pushed(viewStitching)) {
            view_mode = 0; viewFL = false; viewF = false; viewFR = false;
            display_image = getDisplayImage();
        }
        if (pangolin::Pushed(viewFL)) {
            view_mode = 1; viewStitching = false; viewF = false; viewFR = false;
            display_image = getDisplayImage();
        }
        if (pangolin::Pushed(viewF)) {
            view_mode = 2; viewStitching = false; viewFL = false; viewFR = false;
            display_image = getDisplayImage();
        }
        if (pangolin::Pushed(viewFR)) {
            view_mode = 3; viewStitching = false; viewFL = false; viewF = false;
            display_image = getDisplayImage();
        }
        if (pangolin::Pushed(toggleTail)) {
            use_tail_effect = !use_tail_effect;
            if (view_mode == 0) display_image = getDisplayImage();
        }
        
        if (frameStep.GuiChanged()) {
            cali_frame = frameStep.Get();
            CalibrationScaleChange(cali_frame);
        }
        if (degreeStep.GuiChanged()) {
            cali_scale_degree_ = degreeStep.Get();
            CalibrationScaleChange(cali_frame);
        }
        if (tStep.GuiChanged()) {
            cali_scale_trans_ = tStep.Get() / 100.0;
            CalibrationScaleChange(cali_frame);
        }
        if (wallDist.GuiChanged()) {
            wall_distance = wallDist.Get();
            updateAllProjections();
            display_image = getDisplayImage();
        }
        
        for (int i = 0; i < 12; i++) {
            if (pangolin::Pushed(mat_calib_box[i])) {
                cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic * modification_list_[cali_frame][i];
                projectCameraToWall(cameras[cali_frame]);
                display_image = getDisplayImage();
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            projectCameraToWall(cameras[cali_frame]);
            display_image = getDisplayImage();
        }
        
        if (pangolin::Pushed(saveButton)) {
            imwrite("front3_stitched.png", generateFrontStitchingView());
        }
        
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                projectCameraToWall(cameras[cali_frame]);
                display_image = getDisplayImage();
            }
        }
        
        imageArray = display_image.data;
        imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);
        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        // FIXED: Use FlipY to correct OpenGL coordinate system
        imageTexture.RenderToViewportFlipY();
        
        pangolin::FinishFrame();
        glFinish();
    }
    
    delete[] imageArray;
    return 0;
}
