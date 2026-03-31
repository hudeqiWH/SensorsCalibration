/*
 * Front 3 Cameras Manual Calibration - V4
 * FL(Fisheye) + F(Ocam) + FR(Fisheye) with Front Wall Projection
 * 
 * New Features:
 * - Multiple stitching modes: FL+FR, FL+F, F+FR, FL+F+FR
 * - Single camera views: FL, F, FR
 * venv) nio@PC1Y47SH:/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/SensorsCalibration/surround-camera/manual_calib_front3$ cd build/ && make -j4 && cd .. && ./bin/run_front3_calib_v4 ./2_dog/imgs/ ./2_dog/param/ ./2_dog/param/camera_extrinsics_ikalibr.json5 
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
#include <sstream>
#include <json/json.h>
#include <fstream>

using namespace std;
using namespace cv;

// Camera types
enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1 };

// View modes
// 0=FL+F+FR (all), 1=FL, 2=F, 3=FR, 4=FL+FR, 5=FL+F, 6=F+FR
enum ViewMode { VIEW_ALL = 0, VIEW_FL = 1, VIEW_F = 2, VIEW_FR = 3, 
                VIEW_FL_FR = 4, VIEW_FL_F = 5, VIEW_F_FR = 6 };

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
double cali_scale_degree_ = 0.5;   // Default rotation step (degrees)
double cali_scale_trans_ = 0.05;   // Default translation step (meters = 5cm)
int view_mode = 0;  // 0=ALL, 1=FL, 2=F, 3=FR, 4=FL+FR, 5=FL+F, 6=F+FR

// Front wall projection parameters (单位: 米)
double wall_distance = 3.0;       // Default changed to 3m
double wall_width = 12.0;
double wall_height = 6.0;
double wall_center_y = 0.0;

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

// Optimized projection using parallel processing
class ParallelProjector : public cv::ParallelLoopBody {
public:
    ParallelProjector(FrontCameraConfig& cam, Mat& map_x, Mat& map_y)
        : cam_(cam), map_x_(map_x), map_y_(map_y) {
        // Pre-compute constants
        wall_x_max_ = wall_width / 2.0;
        wall_y_max_ = wall_height / 2.0;
        T_body_to_cam_ = cam.extrinsic;
        
        // Pre-extract camera parameters for fisheye
        if (cam.type == FISHEYE_OPENCV) {
            K_ = cam.camera_matrix;
            D_ = cam.dist_coeffs;
            fx_ = K_.at<double>(0, 0);
            fy_ = K_.at<double>(1, 1);
            cx_ = K_.at<double>(0, 2);
            cy_ = K_.at<double>(1, 2);
            k1_ = D_.at<double>(0, 0);
            k2_ = D_.at<double>(0, 1);
            k3_ = D_.at<double>(0, 2);
            k4_ = D_.at<double>(0, 3);
        }
    }

    void operator()(const cv::Range& range) const override {
        for (int v = range.start; v < range.end; v++) {
            float* map_x_row = map_x_.ptr<float>(v);
            float* map_y_row = map_y_.ptr<float>(v);
            
            // Pre-compute wall_y for this row
            double wall_y = wall_y_max_ - v * meter_per_pixel - meter_per_pixel / 2;
            
            for (int u = 0; u < output_width; u++) {
                double wall_x = wall_x_max_ - u * meter_per_pixel - meter_per_pixel / 2;
                
                // Transform wall point to camera coordinates
                Eigen::Vector4d wall_pt_body;
                wall_pt_body << wall_distance, wall_x, wall_y, 1.0;
                Eigen::Vector4d wall_pt_cam = T_body_to_cam_ * wall_pt_body;
                
                double src_u, src_v;
                bool valid = false;
                
                if (cam_.type == FISHEYE_OPENCV) {
                    double xc = wall_pt_cam(0);
                    double yc = wall_pt_cam(1);
                    double zc = wall_pt_cam(2);
                    
                    if (zc > 0) {
                        double xn = xc / zc;
                        double yn = yc / zc;
                        double r = sqrt(xn * xn + yn * yn);
                        
                        if (r > 1e-10) {
                            double theta = atan(r);
                            double theta2 = theta * theta;
                            double theta4 = theta2 * theta2;
                            double theta6 = theta4 * theta2;
                            double theta8 = theta6 * theta2;
                            double theta_d = theta * (1 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
                            
                            double scale = theta_d / r;
                            src_u = fx_ * xn * scale + cx_;
                            src_v = fy_ * yn * scale + cy_;
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
                            for (int k = 0; k < cam_.ocam.world2cam_coeffs.size(); k++)
                                rho = rho * theta + cam_.ocam.world2cam_coeffs(k);
                            
                            double uu = xc / norm * rho;
                            double vv = yc / norm * rho;
                            src_u = uu + vv * cam_.ocam.e + cam_.ocam.cx;
                            src_v = uu * cam_.ocam.d + vv * cam_.ocam.c + cam_.ocam.cy;
                            valid = true;
                        }
                    }
                }
                
                if (valid && src_u >= 0 && src_u < cam_.width - 1 && 
                    src_v >= 0 && src_v < cam_.height - 1) {
                    map_x_row[u] = (float)src_u;
                    map_y_row[u] = (float)src_v;
                } else {
                    map_x_row[u] = -1.0f;
                    map_y_row[u] = -1.0f;
                }
            }
        }
    }

private:
    FrontCameraConfig& cam_;
    Mat& map_x_;
    Mat& map_y_;
    double wall_x_max_;
    double wall_y_max_;
    Eigen::Matrix4d T_body_to_cam_;
    
    // Fisheye cached params
    Mat K_, D_;
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, k3_, k4_;
};

void projectCameraToWall(FrontCameraConfig &cam) {
    cam.img_projected = Mat::zeros(output_height, output_width, CV_8UC3);
    
    Mat map_x(output_height, output_width, CV_32FC1);
    Mat map_y(output_height, output_width, CV_32FC1);
    
    // Use parallel processing for map computation
    ParallelProjector projector(cam, map_x, map_y);
    cv::parallel_for_(cv::Range(0, output_height), projector);
    
    remap(cam.img_original, cam.img_projected, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
}

bool isValidPixel(const Vec3b& pixel) {
    return pixel[0] > 10 || pixel[1] > 10 || pixel[2] > 10;
}

// Generate stitched view from selected cameras
// cam_mask: bitmask of which cameras to include (bit 0=FL, bit 1=F, bit 2=FR)
Mat generateStitchedView(int cam_mask) {
    Mat stitched(output_height, output_width, CV_8UC3, Scalar(0, 0, 0));
    
    Mat img0 = cameras[0].img_projected.clone();
    Mat img1 = cameras[1].img_projected.clone();
    Mat img2 = cameras[2].img_projected.clone();
    
    // Blend the images
    for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
            Vec3b pix0 = img0.at<Vec3b>(y, x);
            Vec3b pix1 = img1.at<Vec3b>(y, x);
            Vec3b pix2 = img2.at<Vec3b>(y, x);
            
            bool valid0 = (cam_mask & 1) && isValidPixel(pix0);
            bool valid1 = (cam_mask & 2) && isValidPixel(pix1);
            bool valid2 = (cam_mask & 4) && isValidPixel(pix2);
            
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
    
    // Add text labels based on which cameras are shown
    if (cam_mask & 1) putText(stitched, "FL", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 255, 0), 2);
    if (cam_mask & 2) putText(stitched, "F", Point(output_width/2 - 20, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(255, 0, 0), 2);
    if (cam_mask & 4) putText(stitched, "FR", Point(output_width - 100, 50), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255), 2);
    
    return stitched;
}

Mat getDisplayImage() {
    switch (view_mode) {
        case VIEW_ALL:      return generateStitchedView(0b111);  // FL+F+FR
        case VIEW_FL:       return cameras[0].img_projected.clone();
        case VIEW_F:        return cameras[1].img_projected.clone();
        case VIEW_FR:       return cameras[2].img_projected.clone();
        case VIEW_FL_FR:    return generateStitchedView(0b101);  // FL+FR
        case VIEW_FL_F:     return generateStitchedView(0b011);  // FL+F
        case VIEW_F_FR:     return generateStitchedView(0b110);  // F+FR
        default:            return generateStitchedView(0b111);
    }
}

void addOverlayToSingleView(Mat& img, int cam_idx) {
    string labels[] = {"FRONT LEFT (FL)", "FRONT CENTER (F)", "FRONT RIGHT (FR)"};
    Scalar colors[] = {Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(0, 0, 255)};
    
    int baseline;
    Size text_size = getTextSize(labels[cam_idx], FONT_HERSHEY_SIMPLEX, 1.2, 2, &baseline);
    rectangle(img, Point(10, 10), Point(10 + text_size.width + 20, 10 + text_size.height + 20), Scalar(0, 0, 0), -1);
    putText(img, labels[cam_idx], Point(20, 45), FONT_HERSHEY_SIMPLEX, 1.2, colors[cam_idx], 2);
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

void updateAllProjections() {
    for (int i = 0; i < 3; i++) projectCameraToWall(cameras[i]);
}

// Update projections based on current view mode
// This ensures that when calibrating in a 2-camera stitch mode, both cameras get updated
void updateProjectionsForViewMode() {
    switch (view_mode) {
        case VIEW_ALL:      // FL+F+FR
            updateAllProjections();
            break;
        case VIEW_FL:       // Single FL
            projectCameraToWall(cameras[0]);
            break;
        case VIEW_F:        // Single F
            projectCameraToWall(cameras[1]);
            break;
        case VIEW_FR:       // Single FR
            projectCameraToWall(cameras[2]);
            break;
        case VIEW_FL_FR:    // FL+FR
            projectCameraToWall(cameras[0]);
            projectCameraToWall(cameras[2]);
            break;
        case VIEW_FL_F:     // FL+F
            projectCameraToWall(cameras[0]);
            projectCameraToWall(cameras[1]);
            break;
        case VIEW_F_FR:     // F+FR
            projectCameraToWall(cameras[1]);
            projectCameraToWall(cameras[2]);
            break;
    }
}

string getViewModeName() {
    switch (view_mode) {
        case VIEW_ALL:      return "ALL (FL+F+FR)";
        case VIEW_FL:       return "FRONT LEFT (FL)";
        case VIEW_F:        return "FRONT CENTER (F)";
        case VIEW_FR:       return "FRONT RIGHT (FR)";
        case VIEW_FL_FR:    return "FL+FR (Left+Right)";
        case VIEW_FL_F:     return "FL+F (Left+Center)";
        case VIEW_F_FR:     return "F+FR (Center+Right)";
        default:            return "UNKNOWN";
    }
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cout << "Usage: ./run_front3_calib_v4 <image_path> <intrinsic_path> <extrinsic_json>" << endl;
        cout << "\nExample:" << endl;
        cout << "  ./run_front3_calib_v4 ./imgs ./intrinsics ./extrinsics.json" << endl;
        cout << "\nView Modes:" << endl;
        cout << "  - FL+F+FR: All three cameras (default)" << endl;
        cout << "  - FL+FR: Left and right side cameras only" << endl;
        cout << "  - FL+F: Left and center cameras" << endl;
        cout << "  - F+FR: Center and right cameras" << endl;
        cout << "  - FL/F/FR: Single camera view" << endl;
        return 0;
    }
    
    string image_path = argv[1];
    string intrinsic_path = argv[2];
    string extrinsic_file = argv[3];
    
    cout << "==========================================" << endl;
    cout << "Front 3 Cameras Manual Calibration (V4)" << endl;
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
    
    int width = 1500, height = 700;
    pangolin::CreateWindowAndBind("Front 3 Cameras Calibration - V4", width, height);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::View &project_image = pangolin::Display("project")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(280), 1.0, 1.0 * width / height)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    unsigned char *imageArray = new unsigned char[3 * output_width * output_height];
    pangolin::GlTexture imageTexture(output_width, output_height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    
    // Main control panel (left column)
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(0), 1.0, 0.0, pangolin::Attach::Pix(140));
    
    // Camera selector using push buttons
    pangolin::Var<bool> btnSelectFL("cp.Cam: FL", false, false);
    pangolin::Var<bool> btnSelectF("cp.Cam: F", false, false);
    pangolin::Var<bool> btnSelectFR("cp.Cam: FR", false, false);
    pangolin::Var<string> currentCamera("cp.Current", "FL");
    
    pangolin::Var<double> degreeStep("cp.deg step", 0.5, 0.1, 5);
    pangolin::Var<double> tStep("cp.t(cm)", 5, 0, 100);
    pangolin::Var<double> wallDist("cp.wall(m)", wall_distance, 1, 20);
    
    // View mode buttons
    pangolin::Var<bool> btnViewAll("cp.View: All", false, false);
    pangolin::Var<bool> btnViewFL_FR("cp.View: FL+FR", false, false);
    pangolin::Var<bool> btnViewFL_F("cp.View: FL+F", false, false);
    pangolin::Var<bool> btnViewF_FR("cp.View: F+FR", false, false);
    pangolin::Var<bool> btnViewFL("cp.View: FL", false, false);
    pangolin::Var<bool> btnViewF("cp.View: F", false, false);
    pangolin::Var<bool> btnViewFR("cp.View: FR", false, false);
    
    pangolin::Var<string> currentViewMode("cp.View", "All");
    
    // Second control panel (right column) for adjustment buttons
    pangolin::CreatePanel("adj").SetBounds(pangolin::Attach::Pix(0), 1.0, pangolin::Attach::Pix(140), pangolin::Attach::Pix(280));
    
    // Calibration buttons - organized in second column
    pangolin::Var<bool> addXdegree("adj.+x deg", false, false);
    pangolin::Var<bool> minusXdegree("adj.-x deg", false, false);
    pangolin::Var<bool> addYdegree("adj.+y deg", false, false);
    pangolin::Var<bool> minusYdegree("adj.-y deg", false, false);
    pangolin::Var<bool> addZdegree("adj.+z deg", false, false);
    pangolin::Var<bool> minusZdegree("adj.-z deg", false, false);
    pangolin::Var<bool> addXtrans("adj.+x tr", false, false);
    pangolin::Var<bool> minusXtrans("adj.-x tr", false, false);
    pangolin::Var<bool> addYtrans("adj.+y tr", false, false);
    pangolin::Var<bool> minusYtrans("adj.-y tr", false, false);
    pangolin::Var<bool> addZtrans("adj.+z tr", false, false);
    pangolin::Var<bool> minusZtrans("adj.-z tr", false, false);
    
    pangolin::Var<bool> resetButton("adj.Reset", false, false);
    pangolin::Var<bool> saveButton("adj.Save", false, false);
    
    vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(addXdegree); mat_calib_box.push_back(minusXdegree);
    mat_calib_box.push_back(addYdegree); mat_calib_box.push_back(minusYdegree);
    mat_calib_box.push_back(addZdegree); mat_calib_box.push_back(minusZdegree);
    mat_calib_box.push_back(addXtrans); mat_calib_box.push_back(minusXtrans);
    mat_calib_box.push_back(addYtrans); mat_calib_box.push_back(minusYtrans);
    mat_calib_box.push_back(addZtrans); mat_calib_box.push_back(minusZtrans);
    
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Handle view mode changes - using push buttons (more reliable than checkboxes)
        if (pangolin::Pushed(btnViewAll)) {
            view_mode = VIEW_ALL;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            cout << "\n[VIEW MODE] FL + F + FR (All cameras)" << endl;
        }
        if (pangolin::Pushed(btnViewFL_FR)) {
            view_mode = VIEW_FL_FR;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            cout << "\n[VIEW MODE] FL + FR (Left + Right side cameras)" << endl;
        }
        if (pangolin::Pushed(btnViewFL_F)) {
            view_mode = VIEW_FL_F;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            cout << "\n[VIEW MODE] FL + F (Left + Center cameras)" << endl;
        }
        if (pangolin::Pushed(btnViewF_FR)) {
            view_mode = VIEW_F_FR;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            cout << "\n[VIEW MODE] F + FR (Center + Right cameras)" << endl;
        }
        if (pangolin::Pushed(btnViewFL)) {
            view_mode = VIEW_FL;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            addOverlayToSingleView(display_image, 0);
            cout << "\n[VIEW MODE] FRONT LEFT (Single camera)" << endl;
        }
        if (pangolin::Pushed(btnViewF)) {
            view_mode = VIEW_F;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            addOverlayToSingleView(display_image, 1);
            cout << "\n[VIEW MODE] FRONT CENTER (Single camera)" << endl;
        }
        if (pangolin::Pushed(btnViewFR)) {
            view_mode = VIEW_FR;
            currentViewMode = getViewModeName();
            display_image = getDisplayImage();
            addOverlayToSingleView(display_image, 2);
            cout << "\n[VIEW MODE] FRONT RIGHT (Single camera)" << endl;
        }
        
        // Handle camera selection buttons
        if (pangolin::Pushed(btnSelectFL)) {
            cali_frame = 0;
            currentCamera = "FL";
            CalibrationScaleChange(cali_frame);
            cout << "Selected camera: FL (front_left)" << endl;
        }
        if (pangolin::Pushed(btnSelectF)) {
            cali_frame = 1;
            currentCamera = "F";
            CalibrationScaleChange(cali_frame);
            cout << "Selected camera: F (front)" << endl;
        }
        if (pangolin::Pushed(btnSelectFR)) {
            cali_frame = 2;
            currentCamera = "FR";
            CalibrationScaleChange(cali_frame);
            cout << "Selected camera: FR (front_right)" << endl;
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
            if (view_mode >= VIEW_FL && view_mode <= VIEW_FR) {
                addOverlayToSingleView(display_image, view_mode - 1);
            }
            cout << "Wall distance: " << wall_distance << " m" << endl;
        }
        
        // Handle calibration buttons
        for (int i = 0; i < 12; i++) {
            if (pangolin::Pushed(mat_calib_box[i])) {
                cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic * modification_list_[cali_frame][i];
                cout << "Calibrated " << cameras[cali_frame].name << endl;
                // FIXED: Update all relevant cameras based on current view mode
                updateProjectionsForViewMode();
                display_image = getDisplayImage();
                if (view_mode >= VIEW_FL && view_mode <= VIEW_FR) {
                    addOverlayToSingleView(display_image, view_mode - 1);
                }
            }
        }
        
        if (pangolin::Pushed(resetButton)) {
            cameras[cali_frame].extrinsic = cameras[cali_frame].extrinsic_initial;
            // FIXED: Update all relevant cameras based on current view mode
            updateProjectionsForViewMode();
            display_image = getDisplayImage();
            if (view_mode >= VIEW_FL && view_mode <= VIEW_FR) {
                addOverlayToSingleView(display_image, view_mode - 1);
            }
            cout << "Reset " << cameras[cali_frame].name << endl;
        }
        
        if (pangolin::Pushed(saveButton)) {
            for (int i = 0; i < 3; i++) saveResult(i);
            saveAllExtrinsics("front3_extrinsics_calibrated.json");
            imwrite("front3_stitched.png", generateStitchedView(0b111));
            cout << "Saved all calibrations" << endl;
        }
        
        // Keyboard input
        if (kbhit()) {
            int c = getchar();
            if (ManualCalibration(c, cali_frame)) {
                // FIXED: Update all relevant cameras based on current view mode
                updateProjectionsForViewMode();
                display_image = getDisplayImage();
                if (view_mode >= VIEW_FL && view_mode <= VIEW_FR) {
                    addOverlayToSingleView(display_image, view_mode - 1);
                }
                cout << "Key calibration applied to " << cameras[cali_frame].name << endl;
            }
        }
        
        // Update display
        imageArray = display_image.data;
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
