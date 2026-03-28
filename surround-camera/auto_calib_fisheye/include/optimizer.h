#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <Eigen/Dense>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Optimizer {
private:
  mutable std::mutex mutexfront;
  mutable std::mutex mutexleft;
  mutable std::mutex mutexright;
  mutable std::mutex mutexbehind;
  mutable std::mutex mutexval;

public:
  // data set index
  string data_index;

  // prefix
  string prefix;

  // camera_model
  int camera_model;

  // which camera fixed
  string fixed;

  // if add coarse search(1st search)
  int coarse_flag;

  // bev rows、cols
  int brows;
  int bcols;

  // SVS cameras intrinsic-T
  Eigen::Matrix3d intrinsic_front;
  Eigen::Matrix3d intrinsic_left;
  Eigen::Matrix3d intrinsic_behind;
  Eigen::Matrix3d intrinsic_right;

  // K_G
  Eigen::Matrix3d KG;

  // initial SVS cameras extrinsics-T(SVS cameras->BEV)
  Eigen::Matrix4d extrinsic_front;
  Eigen::Matrix4d extrinsic_left;
  Eigen::Matrix4d extrinsic_behind;
  Eigen::Matrix4d extrinsic_right;

  // SVS cameras extrinsics-T after optimization
  Eigen::Matrix4d extrinsic_front_opt;
  Eigen::Matrix4d extrinsic_left_opt;
  Eigen::Matrix4d extrinsic_behind_opt;
  Eigen::Matrix4d extrinsic_right_opt;

  // distortion papameters
  vector<double> distortion_params_front;
  vector<double> distortion_params_left;
  vector<double> distortion_params_behind;
  vector<double> distortion_params_right;

  // Ocam model parameters
  struct OcamParams {
    double cx, cy;
    double c, d, e;
    vector<double> cam2world_poly;
    vector<double> world2cam_poly;
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    int width, height;
    
    // Pre-computed polynomial coefficients for efficient evaluation
    // Stored in descending order for Horner's method
    Eigen::VectorXd world2cam_coeffs;
  };

  OcamParams ocam_front;
  OcamParams ocam_left;
  OcamParams ocam_behind;
  OcamParams ocam_right;

  // Extrinsic parameters storage (for saving results)
  Eigen::Matrix4d extrinsic_front_initial;  // Initial extrinsics
  Eigen::Matrix4d extrinsic_left_initial;
  Eigen::Matrix4d extrinsic_behind_initial;
  Eigen::Matrix4d extrinsic_right_initial;

  // SVS cameras height
  double hf, hl, hb, hr;

  // tail size
  int sizef, sizel, sizeb, sizer;

  // SVS luminosity loss after optimization
  double cur_front_loss;
  double cur_left_loss;
  double cur_right_loss;
  double cur_behind_loss;

  // initial SVS luminosity loss
  double max_front_loss;
  double max_left_loss;
  double max_right_loss;
  double max_behind_loss;

  // bev texture pixel
  Mat pG_fl;
  Mat pG_fr;
  Mat pG_bl;
  Mat pG_br;

  // bev texture project to camera coordinate
  Mat PG_fl;
  Mat PG_fr;
  Mat PG_bl;
  Mat PG_br;

  // SVS gray images
  Mat imgf_gray;
  Mat imgl_gray;
  Mat imgb_gray;
  Mat imgr_gray;

  // SVS rgb images
  Mat imgf_rgb;
  Mat imgl_rgb;
  Mat imgb_rgb;
  Mat imgr_rgb;

  // front camera generated BEV image's texture 2d pixels
  vector<Point> fl_pixels_texture;
  vector<Point> fr_pixels_texture;
  vector<Point> bl_pixels_texture;
  vector<Point> br_pixels_texture;

  // euler angles(3) and t parameters(3) to recover precise SVS cameras
  // extrinsics
  vector<vector<double>> bestVal_; //(3*6)

  // SVS generated BEV gray images
  Mat imgf_bev; // initially
  Mat imgl_bev;
  Mat imgr_bev;
  Mat imgb_bev;

  // SVS generated BEV rgb images
  Mat imgf_bev_rgb; // initially
  Mat imgl_bev_rgb;
  Mat imgr_bev_rgb;
  Mat imgb_bev_rgb;

  // ncoef-commonView mean luminorsity ratio
  double ncoef_fl, ncoef_fr, ncoef_bl, ncoef_br;

  // Optimizer();
  Optimizer(const Mat *imgf, const Mat *imgl, const Mat *imgb, const Mat *imgr,
            int camera_model_index, int rows, int cols, string first_order,
            int flag, string data_set);
  ~Optimizer();
  void initializeK();
  void initializeD();
  void initializePose();
  void initializeKG();
  void initializeHeight();
  void initializetailsize();
  void loadOcamParams(const string &calib_file_front, const string &calib_file_left,
                      const string &calib_file_behind, const string &calib_file_right);
  OcamParams loadSingleOcamParam(const string &calib_file);
  
  // Extrinsics loading and saving
  void loadExtrinsicsFromJson(const string &filename);
  void saveExtrinsicsToJson(const string &filename, const string &camera_name,
                            const Eigen::Matrix4d &extrinsic);
  Eigen::Matrix3d rodriguesToRotationMatrix(const vector<double> &rvec);
  void rodriguesToRotationMatrix(const double rvec[3], double R[9]);
  Mat tail(Mat img, string index);
  void Calibrate_left(int search_count, double roll_ep0, double roll_ep1,
                      double pitch_ep0, double pitch_ep1, double yaw_ep0,
                      double yaw_ep1, double t0_ep0, double t0_ep1,
                      double t1_ep0, double t1_ep1, double t2_ep0,
                      double t2_ep1);
  void Calibrate_right(int search_count, double roll_ep0, double roll_ep1,
                       double pitch_ep0, double pitch_ep1, double yaw_ep0,
                       double yaw_ep1, double t0_ep0, double t0_ep1,
                       double t1_ep0, double t1_ep1, double t2_ep0,
                       double t2_ep1);
  void Calibrate_behind(int search_count, double roll_ep0, double roll_ep1,
                        double pitch_ep0, double pitch_ep1, double yaw_ep0,
                        double yaw_ep1, double t0_ep0, double t0_ep1,
                        double t1_ep0, double t1_ep1, double t2_ep0,
                        double t2_ep1);
  void Calibrate_front(int search_count, double roll_ep0, double roll_ep1,
                       double pitch_ep0, double pitch_ep1, double yaw_ep0,
                       double yaw_ep1, double t0_ep0, double t0_ep1,
                       double t1_ep0, double t1_ep1, double t2_ep0,
                       double t2_ep1);
  void fine_Calibrate_right(int search_count, double roll_ep0, double roll_ep1,
                            double pitch_ep0, double pitch_ep1, double yaw_ep0,
                            double yaw_ep1, double t0_ep0, double t0_ep1,
                            double t1_ep0, double t1_ep1, double t2_ep0,
                            double t2_ep1);
  void fine_Calibrate_left(int search_count, double roll_ep0, double roll_ep1,
                           double pitch_ep0, double pitch_ep1, double yaw_ep0,
                           double yaw_ep1, double t0_ep0, double t0_ep1,
                           double t1_ep0, double t1_ep1, double t2_ep0,
                           double t2_ep1);
  void fine_Calibrate_behind(int search_count, double roll_ep0, double roll_ep1,
                             double pitch_ep0, double pitch_ep1, double yaw_ep0,
                             double yaw_ep1, double t0_ep0, double t0_ep1,
                             double t1_ep0, double t1_ep1, double t2_ep0,
                             double t2_ep1);
  void fine_Calibrate_front(int search_count, double roll_ep0, double roll_ep1,
                            double pitch_ep0, double pitch_ep1, double yaw_ep0,
                            double yaw_ep1, double t0_ep0, double t0_ep1,
                            double t1_ep0, double t1_ep1, double t2_ep0,
                            double t2_ep1);
  double CostFunction(const vector<double> var, string idx);
  double fine_CostFunction(const vector<double> var, string idx);
  void SaveOptResult(const string img_name);
  void show(string idx, string filename);
  cv::Mat eigen2mat(Eigen::MatrixXd A);
  Eigen::MatrixXd cvMat2Eigen(cv::Mat A);
  Mat gray_gamma(Mat img);
  void world2cam(double point2D[2], double point3D[3], Eigen::Matrix3d K,
                 vector<double> D);
  void distortPoints(cv::Mat &P_GC1, cv::Mat &p_GC, Eigen::Matrix3d &K_C);
  void distortPointsOcam(cv::Mat &P_GC1, cv::Mat &p_GC, Eigen::Matrix3d &K_C,
                         vector<double> &D_C);
  void random_search_params(int search_count, double roll_ep0, double roll_ep1,
                            double pitch_ep0, double pitch_ep1, double yaw_ep0,
                            double yaw_ep1, double t0_ep0, double t0_ep1,
                            double t1_ep0, double t1_ep1, double t2_ep0,
                            double t2_ep1, string idx);
  void fine_random_search_params(int search_count, double roll_ep0,
                                 double roll_ep1, double pitch_ep0,
                                 double pitch_ep1, double yaw_ep0,
                                 double yaw_ep1, double t0_ep0, double t0_ep1,
                                 double t1_ep0, double t1_ep1, double t2_ep0,
                                 double t2_ep1, string idx);

  Mat generate_surround_view(Mat img_GF, Mat img_GL, Mat img_GB, Mat img_GR);
  vector<Point> readfromcsv(string filename);
  Mat project_on_ground(cv::Mat img, Eigen::Matrix4d T_CG, Eigen::Matrix3d K_C,
                        vector<double> D_C, Eigen::Matrix3d K_G, int rows,
                        int cols, float height);
  Mat project_on_ground(cv::Mat img, Eigen::Matrix4d T_CG, Eigen::Matrix3d K_C,
                        vector<double> D_C, Eigen::Matrix3d K_G, int rows,
                        int cols, float height, const string &camera_idx);
  double back_camera_and_compute_loss(Mat img1, Mat img2, Eigen::Matrix4d T,
                                      string idx, const string &camera_idx);
  double getPixelValue(Mat *image, float x, float y);
};

#endif //  OPTIMIZER_H_