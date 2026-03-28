#include "optimizer.h"
#include "transform_util.h"
#include <fstream>
#include <random>
#include <json/json.h>

Mat Optimizer::eigen2mat(Eigen::MatrixXd A) {
  Mat B;
  eigen2cv(A, B);

  return B;
}

Eigen::MatrixXd Optimizer::cvMat2Eigen(cv::Mat A) {
  Eigen::MatrixXd B;
  cv2eigen(A, B);
  return B;
}

Mat Optimizer::gray_gamma(Mat img) {
  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  double contrast = 1.1;
  double brightness = 0;
  double delta = 30;
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      int g = gray.at<uchar>(i, j);
      gray.at<uchar>(i, j) = saturate_cast<uchar>(
          contrast * (gray.at<uchar>(i, j) - delta) + brightness);
    }
  }
  return gray;
}

double Optimizer::getPixelValue(Mat *image, float x, float y) {
  // 法1：双线性插值
  uchar *data = &image->data[int(y) * image->step + int(x)];
  float xx = x - floor(x);
  float yy = y - floor(y);
  return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] +
               (1 - xx) * yy * data[image->step] +
               xx * yy * data[image->step + 1]);
}

vector<Point> Optimizer::readfromcsv(string filename) {
  vector<Point> pixels;
  ifstream inFile(filename, ios::in);
  string lineStr;
  while (getline(inFile, lineStr)) {
    Point pixel;
    istringstream record(lineStr);
    string x, y;
    record >> x;
    pixel.x = atoi(x.c_str());
    record >> y;
    pixel.y = atoi(y.c_str());
    pixels.push_back(pixel);
  }
  return pixels;
}

Mat Optimizer::tail(Mat img, string index) {
  if (index == "f") {
    Rect m_select_f = Rect(0, 0, img.cols, sizef);
    Mat cropped_image_f = img(m_select_f);
    Mat border(img.rows - sizef, img.cols, cropped_image_f.type(),
               Scalar(0, 0, 0));
    Mat dst_front;
    vconcat(cropped_image_f, border, dst_front);
    return dst_front;
  } else if (index == "l") {
    Rect m_select_l = Rect(0, 0, sizel, img.rows);
    Mat cropped_image_l = img(m_select_l);
    Mat border2(img.rows, img.cols - sizel, cropped_image_l.type(),
                Scalar(0, 0, 0));
    Mat dst_left;
    hconcat(cropped_image_l, border2, dst_left);
    return dst_left;
  } else if (index == "b") {
    Rect m_select_b = Rect(0, img.rows - sizeb, img.cols, sizeb);
    Mat cropped_image_b = img(m_select_b);
    Mat border1(img.rows - sizeb, img.cols, cropped_image_b.type(),
                Scalar(0, 0, 0));
    Mat dst_behind;
    vconcat(border1, cropped_image_b, dst_behind);
    return dst_behind;
  } else if (index == "r") {
    Rect m_select_r = Rect(img.cols - sizer, 0, sizer, img.rows);
    Mat cropped_image_r = img(m_select_r);
    Mat border3(img.rows, img.cols - sizer, cropped_image_r.type(),
                Scalar(0, 0, 0));
    Mat dst_right;
    hconcat(border3, cropped_image_r, dst_right);
    return dst_right;
  }
  return Mat(img.rows, img.cols, img.type());
}

void Optimizer::SaveOptResult(const string filename) {
  Mat opt_after = generate_surround_view(imgf_bev_rgb, imgl_bev_rgb,
                                         imgb_bev_rgb, imgr_bev_rgb);
  imwrite(filename, opt_after);
}

void Optimizer::show(string idx, string filename) {
  Mat dst, dst1;

  if (idx == "right") { // first
    if (fixed == "front")
      addWeighted(imgf_bev_rgb, 0.5, imgr_bev_rgb, 0.5, 3, dst);
    else
      addWeighted(imgb_bev_rgb, 0.5, imgr_bev_rgb, 0.5, 3, dst);
    imwrite(filename, dst);
  }
  if (idx == "behind") { // second
    addWeighted(imgb_bev_rgb, 0.5, imgr_bev_rgb, 0.5, 3, dst);
    addWeighted(dst, 1, imgl_bev_rgb, 0.5, 3, dst1);
    imwrite(filename, dst1);
  }
  if (idx == "left") { // third
    if (fixed == "front")
      addWeighted(imgl_bev_rgb, 0.5, imgf_bev_rgb, 0.5, 3, dst1);
    else
      addWeighted(imgl_bev_rgb, 0.5, imgb_bev_rgb, 0.5, 3, dst1);
  }
  if (idx == "front") {
    addWeighted(imgf_bev_rgb, 0.5, imgr_bev_rgb, 0.5, 3, dst);
    addWeighted(dst, 1, imgl_bev_rgb, 0.5, 3, dst1);
    imwrite(filename, dst1);
  }
}

void Optimizer::world2cam(double point2D[2], double point3D[3],
                          Eigen::Matrix3d K, vector<double> D) {
  double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
  double theta = atan(point3D[2] / norm);
  double t, t_i;
  double rho, x, y;
  double invnorm;
  int i;

  if (norm != 0) {
    invnorm = 1 / norm;
    t = theta;
    rho = D[0];
    t_i = 1;

    for (i = 1; i < D.size(); i++) {
      t_i *= t;
      rho += t_i * D[i];
    }

    x = point3D[0] * invnorm * rho;
    y = point3D[1] * invnorm * rho;

    point2D[0] = x * K(0, 0) + y * K(0, 1) + K(0, 2);
    point2D[1] = x * K(1, 0) + y + K(1, 2);
  } else {
    point2D[0] = K(0, 2);
    point2D[1] = K(1, 2);
  }
}

// Evaluate polynomial using Horner's method
double evaluatePolynomial(const vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = coeffs.size() - 1; i >= 0; i--) {
    result = result * x + coeffs[i];
  }
  return result;
}

void Optimizer::distortPointsOcam(Mat &P_GC1, Mat &p_GC, Eigen::Matrix3d &K_C,
                                  vector<double> &D_C) {
  // Determine which camera's parameters to use based on context
  // For now, use a simple approach - this will be refined in project_on_ground

  for (int i = 0; i < P_GC1.cols; i++) {
    double x = P_GC1.at<Vec2d>(0, i)[0];
    double y = P_GC1.at<Vec2d>(0, i)[1];

    // 3D ray direction (normalized)
    double norm = sqrt(x * x + y * y);
    if (norm < 1e-6) {
      p_GC.at<Vec2d>(0, i)[0] = K_C(0, 2);
      p_GC.at<Vec2d>(0, i)[1] = K_C(1, 2);
      continue;
    }

    // Standard to Ocam coordinate system: z is negative
    double z_omni = -1.0;

    // Calculate incident angle theta
    double theta = atan(z_omni / norm);

    // This is a placeholder - in practice, we need to know which camera
    // we're projecting to and use its world2cam polynomial
    // For now, use a generic polynomial
    double rho = evaluatePolynomial(D_C, theta);

    // Project to normalized plane
    double x_norm = x / norm * rho;
    double y_norm = y / norm * rho;

    // Apply affine transformation (placeholder)
    double u = x_norm + K_C(0, 2);
    double v = y_norm + K_C(1, 2);

    p_GC.at<Vec2d>(0, i)[0] = u;
    p_GC.at<Vec2d>(0, i)[1] = v;
  }
}

// New function: distortPointsOcam with explicit Ocam parameters
// Based on CPAC implementation in bev_generator.py
void distortPointsOcamFull(Mat &P_GC1, Mat &p_GC, const Optimizer::OcamParams &ocam) {
  for (int i = 0; i < P_GC1.cols; i++) {
    // P_GC1 contains 3D camera coordinates [x, y, z]
    double x = P_GC1.at<Vec3d>(0, i)[0];
    double y = P_GC1.at<Vec3d>(0, i)[1];
    double z = P_GC1.at<Vec3d>(0, i)[2];

    // Check if point is visible by camera
    // After transformation, if z > 0, point is in front of camera
    // If z < 0, point is behind camera
    if (z < 0) {
      // Point is behind camera, mark as invalid
      p_GC.at<Vec2d>(0, i)[0] = -1;
      p_GC.at<Vec2d>(0, i)[1] = -1;
      continue;
    }

    // CPAC formula: norm = sqrt(x*x + y*y)
    double norm = sqrt(x * x + y * y);
    if (norm < 1e-14) {
      norm = 1e-14;
    }

    // CPAC: theta = atan(-z / norm) - note the negative sign
    // z>0 is above ground, so -z points downward (into ground)
    double theta = atan(-z / norm);

    // Evaluate polynomial: rho = sum(invpol[k] * theta^k)
    // Using Horner's method (coeffs stored in reverse order)
    double rho = 0.0;
    for (int k = 0; k < ocam.world2cam_coeffs.size(); k++) {
      rho = rho * theta + ocam.world2cam_coeffs(k);
    }

    // CPAC: uu = x / norm * rho, vv = y / norm * rho
    double uu = x / norm * rho;
    double vv = y / norm * rho;

    // CPAC affine transformation:
    // u = uu + vv * intri_e + intri_cx
    // v = uu * intri_d + vv * intri_c + intri_cy
    double u = uu + vv * ocam.e + ocam.cx;
    double v = uu * ocam.d + vv * ocam.c + ocam.cy;

    p_GC.at<Vec2d>(0, i)[0] = u;
    p_GC.at<Vec2d>(0, i)[1] = v;
  }
}

void Optimizer::distortPoints(Mat &P_GC1, Mat &p_GC, Eigen::Matrix3d &K_C) {
  for (int i = 0; i < P_GC1.cols; i++) {
    double x = P_GC1.at<Vec2d>(0, i)[0];
    double y = P_GC1.at<Vec2d>(0, i)[1];

    double u = x * K_C(0, 0) + K_C(0, 2);
    double v = y * K_C(1, 1) + K_C(1, 2);

    p_GC.at<Vec2d>(0, i)[0] = u;
    p_GC.at<Vec2d>(0, i)[1] = v;
  }
}

void Optimizer::initializeK() {
  Eigen::Matrix3d K_F;
  Eigen::Matrix3d K_L;
  Eigen::Matrix3d K_B;
  Eigen::Matrix3d K_R;

  // common
  if (data_index == "common") {
    K_F << 390.425287, 0.00000000, 750, 0.00000000, 390.425287, 750, 0.00000000,
        0.00000000, 1.00000000;
    K_L << 390.425287, 0.00000000, 750, 0.00000000, 390.425287, 750, 0.00000000,
        0.00000000, 1.00000000;
    K_B << 390.425287, 0.00000000, 750, 0.00000000, 390.425287, 750, 0.00000000,
        0.00000000, 1.00000000;
    K_R << 390.425287, 0.00000000, 750, 0.00000000, 390.425287, 750, 0.00000000,
        0.00000000, 1.00000000;
  }
  // fisheye
  else if (data_index == "fisheye") {
    K_F << 422.13163849, 0.00000000, 612.82890504, 0.00000000, 421.10340889,
        545.05656249, 0.00000000, 0.00000000, 1.00000000;
    K_L << 420.60079305, 0.00000000, 650.54173853, 0.00000000, 418.94827188,
        527.27178143, 0.00000000, 0.00000000, 1.00000000;
    K_B << 422.61569350, 0.00000000, 632.46019501, 0.00000000, 421.27373079,
        548.34673288, 0.00000000, 0.00000000, 1.00000000;
    K_R << 421.64203585, 0.00000000, 640.09362064, 0.00000000, 420.26647020,
        529.05566315, 0.00000000, 0.00000000, 1.00000000;
  }
  // ocam - use identity or approximate values, real projection uses Ocam params
  else if (data_index == "ocam") {
    // For Ocam, we use a dummy intrinsic matrix since actual projection
    // uses the Ocam polynomial model. This is just for compatibility.
    K_F = Eigen::Matrix3d::Identity();
    K_L = Eigen::Matrix3d::Identity();
    K_B = Eigen::Matrix3d::Identity();
    K_R = Eigen::Matrix3d::Identity();
    K_F(0, 2) = ocam_front.cx;
    K_F(1, 2) = ocam_front.cy;
    K_L(0, 2) = ocam_left.cx;
    K_L(1, 2) = ocam_left.cy;
    K_B(0, 2) = ocam_behind.cx;
    K_B(1, 2) = ocam_behind.cy;
    K_R(0, 2) = ocam_right.cx;
    K_R(1, 2) = ocam_right.cy;
  }

  intrinsic_front = K_F;
  intrinsic_left = K_L;
  intrinsic_behind = K_B;
  intrinsic_right = K_R;
}

void Optimizer::initializeD() {
  vector<double> D_F;
  vector<double> D_L;
  vector<double> D_B;
  vector<double> D_R;

  // common-pinhole
  if (data_index == "common") {
    D_F = {0, 0, 0, 0};
    D_L = {0, 0, 0, 0};
    D_B = {0, 0, 0, 0};
    D_R = {0, 0, 0, 0};
  }
  // fisheye
  else if (data_index == "fisheye") {
    D_F = {-0.07031853, 0.00387505, -0.00333139, 0.00056406};
    D_L = {-6.58382798e-02, -2.00728513e-03, -3.72535694e-04, 1.81851668e-06};
    D_B = {-0.06553861, -0.00094857, -0.00150748, 0.000325};
    D_R = {-0.07289752, 0.01254629, -0.01300477, 0.00361266};
  }
  // ocam - not used, actual distortion uses Ocam polynomial model
  else if (data_index == "ocam") {
    D_F = {0, 0, 0, 0};
    D_L = {0, 0, 0, 0};
    D_B = {0, 0, 0, 0};
    D_R = {0, 0, 0, 0};
  }

  distortion_params_front = D_F;
  distortion_params_left = D_L;
  distortion_params_behind = D_B;
  distortion_params_right = D_R;
}

void Optimizer::initializePose() { // ground->camera
  Eigen::Matrix4d T_FG;
  Eigen::Matrix4d T_LG;
  Eigen::Matrix4d T_BG;
  Eigen::Matrix4d T_RG;

  // common
  if (data_index == "common") {
    T_FG << 1, 0, 0, 0, 0, 0, 1, -4.1, 0, -1, 0, -2.5, 0, 0, 0, 1;
    T_LG << 0, -1, 0, 0, 0, 0, 1, -4.1, -1, 0, 0, -1, 0, 0, 0, 1;
    T_BG << -1, 0, 0, 0, 0, 0, 1, -4.1, 0, 1, 0, -2, 0, 0, 0, 1;
    T_RG << 0, 1, 0, 0, 0, 0, 1, -4.1, 1, 0, 0, -1, 0, 0, 0, 1;
  }
  // ROCES
  else if (data_index == "fisheye") {
    T_FG << 9.99277118e-01, 3.82390286e-04, -3.80143958e-02, 6.75437418e-01,
        -2.30748265e-02, -7.88582447e-01, -6.14495953e-01, 2.50896883e+01,
        -3.02124625e-02, 6.14928921e-01, -7.88003572e-01, 3.17779305e+00, 0, 0,
        0, 1;
    T_LG << -1.21898860e-02, 9.99924056e-01, -1.81349393e-03, 1.36392943e+00,
        8.02363600e-01, 8.69913885e-03, -5.96772133e-01, 1.60942881e+01,
        -5.96711036e-01, -8.72966581e-03, -8.02408707e-01, 1.04105913e+01, 0, 0,
        0, 1;
    T_BG << -9.99615699e-01, 1.56439861e-02, -2.28849354e-02, 1.09266953e+00,
        2.59906371e-02, 8.16008735e-01, -5.77454960e-01, 2.46308124e+01,
        9.64060983e-03, -5.77827838e-01, -8.16101739e-01, 6.60957845e+00, 0, 0,
        0, 1;
    T_RG << 4.57647596e-03, -9.99989102e-01, 9.22798184e-04, -1.66115120e-01,
        -6.26343448e-01, -3.58584197e-03, -7.79538984e-01, 1.76226207e+01,
        7.79533797e-01, 2.98955282e-03, -6.26353033e-01, 6.08338205e+00, 0, 0,
        0, 1;

    Eigen::Matrix4d left_disturbance;
    Eigen::Matrix3d left_disturbance_rot_mat;
    Vec3f left_disturbance_rot_euler; // R(euler)
    Mat_<double> left_disturbance_t =
        (Mat_<double>(3, 1) << 0.0095, 0.0025, -0.0086);
    left_disturbance_rot_euler << 0.95, 1.25, -2.86;
    left_disturbance_rot_mat =
        TransformUtil::eulerAnglesToRotationMatrix(left_disturbance_rot_euler);
    left_disturbance = TransformUtil::R_T2RT(
        TransformUtil::eigen2mat(left_disturbance_rot_mat), left_disturbance_t);
    T_LG *= left_disturbance;

    Eigen::Matrix4d right_disturbance;
    Eigen::Matrix3d right_disturbance_rot_mat;
    Vec3f right_disturbance_rot_euler;
    Mat_<double> right_disturbance_t =
        (Mat_<double>(3, 1) << 0.0065, -0.0075, 0.0095);
    right_disturbance_rot_euler << -2.95, 1.25, -2.8;
    right_disturbance_rot_mat =
        TransformUtil::eulerAnglesToRotationMatrix(right_disturbance_rot_euler);
    right_disturbance = TransformUtil::R_T2RT(
        TransformUtil::eigen2mat(right_disturbance_rot_mat), right_disturbance_t);
    T_RG *= right_disturbance;

    Eigen::Matrix4d behind_disturbance;
    Eigen::Matrix3d behind_disturbance_rot_mat;
    Vec3f behind_disturbance_rot_euler;
    Mat_<double> behind_disturbance_t =
        (Mat_<double>(3, 1) << -0.002, -0.0076, 0.0096);
    behind_disturbance_rot_euler << -1.75, 2.95, -1.8;
    behind_disturbance_rot_mat =
        TransformUtil::eulerAnglesToRotationMatrix(behind_disturbance_rot_euler);
    behind_disturbance = TransformUtil::R_T2RT(
        TransformUtil::eigen2mat(behind_disturbance_rot_mat),
        behind_disturbance_t);
    T_BG *= behind_disturbance;
  }
  // ocam - initialize with identity, will be overwritten by loadExtrinsicsFromJson
  else if (data_index == "ocam") {
    T_FG = Eigen::Matrix4d::Identity();
    T_LG = Eigen::Matrix4d::Identity();
    T_BG = Eigen::Matrix4d::Identity();
    T_RG = Eigen::Matrix4d::Identity();
    // No disturbance for ocam - rely on loaded extrinsics
  }

  extrinsic_front = T_FG;
  extrinsic_left = T_LG;
  extrinsic_behind = T_BG;
  extrinsic_right = T_RG;

  cout << "extrinsic_front:" << endl << extrinsic_front << endl;
  cout << "eular:" << endl
       << TransformUtil::Rotation2Eul(extrinsic_front.block(0, 0, 3, 3))
       << endl;
  cout << "extrinsic_left:" << endl << extrinsic_left << endl;
  cout << "eular:" << endl
       << TransformUtil::Rotation2Eul(extrinsic_left.block(0, 0, 3, 3)) << endl;
  cout << "extrinsic_right:" << endl << extrinsic_right << endl;
  cout << "eular:" << endl
       << TransformUtil::Rotation2Eul(extrinsic_right.block(0, 0, 3, 3))
       << endl;
  cout << "extrinsic_behind:" << endl << extrinsic_behind << endl;
  cout << "eular:" << endl
       << TransformUtil::Rotation2Eul(extrinsic_behind.block(0, 0, 3, 3))
       << endl;
  return;
}

void Optimizer::initializeKG() {
  Eigen::Matrix3d K_G = Eigen::Matrix3d::Zero();

  // common
  if (data_index == "common") {
    K_G << 390.425287, 0.00000000, 750, 0.00000000, 390.425287, 750, 0.00000000,
        0.00000000, 1.00000000;
    KG = K_G;
  }
  // fisheye
  else if (data_index == "fisheye") {
    K_G(0, 0) = 1 / 0.15;
    K_G(1, 1) = -1 / 0.15;
    K_G(0, 2) = bcols / 2;
    K_G(1, 2) = brows / 2;
    K_G(2, 2) = 1.0;
    KG = K_G;
  }
  // ocam - calculate from camera parameters (approximate)
  else if (data_index == "ocam") {
    // CPAC uses metric_ratio = 1.0 (cm/pixel) = 0.01 (m/pixel)
    // This means KG = 1/0.01 = 100 (pixels/meter)
    // In CPAC: bev_width_pixel = veh_width * 100 / metric_ratio
    //         veh_pt = [veh_x * metric_ratio/100, veh_y * metric_ratio/100, 0, 1]
    
    double metric_ratio = 0.01; // meters per pixel (1cm/pixel, same as CPAC)
    double pixels_per_meter = 1.0 / metric_ratio; // = 100 pixels/meter
    
    // Note: In CPAC, y is left direction, and image y is down
    // So K_G(1,1) should be negative to flip the y direction
    K_G(0, 0) = pixels_per_meter;
    K_G(1, 1) = -pixels_per_meter; // negative because image y is down, but vehicle y is left
    K_G(0, 2) = bcols / 2.0;
    K_G(1, 2) = brows / 2.0;
    K_G(2, 2) = 1.0;
    KG = K_G;

    cout << "[KG Initialization] metric_ratio: " << metric_ratio * 100 << " cm/pixel (same as CPAC)" << endl;
    cout << "[KG Initialization] pixels_per_meter: " << pixels_per_meter << " pixels/m" << endl;
    cout << "[KG Initialization] K_G matrix:" << endl << K_G << endl;
  }
}

void Optimizer::initializeHeight() {
  if (data_index == "common") { // common
    hf = 5.1;
    hl = 5.1;
    hb = 5.1;
    hr = 5.1;
  } else if (data_index == "fisheye") { // fisheye
    hf = 1;
    hl = 1;
    hb = 1;
    hr = 1;
  } else if (data_index == "ocam") { // ocam - camera height in meters
    // Approximate height for park cameras (from extrinsics)
    // Will be updated after loading extrinsics
    hf = 0.160;
    hl = 0.160;
    hb = 0.160;
    hr = 0.160;
  }
}

void Optimizer::initializetailsize() {
  // common
  if (data_index == "common") {
    sizef = 450;
    sizel = 450;
    sizeb = 350;
    sizer = 450;
  }
  // fisheye
  else if (data_index == "fisheye") {
    sizef = 340;
    sizel = 390;
    sizeb = 380;
    sizer = 390;
  }
  // ocam - use similar values as fisheye for now
  else if (data_index == "ocam") {
    sizef = 340;
    sizel = 390;
    sizeb = 380;
    sizer = 390;
  }
}

Optimizer::Optimizer(const Mat *imgf, const Mat *imgl, const Mat *imgb,
                     const Mat *imgr, int camera_model_index, int rows,
                     int cols, string fixed_, int flag, string data_set) {
  imgf_rgb = *imgf;
  imgf_gray = gray_gamma(imgf_rgb);

  imgl_rgb = *imgl;
  imgl_gray = gray_gamma(imgl_rgb);

  imgb_rgb = *imgb;
  imgb_gray = gray_gamma(imgb_rgb);

  imgr_rgb = *imgr;
  imgr_gray = gray_gamma(imgr_rgb);

  brows = rows;
  bcols = cols;

  data_index = data_set;

  initializeK();
  initializeD();
  initializePose();
  initializeKG();
  initializeHeight();
  initializetailsize();

  camera_model = camera_model_index;

  bestVal_.resize(3, vector<double>(6));
  fixed = fixed_;

  coarse_flag = flag;

  // For ocam model, delay BEV generation until after extrinsics are loaded
  // because camera height is needed for accurate projection
  if (data_index != "ocam") {
    if (fixed == "front") {
      imgf_bev = project_on_ground(imgf_gray, extrinsic_front, intrinsic_front,
                                   distortion_params_front, KG, brows, bcols, hf, "front");
      imgf_bev_rgb =
          project_on_ground(imgf_rgb, extrinsic_front, intrinsic_front,
                            distortion_params_front, KG, brows, bcols, hf, "front");
      imgf_bev = tail(imgf_bev, "f");
      imgf_bev_rgb = tail(imgf_bev_rgb, "f");
    } else {
      imgb_bev =
          project_on_ground(imgb_gray, extrinsic_behind, intrinsic_behind,
                            distortion_params_behind, KG, brows, bcols, hb, "behind");
      imgb_bev_rgb =
          project_on_ground(imgb_rgb, extrinsic_behind, intrinsic_behind,
                            distortion_params_behind, KG, brows, bcols, hb, "behind");
      imgb_bev = tail(imgb_bev, "b");
      imgb_bev_rgb = tail(imgb_bev_rgb, "b");
    }
  }
}

Optimizer::~Optimizer() {}

Mat Optimizer::project_on_ground(Mat img, Eigen::Matrix4d T_CG,
                                 Eigen::Matrix3d K_C, vector<double> D_C,
                                 Eigen::Matrix3d K_G, int rows, int cols,
                                 float height) {
  return project_on_ground(img, T_CG, K_C, D_C, K_G, rows, cols, height, "front");
}

// Overloaded version with Ocam support
// Based on CPAC bev_generator.py implementation
Mat Optimizer::project_on_ground(Mat img, Eigen::Matrix4d T_CG,
                                 Eigen::Matrix3d K_C, vector<double> D_C,
                                 Eigen::Matrix3d K_G, int rows, int cols,
                                 float height, const string &camera_idx) {
  cout << "\n==========================================" << endl;
  cout << "DEBUG project_on_ground: " << camera_idx << endl;
  cout << "Input image: " << img.cols << "x" << img.rows << " channels=" << img.channels() << endl;

  // CPAC BEV coordinate generation:
  // veh_x = wheel_base_pixel/2 + bev_height_pixel/2 - j (row)
  // veh_y = bev_width_pixel/2 - i (col)
  // veh_pt = [veh_x * metric_ratio/100, veh_y * metric_ratio/100, 0, 1]
  // 
  // Body coordinate system: x=forward, y=left, z=up
  // BEV image: row=j (downward), col=i (rightward)
  // 
  // For metric_ratio = 1.0 (cm/pixel) = 0.01 (m/pixel):
  // veh_pt in meters
  
  double metric_ratio_m = 0.01;  // meters per pixel (1cm/pixel, same as CPAC)
  double wheel_base = 3.01;  // Default wheel base in meters
  
  // Build 4xN matrix of ground points in body/vehicle coordinates
  Mat P_G = Mat::ones(4, rows * cols, CV_64FC1);
  
  for (int j = 0; j < rows; j++) {  // j = row index (0 at top)
    for (int i = 0; i < cols; i++) {  // i = col index (0 at left)
      int idx = j * cols + i;
      
      // CPAC formula interpretation:
      // wheel_base_pixel = wheel_base / metric_ratio (converting meters to pixels)
      // veh_x (forward): larger at top of image (smaller j), smaller at bottom
      // veh_y (left): larger at left of image (smaller i), smaller at right
      double wheel_base_pixel = wheel_base / metric_ratio_m;
      
      // Forward direction: x positive forward
      // At j=0 (top): veh_x = wheel_base/2 + height/2 (forward)
      // At j=rows (bottom): veh_x = wheel_base/2 - height/2 (backward)
      double veh_x = (wheel_base_pixel / 2.0 + rows / 2.0 - j) * metric_ratio_m;
      
      // Left direction: y positive left  
      // At i=0 (left): veh_y = width/2 (left)
      // At i=cols (right): veh_y = -width/2 (right)
      double veh_y = (cols / 2.0 - i) * metric_ratio_m;
      
      P_G.at<double>(0, idx) = veh_x;  // x_forward (meters)
      P_G.at<double>(1, idx) = veh_y;  // y_left (meters)
      P_G.at<double>(2, idx) = 0.0;    // z_up (ground plane)
    }
  }

  double min_x, max_x, min_y, max_y;
  cv::minMaxLoc(P_G.row(0), &min_x, &max_x);
  cv::minMaxLoc(P_G.row(1), &min_y, &max_y);
  cout << "Ground range (body coords, meters): x=[" << min_x << ", " << max_x << "], y=[" << min_y << ", " << max_y << "]" << endl;

  // Transform from body coordinates to camera coordinates
  // T_CG is T_vehicle_to_camera (body-to-camera transform)
  Eigen::MatrixXd P_G_eigen = cvMat2Eigen(P_G);
  cout << "T_CG (body-to-camera):" << endl << T_CG << endl;
  
  Eigen::MatrixXd P_GC_eigen = T_CG * P_G_eigen;
  Mat P_GC = eigen2mat(P_GC_eigen);

  double min_xc, max_xc, min_yc, max_yc, min_zc, max_zc;
  cv::minMaxLoc(P_GC.row(0), &min_xc, &max_xc);
  cv::minMaxLoc(P_GC.row(1), &min_yc, &max_yc);
  cv::minMaxLoc(P_GC.row(2), &min_zc, &max_zc);
  cout << "Camera frame range: x=[" << min_xc << ", " << max_xc << "], y=[" << min_yc << ", " << max_yc << "], z=[" << min_zc << ", " << max_zc << "]" << endl;

  // Convert to 3D points for projection
  Mat P_GC1 = Mat::zeros(1, rows * cols, CV_64FC3);
  for (int i = 0; i < rows * cols; i++) {
    P_GC1.at<Vec3d>(0, i) = Vec3d(P_GC.at<double>(0, i), P_GC.at<double>(1, i), P_GC.at<double>(2, i));
  }

  cout << "P_GC1 first 5 points (camera coords):" << endl;
  for (int i = 0; i < min(5, rows * cols); i++) {
    Vec3d pt = P_GC1.at<Vec3d>(0, i);
    cout << "  " << i << ": (" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << endl;
  }

  Mat p_GC = Mat::zeros(1, rows * cols, CV_64FC2);

  // Project to image using appropriate camera model
  if (camera_model == 0) {
    // For fisheye, convert K_C to cv::Mat
    Mat K_C_mat = eigen2mat(K_C);
    fisheye::distortPoints(P_GC1, p_GC, K_C_mat, D_C);
  } else if (camera_model == 1) {
    const OcamParams *ocam = nullptr;
    if (camera_idx == "front") ocam = &ocam_front;
    else if (camera_idx == "left") ocam = &ocam_left;
    else if (camera_idx == "behind") ocam = &ocam_behind;
    else if (camera_idx == "right") ocam = &ocam_right;

    if (ocam && !ocam->world2cam_poly.empty()) {
      cout << "Using Ocam full model for camera: " << camera_idx << endl;
      cout << "Ocam params - cx=" << ocam->cx << ", cy=" << ocam->cy << ", pol[0]=" << ocam->world2cam_poly[0] << endl;
      distortPointsOcamFull(P_GC1, p_GC, *ocam);
    } else {
      cout << "WARNING: Ocam params empty, using fallback" << endl;
      distortPointsOcam(P_GC1, p_GC, K_C, D_C);
    }
  } else {
    distortPoints(P_GC1, p_GC, K_C);
  }

  // Boundary check: count valid points and compute pixel range
  int valid_count = 0;
  double min_u = img.cols, max_u = 0, min_v = img.rows, max_v = 0;
  
  for (int i = 0; i < rows * cols; i++) {
    double u = p_GC.at<Vec2d>(0, i)[0];
    double v = p_GC.at<Vec2d>(0, i)[1];
    
    if (u >= 0 && u < img.cols && v >= 0 && v < img.rows) {
      valid_count++;
      if (u < min_u) min_u = u;
      if (u > max_u) max_u = u;
      if (v < min_v) min_v = v;
      if (v > max_v) max_v = v;
    }
  }
  cout << "Valid projected points: " << valid_count << "/" << rows * cols 
       << " (" << 100.0 * valid_count / (rows * cols) << "%)" << endl;
  cout << "Projected pixel range: u=[" << min_u << ", " << max_u << "], v=[" << min_v << ", " << max_v << "]" << endl;

  Mat p_GC_table = p_GC.reshape(0, rows);
  Mat p_GC_table_32F;
  p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);

  Mat img_GC;
  remap(img, img_GC, p_GC_table_32F, Mat(), INTER_LINEAR);

  cout << "Output BEV: " << img_GC.cols << "x" << img_GC.rows << " channels=" << img_GC.channels() << endl;
  return img_GC;
}

Mat Optimizer::generate_surround_view(Mat img_GF, Mat img_GL, Mat img_GB,
                                      Mat img_GR) {
  Mat dst1, dst2, dst3;
  addWeighted(img_GF, 0.5, img_GL, 0.5, 3, dst1);
  addWeighted(dst1, 1.0, img_GB, 0.5, 3, dst2);
  addWeighted(dst2, 1.0, img_GR, 0.5, 3, dst3);
  return dst3;
}

void Optimizer::Calibrate_left(int search_count, double roll_ep0,
                               double roll_ep1, double pitch_ep0,
                               double pitch_ep1, double yaw_ep0, double yaw_ep1,
                               double t0_ep0, double t0_ep1, double t1_ep0,
                               double t1_ep1, double t2_ep0, double t2_ep1) {
  vector<double> var(6, 0);
  string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};

  max_left_loss = cur_left_loss = CostFunction(var, "left");

  random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0, pitch_ep1,
                       yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0, t1_ep1, t2_ep0,
                       t2_ep1, "left");
}

void Optimizer::Calibrate_right(int search_count, double roll_ep0,
                                double roll_ep1, double pitch_ep0,
                                double pitch_ep1, double yaw_ep0,
                                double yaw_ep1, double t0_ep0, double t0_ep1,
                                double t1_ep0, double t1_ep1, double t2_ep0,
                                double t2_ep1) {
  vector<double> var(6, 0);
  string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  max_right_loss = cur_right_loss = CostFunction(var, "right");
  random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0, pitch_ep1,
                       yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0, t1_ep1, t2_ep0,
                       t2_ep1, "right");
}

void Optimizer::Calibrate_behind(int search_count, double roll_ep0,
                                 double roll_ep1, double pitch_ep0,
                                 double pitch_ep1, double yaw_ep0,
                                 double yaw_ep1, double t0_ep0, double t0_ep1,
                                 double t1_ep0, double t1_ep1, double t2_ep0,
                                 double t2_ep1) {
  vector<double> var(6, 0);
  string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};

  max_behind_loss = cur_behind_loss = CostFunction(var, "behind");

  random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0, pitch_ep1,
                       yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0, t1_ep1, t2_ep0,
                       t2_ep1, "behind");
}

void Optimizer::Calibrate_front(int search_count, double roll_ep0,
                                double roll_ep1, double pitch_ep0,
                                double pitch_ep1, double yaw_ep0,
                                double yaw_ep1, double t0_ep0, double t0_ep1,
                                double t1_ep0, double t1_ep1, double t2_ep0,
                                double t2_ep1) {
  vector<double> var(6, 0);
  string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};

  max_front_loss = cur_front_loss = CostFunction(var, "front");

  random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0, pitch_ep1,
                       yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0, t1_ep1, t2_ep0,
                       t2_ep1, "front");
}

void Optimizer::fine_Calibrate_left(int search_count, double roll_ep0,
                                    double roll_ep1, double pitch_ep0,
                                    double pitch_ep1, double yaw_ep0,
                                    double yaw_ep1, double t0_ep0,
                                    double t0_ep1, double t1_ep0, double t1_ep1,
                                    double t2_ep0, double t2_ep1) {
  fine_random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0,
                            pitch_ep1, yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0,
                            t1_ep1, t2_ep0, t2_ep1, "left");
}

void Optimizer::fine_Calibrate_right(
    int search_count, double roll_ep0, double roll_ep1, double pitch_ep0,
    double pitch_ep1, double yaw_ep0, double yaw_ep1, double t0_ep0,
    double t0_ep1, double t1_ep0, double t1_ep1, double t2_ep0, double t2_ep1) {
  fine_random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0,
                            pitch_ep1, yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0,
                            t1_ep1, t2_ep0, t2_ep1, "right");
}

void Optimizer::fine_Calibrate_behind(
    int search_count, double roll_ep0, double roll_ep1, double pitch_ep0,
    double pitch_ep1, double yaw_ep0, double yaw_ep1, double t0_ep0,
    double t0_ep1, double t1_ep0, double t1_ep1, double t2_ep0, double t2_ep1) {
  fine_random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0,
                            pitch_ep1, yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0,
                            t1_ep1, t2_ep0, t2_ep1, "behind");
}

void Optimizer::fine_Calibrate_front(
    int search_count, double roll_ep0, double roll_ep1, double pitch_ep0,
    double pitch_ep1, double yaw_ep0, double yaw_ep1, double t0_ep0,
    double t0_ep1, double t1_ep0, double t1_ep1, double t2_ep0, double t2_ep1) {
  fine_random_search_params(search_count, roll_ep0, roll_ep1, pitch_ep0,
                            pitch_ep1, yaw_ep0, yaw_ep1, t0_ep0, t0_ep1, t1_ep0,
                            t1_ep1, t2_ep0, t2_ep1, "front");
}

double Optimizer::CostFunction(const vector<double> var, string idx) {
  double loss;
  if (idx == "right") {
    Eigen::Matrix4d Tr = extrinsic_right;
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    Tr *= deltaT;
    if (fixed == "front")
      loss = back_camera_and_compute_loss(imgf_bev, imgr_gray, Tr, "fr", "right");
    else // fixed back,so rear bev pixels back projected to right camera
      loss = back_camera_and_compute_loss(imgb_bev, imgr_gray, Tr, "br", "right");
    return loss;
  } else if (idx == "left") {
    Eigen::Matrix4d Tl = extrinsic_left;
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    Tl *= deltaT;
    if (fixed == "front")
      loss = back_camera_and_compute_loss(imgf_bev, imgl_gray, Tl, "fl", "left");
    else // fixed back,so rear bev pixels back projected to left camera
      loss = back_camera_and_compute_loss(imgb_bev, imgl_gray, Tl, "bl", "left");
    return loss;
  } else { // behind(fist_order="front") or front(fist_order="behind")
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    if (fixed == "front") {
      Eigen::Matrix4d Tb = extrinsic_behind;
      Tb *= deltaT;
      loss = back_camera_and_compute_loss(imgl_bev, imgb_gray, Tb, "lb", "behind");
      loss += back_camera_and_compute_loss(imgr_bev, imgb_gray, Tb, "rb", "behind");
    } else { // fixed back,so left&right bev pixels back projected to front
             // camera at last
      Eigen::Matrix4d Tf = extrinsic_front;
      Tf *= deltaT;
      loss = back_camera_and_compute_loss(imgl_bev, imgf_gray, Tf, "lf", "front");
      loss += back_camera_and_compute_loss(imgr_bev, imgf_gray, Tf, "rf", "front");
      // cout<<loss<<endl;
    }
    return loss;
  }
}

double Optimizer::fine_CostFunction(const vector<double> var, string idx) {
  double loss;
  if (idx == "right") {
    Eigen::Matrix4d Tr = extrinsic_right_opt;
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    Tr *= deltaT;
    if (fixed == "front")
      loss = back_camera_and_compute_loss(imgf_bev, imgr_gray, Tr, "fr", "right");
    else // fixed back,so rear bev pixels back projected to right camera
      loss = back_camera_and_compute_loss(imgb_bev, imgr_gray, Tr, "br", "right");
    return loss;
  } else if (idx == "left") {
    Eigen::Matrix4d Tl = extrinsic_left_opt;
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    Tl *= deltaT;
    if (fixed == "front")
      loss = back_camera_and_compute_loss(imgf_bev, imgl_gray, Tl, "fl", "left");
    else // fixed back,so rear bev pixels back projected to left camera
      loss = back_camera_and_compute_loss(imgb_bev, imgl_gray, Tl, "bl", "left");
    return loss;
  } else { // behind(fist_order="front") or front(fist_order="behind")
    Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
    if (fixed == "front") {
      Eigen::Matrix4d Tb = extrinsic_behind_opt;
      Tb *= deltaT;
      loss = back_camera_and_compute_loss(imgl_bev, imgb_gray, Tb, "lb", "behind");
      loss += back_camera_and_compute_loss(imgr_bev, imgb_gray, Tb, "rb", "behind");
    } else { // fixed back,so left&right bev pixels back projected to front
             // camera at last
      Eigen::Matrix4d Tf = extrinsic_front_opt;
      Tf *= deltaT;
      loss = back_camera_and_compute_loss(imgl_bev, imgf_gray, Tf, "lf", "front");
      loss += back_camera_and_compute_loss(imgr_bev, imgf_gray, Tf, "rf", "front");
    }
    return loss;
  }
}

double Optimizer::back_camera_and_compute_loss(Mat img1, Mat img2,
                                               Eigen::Matrix4d T, string idx,
                                               const string &camera_idx) {
  vector<Point> pixels;
  Eigen::Matrix3d KC;
  vector<double> DC;
  Mat pG;
  Mat PG;
  Mat show;
  double ncoef;

  // Determine which camera we're projecting to based on idx
  string target_camera;
  if (idx == "fl") { // bev_front->camera_left
    target_camera = "left";
    DC = distortion_params_left;
    KC = intrinsic_left;
    pG = pG_fl;
    PG = PG_fl;
    pixels = fl_pixels_texture;
    ncoef = ncoef_fl;
  } else if (idx == "fr") { // bev_front->camera_right
    target_camera = "right";
    DC = distortion_params_right;
    KC = intrinsic_right;
    pG = pG_fr;
    PG = PG_fr;
    pixels = fr_pixels_texture;
    ncoef = ncoef_fr;
  } else if (idx == "lb") { // bev_left->camera_behind
    target_camera = "behind";
    DC = distortion_params_behind;
    KC = intrinsic_behind;
    pG = pG_bl;
    PG = PG_bl;
    pixels = bl_pixels_texture;
    ncoef = ncoef_bl;
  } else if (idx == "rb") { // bev_right->camera_behind
    target_camera = "behind";
    DC = distortion_params_behind;
    KC = intrinsic_behind;
    pG = pG_br;
    PG = PG_br;
    pixels = br_pixels_texture;
    ncoef = ncoef_br;
  } else if (idx == "lf") { // bev_left->camera_front
    target_camera = "front";
    DC = distortion_params_front;
    KC = intrinsic_front;
    pG = pG_fl;
    PG = PG_fl;
    pixels = fl_pixels_texture;
    ncoef = ncoef_fl;
  } else if (idx == "rf") { // bev_right->camera_front
    target_camera = "front";
    DC = distortion_params_front;
    KC = intrinsic_front;
    pG = pG_fr;
    PG = PG_fr;
    pixels = fr_pixels_texture;
    ncoef = ncoef_fr;
  } else if (idx == "br") { // bev_behind->camera_right
    target_camera = "right";
    DC = distortion_params_right;
    KC = intrinsic_right;
    pG = pG_br;
    PG = PG_br;
    pixels = br_pixels_texture;
    ncoef = ncoef_br;
  } else if (idx == "bl") { // bev_behind->camera_left
    target_camera = "left";
    DC = distortion_params_left;
    KC = intrinsic_left;
    pG = pG_bl;
    PG = PG_bl;
    pixels = bl_pixels_texture;
    ncoef = ncoef_bl;
  }

  // Check if pixels vector is empty
  if (pixels.empty()) {
    return INT_MAX;
  }

  int size = pixels.size();
  if (camera_model == 0)
    PG(Rect(0, 2, size, 1)) = cv::Mat::zeros(size, 1, CV_64FC1);
  double loss = 0;
  int failcount = 0;
  Mat PG2C = Mat::zeros(4, size, CV_64FC1);
  PG2C = eigen2mat(T) * PG;
  Mat PG2C1 = Mat::zeros(1, size, CV_64FC3);
  for (int i = 0; i < size; i++) {
    double x = PG2C.at<double>(0, i);
    double y = PG2C.at<double>(1, i);
    double z = PG2C.at<double>(2, i);
    PG2C1.at<Vec3d>(0, i) = Vec3d(x, y, z);
  }
  Mat pG2C(1, size, CV_64FC2);
  
  if (camera_model == 0) {
    fisheye::distortPoints(PG2C1, pG2C, eigen2mat(KC), DC); // fisheye
  } else if (camera_model == 1) {
    // Ocam - use camera-specific parameters
    const OcamParams *ocam = nullptr;
    if (target_camera == "front") ocam = &ocam_front;
    else if (target_camera == "left") ocam = &ocam_left;
    else if (target_camera == "behind") ocam = &ocam_behind;
    else if (target_camera == "right") ocam = &ocam_right;
    
    if (ocam && !ocam->world2cam_poly.empty()) {
      distortPointsOcamFull(PG2C1, pG2C, *ocam);
    } else {
      distortPointsOcam(PG2C1, pG2C, KC, DC); // Fallback
    }
  } else {
    distortPoints(PG2C1, pG2C, KC); // pinhole
  }
  for (int i = 0; i < size; i++) {
    double x = pG.at<double>(0, i);
    double y = pG.at<double>(1, i);
    double x1 = pG2C.at<Vec2d>(0, i)[0];
    double y1 = pG2C.at<Vec2d>(0, i)[1];
    // cout<<x1<<" "<<y1<<endl;
    if (x1 > 0 && y1 > 0 && x1 < img2.cols && y1 < img2.rows) {
      loss += pow(fabs(getPixelValue(&img1, x, y) / ncoef -
                       getPixelValue(&img2, x1, y1)),
                  2);
      // if(idx=="lb")
      // 	circle(show,Point(x1,y1),1,Scalar(0,255,0),-1);
    } else {
      failcount++;
      if (failcount > 30)
        return INT_MAX;
    }
  }

  return loss;
}

void Optimizer::random_search_params(int search_count, double roll_ep0,
                                     double roll_ep1, double pitch_ep0,
                                     double pitch_ep1, double yaw_ep0,
                                     double yaw_ep1, double t0_ep0,
                                     double t0_ep1, double t1_ep0,
                                     double t1_ep1, double t2_ep0,
                                     double t2_ep1, string idx) {
  vector<double> var(6, 0.0);
  double resolution_r = 100;
  double resolution_t = 100;

  random_device generator;
  std::uniform_int_distribution<int> distribution_roll(roll_ep0 * resolution_r,
                                                       roll_ep1 * resolution_r);
  std::uniform_int_distribution<int> distribution_pitch(
      pitch_ep0 * resolution_r, pitch_ep1 * resolution_r);
  std::uniform_int_distribution<int> distribution_yaw(yaw_ep0 * resolution_r,
                                                      yaw_ep1 * resolution_r);
  std::uniform_int_distribution<int> distribution_x(t0_ep0 * resolution_t,
                                                    t0_ep1 * resolution_t);
  std::uniform_int_distribution<int> distribution_y(t1_ep0 * resolution_t,
                                                    t1_ep1 * resolution_t);
  std::uniform_int_distribution<int> distribution_z(t2_ep0 * resolution_t,
                                                    t2_ep1 * resolution_t);

  for (size_t i = 0; i < search_count; i++) {
    mutexval.lock();
    var[0] = double(distribution_roll(generator)) / resolution_r;
    var[1] = double(distribution_pitch(generator)) / resolution_r;
    var[2] = double(distribution_yaw(generator)) / resolution_r;
    var[3] = double(distribution_x(generator)) / resolution_t;
    var[4] = double(distribution_y(generator)) / resolution_t;
    var[5] = double(distribution_z(generator)) / resolution_t;
    mutexval.unlock();

    double loss_new = CostFunction(var, idx);
    if (idx == "left" && loss_new < cur_left_loss) {
      lock_guard<std::mutex> lock(mutexleft);
      cur_left_loss = loss_new;
      extrinsic_left_opt = extrinsic_left * TransformUtil::GetDeltaT(var);
      bestVal_[0] = var;
    }
    if (idx == "right" && loss_new < cur_right_loss) {
      lock_guard<std::mutex> lock(mutexright);
      cur_right_loss = loss_new;
      extrinsic_right_opt = extrinsic_right * TransformUtil::GetDeltaT(var);
      bestVal_[1] = var;
    }
    if (idx == "behind" && loss_new < cur_behind_loss) {
      lock_guard<std::mutex> lock(mutexbehind);
      cur_behind_loss = loss_new;
      extrinsic_behind_opt = extrinsic_behind * TransformUtil::GetDeltaT(var);
      bestVal_[2] = var;
    }
    if (idx == "front" && loss_new < cur_front_loss) { // if fix back
                                                       // camera,front camera is
                                                       // calibrated at last
      lock_guard<std::mutex> lock(mutexfront);
      cur_front_loss = loss_new;
      extrinsic_front_opt = extrinsic_front * TransformUtil::GetDeltaT(var);
      bestVal_[2] = var;
    }
  }

  if (idx == "left") {
    imgl_bev = project_on_ground(imgl_gray, extrinsic_left_opt, intrinsic_left,
                                 distortion_params_left, KG, brows, bcols, hl, "left");
    imgl_bev_rgb =
        project_on_ground(imgl_rgb, extrinsic_left_opt, intrinsic_left,
                          distortion_params_left, KG, brows, bcols, hl, "left");
    imgl_bev = tail(imgl_bev, "l");
    imgl_bev_rgb = tail(imgl_bev_rgb, "l");
  } else if (idx == "right") {
    imgr_bev =
        project_on_ground(imgr_gray, extrinsic_right_opt, intrinsic_right,
                          distortion_params_right, KG, brows, bcols, hr, "right");
    imgr_bev_rgb =
        project_on_ground(imgr_rgb, extrinsic_right_opt, intrinsic_right,
                          distortion_params_right, KG, brows, bcols, hr, "right");
    imgr_bev = tail(imgr_bev, "r");
    imgr_bev_rgb = tail(imgr_bev_rgb, "r");
  } else if (idx == "behind") {
    imgb_bev =
        project_on_ground(imgb_gray, extrinsic_behind_opt, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev_rgb =
        project_on_ground(imgb_rgb, extrinsic_behind_opt, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev = tail(imgb_bev, "b");
    imgb_bev_rgb = tail(imgb_bev_rgb, "b");
  } else { // if fix back camera,front camera is calibrated at last
    imgf_bev =
        project_on_ground(imgf_gray, extrinsic_front_opt, intrinsic_front,
                          distortion_params_front, KG, brows, bcols, hf, "front");
    imgf_bev_rgb =
        project_on_ground(imgf_rgb, extrinsic_front_opt, intrinsic_front,
                          distortion_params_front, KG, brows, bcols, hb, "front");
    imgf_bev = tail(imgf_bev, "f");
    imgf_bev_rgb = tail(imgf_bev_rgb, "f");
  }
}

void Optimizer::fine_random_search_params(int search_count, double roll_ep0,
                                          double roll_ep1, double pitch_ep0,
                                          double pitch_ep1, double yaw_ep0,
                                          double yaw_ep1, double t0_ep0,
                                          double t0_ep1, double t1_ep0,
                                          double t1_ep1, double t2_ep0,
                                          double t2_ep1, string idx) {
  vector<double> var(6, 0.0);

  random_device generator;
  std::uniform_real_distribution<double> distribution_roll(roll_ep0, roll_ep1);
  std::uniform_real_distribution<double> distribution_pitch(pitch_ep0,
                                                            pitch_ep1);
  std::uniform_real_distribution<double> distribution_yaw(yaw_ep0, yaw_ep1);
  std::uniform_real_distribution<double> distribution_x(t0_ep0, t0_ep1);
  std::uniform_real_distribution<double> distribution_y(t1_ep0, t1_ep1);
  std::uniform_real_distribution<double> distribution_z(t2_ep0, t2_ep1);

  if (!coarse_flag) {
    extrinsic_left_opt = extrinsic_left;
    extrinsic_right_opt = extrinsic_right;
    extrinsic_behind_opt = extrinsic_behind;
    extrinsic_front_opt = extrinsic_front;
  }

  for (size_t i = 0; i < search_count; i++) {
    mutexval.lock();
    var[0] = double(distribution_roll(generator));
    var[1] = double(distribution_pitch(generator));
    var[2] = double(distribution_yaw(generator));
    var[3] = double(distribution_x(generator));
    var[4] = double(distribution_y(generator));
    var[5] = double(distribution_z(generator));
    mutexval.unlock();
    double loss_new = fine_CostFunction(var, idx);
    if (idx == "left" && loss_new < cur_left_loss) {
      lock_guard<std::mutex> lock(mutexleft);
      cur_left_loss = loss_new;
      extrinsic_left_opt = extrinsic_left_opt * TransformUtil::GetDeltaT(var);
      for (int i = 0; i < 6; i++) {
        bestVal_[0][i] += var[i];
      }
    }
    if (idx == "right" && loss_new < cur_right_loss) {
      lock_guard<std::mutex> lock(mutexright);
      cur_right_loss = loss_new;
      extrinsic_right_opt = extrinsic_right_opt * TransformUtil::GetDeltaT(var);
      for (int i = 0; i < 6; i++) {
        bestVal_[1][i] += var[i];
      }
    }
    if (idx == "behind" && loss_new < cur_behind_loss) {
      lock_guard<std::mutex> lock(mutexbehind);
      cur_behind_loss = loss_new;
      extrinsic_behind_opt =
          extrinsic_behind_opt * TransformUtil::GetDeltaT(var);
      for (int i = 0; i < 6; i++) {
        bestVal_[2][i] += var[i];
      }
    }
    if (idx == "front" && loss_new < cur_front_loss) { // if fix back
                                                       // camera,front camera is
                                                       // calibrated at last
      lock_guard<std::mutex> lock(mutexfront);
      cur_front_loss = loss_new;
      extrinsic_front_opt = extrinsic_front_opt * TransformUtil::GetDeltaT(var);
      for (int i = 0; i < 6; i++) {
        bestVal_[2][i] += var[i];
      }
    }
  }

  if (idx == "left") {
    imgl_bev = project_on_ground(imgl_gray, extrinsic_left_opt, intrinsic_left,
                                 distortion_params_left, KG, brows, bcols, hl, "left");
    imgl_bev_rgb =
        project_on_ground(imgl_rgb, extrinsic_left_opt, intrinsic_left,
                          distortion_params_left, KG, brows, bcols, hl, "left");
    imgl_bev = tail(imgl_bev, "l");
    imgl_bev_rgb = tail(imgl_bev_rgb, "l");
  } else if (idx == "right") {
    imgr_bev =
        project_on_ground(imgr_gray, extrinsic_right_opt, intrinsic_right,
                          distortion_params_right, KG, brows, bcols, hr, "right");
    imgr_bev_rgb =
        project_on_ground(imgr_rgb, extrinsic_right_opt, intrinsic_right,
                          distortion_params_right, KG, brows, bcols, hr, "right");
    imgr_bev = tail(imgr_bev, "r");
    imgr_bev_rgb = tail(imgr_bev_rgb, "r");
  } else if (idx == "behind") {
    imgb_bev =
        project_on_ground(imgb_gray, extrinsic_behind_opt, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev_rgb =
        project_on_ground(imgb_rgb, extrinsic_behind_opt, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev = tail(imgb_bev, "b");
    imgb_bev_rgb = tail(imgb_bev_rgb, "b");
  } else { // if fix back camera,front camera is calibrated at last
    imgf_bev =
        project_on_ground(imgf_gray, extrinsic_front_opt, intrinsic_front,
                          distortion_params_front, KG, brows, bcols, hf, "front");
    imgf_bev_rgb =
        project_on_ground(imgf_rgb, extrinsic_front_opt, intrinsic_front,
                          distortion_params_front, KG, brows, bcols, hb, "front");
    imgf_bev = tail(imgf_bev, "f");
    imgf_bev_rgb = tail(imgf_bev_rgb, "f");
  }
}

Optimizer::OcamParams Optimizer::loadSingleOcamParam(const string &calib_file) {
  OcamParams params;

  std::ifstream file(calib_file);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open calibration file: " + calib_file);
  }

  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(file, root)) {
    throw std::runtime_error("Failed to parse JSON from " + calib_file);
  }

  const Json::Value &intrinsic = root["intrinsic_param"];

  // Basic parameters
  params.width = intrinsic["camera_width"].asInt();
  params.height = intrinsic["camera_height"].asInt();
  params.cx = intrinsic["principal_point"][0].asDouble();
  params.cy = intrinsic["principal_point"][1].asDouble();

  // Affine parameters
  params.c = intrinsic["affine_c"].asDouble();
  params.d = intrinsic["affine_d"].asDouble();
  params.e = intrinsic["affine_e"].asDouble();

  // Polynomial coefficients
  const Json::Value &cam2world = intrinsic["cam2world"];
  const Json::Value &world2cam = intrinsic["world2cam"];

  for (int i = 0; i < cam2world.size(); i++) {
    params.cam2world_poly.push_back(cam2world[i].asDouble());
  }
  for (int i = 0; i < world2cam.size(); i++) {
    params.world2cam_poly.push_back(world2cam[i].asDouble());
  }

  // Affine transformation matrix
  params.A << 1.0, params.e, params.d, params.c;
  params.B << params.cx, params.cy;

  // Pre-compute polynomial coefficients for efficient evaluation
  // Convert from [a0, a1, a2, ...] to [..., a2, a1, a0] for Horner's method
  params.world2cam_coeffs.resize(params.world2cam_poly.size());
  for (size_t i = 0; i < params.world2cam_poly.size(); i++) {
    params.world2cam_coeffs(i) = params.world2cam_poly[params.world2cam_poly.size() - 1 - i];
  }

  return params;
}

void Optimizer::loadOcamParams(const string &calib_file_front,
                               const string &calib_file_left,
                               const string &calib_file_behind,
                               const string &calib_file_right) {
  cout << "Loading Ocam calibration parameters..." << endl;

  ocam_front = loadSingleOcamParam(calib_file_front);
  ocam_left = loadSingleOcamParam(calib_file_left);
  ocam_behind = loadSingleOcamParam(calib_file_behind);
  ocam_right = loadSingleOcamParam(calib_file_right);

  cout << "Front camera: " << ocam_front.width << "x" << ocam_front.height << endl;
  cout << "Left camera: " << ocam_left.width << "x" << ocam_left.height << endl;
  cout << "Behind camera: " << ocam_behind.width << "x" << ocam_behind.height << endl;
  cout << "Right camera: " << ocam_right.width << "x" << ocam_right.height << endl;
}

// Convert Rodrigues vector to rotation matrix
Eigen::Matrix3d Optimizer::rodriguesToRotationMatrix(const vector<double> &rvec) {
  double theta = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
  
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity();
  }
  
  double r[3] = {rvec[0] / theta, rvec[1] / theta, rvec[2] / theta};
  
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);
  double one_minus_cos = 1.0 - cos_theta;
  
  // Rodrigues formula: R = cosθ*I + (1-cosθ)*r*r^T + sinθ*[r]_x
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

void Optimizer::loadExtrinsicsFromJson(const string &filename) {
  cout << "\n==========================================" << endl;
  cout << "Loading extrinsics from: " << filename << endl;
  cout << "==========================================" << endl;
  
  std::ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Warning: Cannot open extrinsics file: " << filename << endl;
    cerr << "Using default extrinsics..." << endl;
    return;
  }
  
  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(file, root)) {
    cerr << "Warning: Failed to parse extrinsics JSON from " << filename << endl;
    cerr << "Using default extrinsics..." << endl;
    return;
  }
  
  const Json::Value &extrinsics = root["extrinsic_param"];
  
  // Camera name mapping
  vector<pair<string, string>> camera_mappings = {
    {"park_front", "front"},
    {"park_left", "left"},
    {"park_back", "behind"},
    {"park_right", "right"}
  };
  
  for (const auto &mapping : camera_mappings) {
    const string &json_name = mapping.first;
    const string &cam_name = mapping.second;
    
    if (extrinsics.isMember(json_name)) {
      const Json::Value &cam_ext = extrinsics[json_name];
      
      // Get rotation vector and translation
      // NOTE: Extrinsics file contains camera-to-body transform (rotation/translation)
      vector<double> rvec(3);
      for (int i = 0; i < 3; i++) {
        rvec[i] = cam_ext["rotation"][i].asDouble();
      }
      
      Eigen::Vector3d translation_vec;
      for (int i = 0; i < 3; i++) {
        translation_vec[i] = cam_ext["translation"][i].asDouble();
      }
      
      // CPAC approach: 
      // 1. rvec/translation = camera-to-body transform (from extrinsics file)
      // 2. Build T_cam_to_body = [R_cam_to_body | t_cam_in_body]
      // 3. T_vehicle_to_camera = T_body_to_cam = (T_cam_to_body)^-1
      
      // Convert Rodrigues vector to rotation matrix (camera-to-body rotation)
      Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
      
      // Build T_cam_to_body (4x4)
      Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
      T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
      T_cam_to_body.block<3, 1>(0, 3) = translation_vec;  // camera position in body frame
      
      // Take inverse to get T_body_to_cam (vehicle-to-camera)
      // This is what CPAC does: T_vehicle_to_camera = np.linalg.inv(T_cam_to_body)
      Eigen::Matrix4d T_body_to_cam = T_cam_to_body.inverse();
      
      // Store in appropriate variable
      // Camera height is |translation_vec[2]| (z-component of camera in body frame)
      double camera_height = fabs(translation_vec[2]);
      
      if (cam_name == "front") {
        extrinsic_front = T_body_to_cam;
        extrinsic_front_initial = T_body_to_cam;
        hf = camera_height;
        cout << "Loaded front extrinsics:" << endl;
      } else if (cam_name == "left") {
        extrinsic_left = T_body_to_cam;
        extrinsic_left_initial = T_body_to_cam;
        hl = camera_height;
        cout << "Loaded left extrinsics:" << endl;
      } else if (cam_name == "behind") {
        extrinsic_behind = T_body_to_cam;
        extrinsic_behind_initial = T_body_to_cam;
        hb = camera_height;
        cout << "Loaded behind extrinsics:" << endl;
      } else if (cam_name == "right") {
        extrinsic_right = T_body_to_cam;
        extrinsic_right_initial = T_body_to_cam;
        hr = camera_height;
        cout << "Loaded right extrinsics:" << endl;
      }
      
      cout << "  Rotation (Euler): " << TransformUtil::Rotation2Eul(T_body_to_cam.block<3,3>(0,0)).transpose() << endl;
      cout << "  Translation: [" << T_body_to_cam.block<3,1>(0,3).transpose() << "]" << endl;
      cout << "  Camera height from extrinsics: " << camera_height << " m" << endl;
    }
  }
  
  cout << "==========================================" << endl;
  
  // After loading extrinsics, recompute initial BEV images if needed
  if (fixed == "front") {
    imgf_bev = project_on_ground(imgf_gray, extrinsic_front, intrinsic_front,
                                 distortion_params_front, KG, brows, bcols, hf, "front");
    imgf_bev_rgb = project_on_ground(imgf_rgb, extrinsic_front, intrinsic_front,
                          distortion_params_front, KG, brows, bcols, hf, "front");
    imgf_bev = tail(imgf_bev, "f");
    imgf_bev_rgb = tail(imgf_bev_rgb, "f");
  } else {
    imgb_bev = project_on_ground(imgb_gray, extrinsic_behind, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev_rgb = project_on_ground(imgb_rgb, extrinsic_behind, intrinsic_behind,
                          distortion_params_behind, KG, brows, bcols, hb, "behind");
    imgb_bev = tail(imgb_bev, "b");
    imgb_bev_rgb = tail(imgb_bev_rgb, "b");
  }
}

void Optimizer::saveExtrinsicsToJson(const string &filename, const string &camera_name,
                                     const Eigen::Matrix4d &extrinsic) {
  // Extract rotation matrix and translation vector
  Eigen::Matrix3d rotation = extrinsic.block<3, 3>(0, 0);
  Eigen::Vector3d translation = extrinsic.block<3, 1>(0, 3);
  
  // Convert rotation matrix to Rodrigues vector
  double theta = acos((rotation.trace() - 1.0) / 2.0);
  Eigen::Vector3d rvec;
  
  if (theta < 1e-8) {
    rvec = Eigen::Vector3d::Zero();
  } else {
    rvec << rotation(2, 1) - rotation(1, 2),
            rotation(0, 2) - rotation(2, 0),
            rotation(1, 0) - rotation(0, 1);
    rvec *= theta / (2.0 * sin(theta));
  }
  
  // Read existing file or create new
  Json::Value root;
  std::ifstream infile(filename);
  if (infile.is_open()) {
    Json::Reader reader;
    reader.parse(infile, root, false);
  } else {
    root["extrinsic_param"] = Json::Value(Json::objectValue);
  }
  infile.close();
  
  // Add/update camera extrinsics
  root["extrinsic_param"][camera_name] = Json::Value(Json::objectValue);
  Json::Value &cam_ext = root["extrinsic_param"][camera_name];
  
  // Add rotation vector
  cam_ext["rotation"] = Json::Value(Json::arrayValue);
  for (int i = 0; i < 3; i++) {
    cam_ext["rotation"].append(rvec[i]);
  }
  
  // Add translation vector
  cam_ext["translation"] = Json::Value(Json::arrayValue);
  for (int i = 0; i < 3; i++) {
    cam_ext["translation"].append(translation[i]);
  }
  
  // Write to file
  std::ofstream outfile(filename);
  if (!outfile.is_open()) {
    cerr << "Error: Cannot open file for writing: " << filename << endl;
    return;
  }
  
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "    ";
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &outfile);
  outfile.close();
  
  cout << "Saved extrinsics for " << camera_name << " to " << filename << endl;
}