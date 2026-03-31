/*
 * Front 3 Cameras Calibration Quality Metrics - Implementation
 */
#include "calibration_metrics.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace CalibrationMetrics {

// ==================== Utils实现 ====================
namespace Utils {

Mat computeOverlapMask(const Mat& mask1, const Mat& mask2) {
    Mat overlap;
    bitwise_and(mask1, mask2, overlap);
    return overlap;
}

Mat computeValidMask(const Mat& img) {
    Mat gray, mask;
    if (img.channels() == 3) {
        cvtColor(img, gray, COLOR_BGR2GRAY);
    } else {
        gray = img;
    }
    threshold(gray, mask, 10, 255, THRESH_BINARY);
    return mask;
}

double computeMSE(const Mat& img1, const Mat& img2, const Mat& mask) {
    Mat diff;
    absdiff(img1, img2, diff);
    diff = diff.mul(diff);
    diff.convertTo(diff, CV_64F);

    double sum = 0;
    int count = 0;
    for (int y = 0; y < diff.rows; y++) {
        for (int x = 0; x < diff.cols; x++) {
            if (mask.at<uchar>(y, x) > 0) {
                if (diff.channels() == 1) {
                    sum += diff.at<double>(y, x);
                } else {
                    Vec3d val = diff.at<Vec3d>(y, x);
                    sum += (val[0] + val[1] + val[2]) / 3.0;
                }
                count++;
            }
        }
    }
    return count > 0 ? sum / count : 0;
}

double computeMAE(const Mat& img1, const Mat& img2, const Mat& mask) {
    Mat diff;
    absdiff(img1, img2, diff);

    double sum = 0;
    int count = 0;
    for (int y = 0; y < diff.rows; y++) {
        for (int x = 0; x < diff.cols; x++) {
            if (mask.at<uchar>(y, x) > 0) {
                if (diff.channels() == 1) {
                    sum += diff.at<uchar>(y, x);
                } else {
                    Vec3b val = diff.at<Vec3b>(y, x);
                    sum += (val[0] + val[1] + val[2]) / 3.0;
                }
                count++;
            }
        }
    }
    return count > 0 ? sum / count : 0;
}

double computeHistogramSimilarity(const Mat& img1, const Mat& img2,
                                  const Mat& mask) {
    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // 计算直方图
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    Mat hist1, hist2;

    calcHist(&gray1, 1, 0, mask, hist1, 1, &histSize, &histRange);
    calcHist(&gray2, 1, 0, mask, hist2, 1, &histSize, &histRange);

    // 归一化
    normalize(hist1, hist1, 0, 1, NORM_MINMAX);
    normalize(hist2, hist2, 0, 1, NORM_MINMAX);

    // 计算相关性
    return compareHist(hist1, hist2, HISTCMP_CORREL);
}

Mat normalizeImage(const Mat& img) {
    Mat norm_img;
    img.convertTo(norm_img, CV_64F, 1.0 / 255.0);
    return norm_img;
}

} // namespace Utils

// ==================== PhotometricMetric实现 ====================
double PhotometricMetric::compute(const Mat& img1, const Mat& img2,
                                  const Mat& mask1, const Mat& mask2,
                                  Mat& overlap_mask) {
    overlap_mask = Utils::computeOverlapMask(mask1, mask2);

    // 转换为灰度图
    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // 计算光度损失 (L2范数)
    double loss = 0;
    int count = 0;

    for (int y = 0; y < gray1.rows; y++) {
        for (int x = 0; x < gray1.cols; x++) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
                double diff = (double)gray1.at<uchar>(y, x) -
                             (double)gray2.at<uchar>(y, x);
                loss += diff * diff;
                count++;
            }
        }
    }

    return count > 0 ? sqrt(loss / count) : 0;  // RMSE
}

double PhotometricMetric::normalizeScore(double loss) {
    // 经验阈值：
    // loss < 5: Excellent (90-100)
    // loss 5-15: Good (70-90)
    // loss 15-30: Fair (40-70)
    // loss > 30: Poor (<40)
    if (loss < 1) return 100;
    double score = 100 * exp(-loss / 20.0);
    return max(0.0, min(100.0, score));
}

// ==================== SSIMMetric实现 ====================
double SSIMMetric::compute(const Mat& img1, const Mat& img2,
                           const Mat& overlap_mask) {
    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // SSIM参数
    const double C1 = 6.5025, C2 = 58.5225;

    Mat I1, I2;
    gray1.convertTo(I1, CV_64F);
    gray2.convertTo(I2, CV_64F);

    Mat I1_2 = I1.mul(I1);
    Mat I2_2 = I2.mul(I2);
    Mat I1_I2 = I1.mul(I2);

    Mat mu1, mu2;
    GaussianBlur(I1, mu1, Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, Size(11, 11), 1.5);

    Mat mu1_2 = mu1.mul(mu1);
    Mat mu2_2 = mu2.mul(mu2);
    Mat mu1_mu2 = mu1.mul(mu2);

    Mat sigma1_2, sigma2_2, sigma12;
    GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    Mat t1, t2, t3;
    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);

    Mat ssim_map;
    divide(t3, t1, ssim_map);

    // 只计算重叠区域
    double ssim_sum = 0;
    int count = 0;
    for (int y = 0; y < ssim_map.rows; y++) {
        for (int x = 0; x < ssim_map.cols; x++) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
                ssim_sum += ssim_map.at<double>(y, x);
                count++;
            }
        }
    }

    return count > 0 ? ssim_sum / count : 0;
}

double SSIMMetric::normalizeScore(double ssim) {
    // SSIM范围[-1, 1]，通常[0.5, 1]表示较好质量
    // 映射到0-100
    return max(0.0, min(100.0, (ssim + 1) * 50));
}

// ==================== GradientMetric实现 ====================
double GradientMetric::compute(const Mat& img1, const Mat& img2,
                               const Mat& overlap_mask) {
    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // 计算Sobel梯度
    Mat grad1_x, grad1_y, grad2_x, grad2_y;
    Sobel(gray1, grad1_x, CV_64F, 1, 0, 3);
    Sobel(gray1, grad1_y, CV_64F, 0, 1, 3);
    Sobel(gray2, grad2_x, CV_64F, 1, 0, 3);
    Sobel(gray2, grad2_y, CV_64F, 0, 1, 3);

    // 计算梯度差异
    double diff_sum = 0;
    int count = 0;

    for (int y = 0; y < gray1.rows; y++) {
        for (int x = 0; x < gray1.cols; x++) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
                double gx1 = grad1_x.at<double>(y, x);
                double gy1 = grad1_y.at<double>(y, x);
                double gx2 = grad2_x.at<double>(y, x);
                double gy2 = grad2_y.at<double>(y, x);

                double mag1 = sqrt(gx1*gx1 + gy1*gy1);
                double mag2 = sqrt(gx2*gx2 + gy2*gy2);

                diff_sum += abs(mag1 - mag2);
                count++;
            }
        }
    }

    return count > 0 ? diff_sum / count : 0;
}

double GradientMetric::computeEdgeAlignment(const Mat& img1, const Mat& img2,
                                            const Mat& overlap_mask) {
    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // Canny边缘检测
    Mat edge1, edge2;
    Canny(gray1, edge1, 50, 150);
    Canny(gray2, edge2, 50, 150);

    // 只保留重叠区域的边缘
    Mat overlap_edges1, overlap_edges2;
    bitwise_and(edge1, overlap_mask, overlap_edges1);
    bitwise_and(edge2, overlap_mask, overlap_edges2);

    // 计算边缘匹配度
    Mat intersection, union_img;
    bitwise_and(overlap_edges1, overlap_edges2, intersection);
    bitwise_or(overlap_edges1, overlap_edges2, union_img);

    double inter_area = countNonZero(intersection);
    double union_area = countNonZero(union_img);

    return union_area > 0 ? inter_area / union_area : 0;
}

// ==================== FeatureMetric实现 ====================
FeatureMetric::FeatureMatchResult FeatureMetric::compute(
    const Mat& img1, const Mat& img2, const Mat& overlap_mask) {

    FeatureMatchResult result = {0, 0, 0, 0, 0};

    Mat gray1, gray2;
    if (img1.channels() == 3) cvtColor(img1, gray1, COLOR_BGR2GRAY);
    else gray1 = img1;
    if (img2.channels() == 3) cvtColor(img2, gray2, COLOR_BGR2GRAY);
    else gray2 = img2;

    // 只处理重叠区域
    Mat masked1, masked2;
    gray1.copyTo(masked1, overlap_mask);
    gray2.copyTo(masked2, overlap_mask);

    // 使用ORB特征
    Ptr<ORB> orb = ORB::create(500);
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    orb->detectAndCompute(masked1, noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(masked2, noArray(), keypoints2, descriptors2);

    if (keypoints1.empty() || keypoints2.empty() ||
        descriptors1.empty() || descriptors2.empty()) {
        return result;
    }

    // 特征匹配
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> knn_matches;
    matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);

    // 筛选好的匹配
    vector<DMatch> good_matches;
    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i].size() >= 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    result.num_matches = (int)good_matches.size();

    if (good_matches.size() < 4) {
        return result;
    }

    // 使用RANSAC计算内点
    vector<Point2f> points1, points2;
    for (const auto& match : good_matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    vector<uchar> inliers;
    Mat H = findHomography(points1, points2, RANSAC, 3.0, inliers);

    result.num_inliers = countNonZero(inliers);
    result.inlier_ratio = (double)result.num_inliers / good_matches.size();

    // 计算平均距离
    double dist_sum = 0;
    double max_dist = 0;
    int valid_count = 0;
    for (size_t i = 0; i < good_matches.size(); i++) {
        if (inliers[i]) {
            Point2f p1 = points1[i];
            Point2f p2 = points2[i];
            double dist = norm(p1 - p2);
            dist_sum += dist;
            max_dist = max(max_dist, dist);
            valid_count++;
        }
    }

    result.mean_distance = valid_count > 0 ? dist_sum / valid_count : 0;
    result.max_distance = max_dist;

    return result;
}

double FeatureMetric::normalizeScore(const FeatureMatchResult& result) {
    // 基于内点数量和比率计算分数
    if (result.num_inliers < 10) return 0;

    double score = result.inlier_ratio * 50 +
                   min(50.0, result.num_inliers / 2.0);

    return min(100.0, score);
}

// ==================== CalibrationEvaluator实现 ====================
EvaluationReport CalibrationEvaluator::evaluateCameraPair(
    const Mat& img1, const Mat& img2,
    const Mat& mask1, const Mat& mask2,
    const string& pair_name) {

    EvaluationReport report;
    report.metrics.clear();

    Mat overlap_mask;

    // 1. 光度一致性
    double photo_loss = PhotometricMetric::compute(img1, img2, mask1, mask2, overlap_mask);
    MetricResult photo_metric;
    photo_metric.name = pair_name + " Photometric Loss";
    photo_metric.value = photo_loss;
    photo_metric.normalized_score = PhotometricMetric::normalizeScore(photo_loss);
    photo_metric.description = "重叠区域像素灰度差异 (RMSE)";
    if (photo_metric.normalized_score > 80) photo_metric.status = "Excellent";
    else if (photo_metric.normalized_score > 60) photo_metric.status = "Good";
    else if (photo_metric.normalized_score > 40) photo_metric.status = "Fair";
    else photo_metric.status = "Poor";
    report.metrics.push_back(photo_metric);

    // 2. SSIM
    double ssim = SSIMMetric::compute(img1, img2, overlap_mask);
    MetricResult ssim_metric;
    ssim_metric.name = pair_name + " SSIM";
    ssim_metric.value = ssim;
    ssim_metric.normalized_score = SSIMMetric::normalizeScore(ssim);
    ssim_metric.description = "结构相似性指数 [-1, 1]";
    if (ssim_metric.normalized_score > 85) ssim_metric.status = "Excellent";
    else if (ssim_metric.normalized_score > 70) ssim_metric.status = "Good";
    else if (ssim_metric.normalized_score > 50) ssim_metric.status = "Fair";
    else ssim_metric.status = "Poor";
    report.metrics.push_back(ssim_metric);

    // 3. 梯度一致性
    double gradient = GradientMetric::compute(img1, img2, overlap_mask);
    double edge_align = GradientMetric::computeEdgeAlignment(img1, img2, overlap_mask);
    MetricResult grad_metric;
    grad_metric.name = pair_name + " Edge Alignment";
    grad_metric.value = edge_align;
    grad_metric.normalized_score = edge_align * 100;
    grad_metric.description = "边缘对齐度 (IoU)";
    if (grad_metric.normalized_score > 75) grad_metric.status = "Excellent";
    else if (grad_metric.normalized_score > 55) grad_metric.status = "Good";
    else if (grad_metric.normalized_score > 35) grad_metric.status = "Fair";
    else grad_metric.status = "Poor";
    report.metrics.push_back(grad_metric);

    // 4. 特征点匹配
    FeatureMetric::FeatureMatchResult feature_result = FeatureMetric::compute(img1, img2, overlap_mask);
    MetricResult feature_metric;
    feature_metric.name = pair_name + " Feature Matching";
    feature_metric.value = feature_result.num_inliers;
    feature_metric.normalized_score = FeatureMetric::normalizeScore(feature_result);
    feature_metric.description = "RANSAC内点数量: " + to_string(feature_result.num_inliers) +
                                  "/" + to_string(feature_result.num_matches) +
                                  " (ratio: " + to_string((int)(feature_result.inlier_ratio * 100)) + "%)";
    if (feature_metric.normalized_score > 75) feature_metric.status = "Excellent";
    else if (feature_metric.normalized_score > 55) feature_metric.status = "Good";
    else if (feature_metric.normalized_score > 35) feature_metric.status = "Fair";
    else feature_metric.status = "Poor";
    report.metrics.push_back(feature_metric);

    // 计算综合得分
    double total_score = 0;
    for (const auto& m : report.metrics) {
        total_score += m.normalized_score;
    }
    report.overall_score = total_score / report.metrics.size();

    // 生成总结
    stringstream ss;
    ss << fixed << setprecision(1);
    ss << pair_name << " Pair Evaluation: ";
    if (report.overall_score >= 80) ss << "EXCELLENT";
    else if (report.overall_score >= 60) ss << "GOOD";
    else if (report.overall_score >= 40) ss << "FAIR";
    else ss << "POOR - REQUIRES ADJUSTMENT";
    ss << " (Score: " << report.overall_score << ")";
    report.summary = ss.str();

    return report;
}

string CalibrationEvaluator::generateReportText(const EvaluationReport& report) {
    stringstream ss;
    ss << "\n========================================\n";
    ss << "      Calibration Quality Report        \n";
    ss << "========================================\n";
    ss << report.summary << "\n";
    ss << "----------------------------------------\n";

    for (const auto& m : report.metrics) {
        ss << "\n" << m.name << ":\n";
        ss << "  Value: " << fixed << setprecision(3) << m.value << "\n";
        ss << "  Score: " << setprecision(1) << m.normalized_score << "/100\n";
        ss << "  Status: " << m.status << "\n";
        ss << "  Description: " << m.description << "\n";
    }

    ss << "\n========================================\n";
    ss << "Overall Score: " << fixed << setprecision(1) << report.overall_score << "/100\n";
    ss << "========================================\n";

    return ss.str();
}

Mat RealtimeMetricsDisplay::createMetricsPanel(const vector<MetricResult>& metrics,
                                               int width, int height) {
    Mat panel(height, width, CV_8UC3, Scalar(40, 40, 40));

    int bar_height = (height - 60) / max(1, (int)metrics.size());
    int bar_max_width = width - 250;

    for (size_t i = 0; i < metrics.size(); i++) {
        int y = 30 + i * bar_height;

        // 绘制指标名称
        putText(panel, metrics[i].name, Point(10, y + bar_height/2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

        // 绘制进度条背景
        rectangle(panel, Point(150, y + 5), Point(150 + bar_max_width, y + bar_height - 5),
                 Scalar(60, 60, 60), -1);

        // 确定颜色
        Scalar color;
        if (metrics[i].normalized_score >= 80) color = Scalar(0, 255, 0);  // 绿色
        else if (metrics[i].normalized_score >= 60) color = Scalar(0, 255, 255);  // 黄色
        else if (metrics[i].normalized_score >= 40) color = Scalar(0, 128, 255);  // 橙色
        else color = Scalar(0, 0, 255);  // 红色

        // 绘制进度条
        int bar_width = (int)(metrics[i].normalized_score / 100.0 * bar_max_width);
        rectangle(panel, Point(150, y + 5), Point(150 + bar_width, y + bar_height - 5),
                 color, -1);

        // 绘制数值
        stringstream ss;
        ss << fixed << setprecision(1) << metrics[i].normalized_score;
        putText(panel, ss.str() + " (" + metrics[i].status + ")",
                Point(160 + bar_max_width, y + bar_height/2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    }

    return panel;
}

} // namespace CalibrationMetrics
