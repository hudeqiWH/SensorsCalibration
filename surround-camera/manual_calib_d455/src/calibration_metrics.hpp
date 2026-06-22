/*
 * Front 3 Cameras Calibration Quality Metrics
 * 前视3相机标定质量评估指标
 */
#ifndef CALIBRATION_METRICS_HPP
#define CALIBRATION_METRICS_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

namespace CalibrationMetrics {

// 评估结果结构体
struct MetricResult {
    string name;
    double value;
    double normalized_score;  // 0-100, 越高越好
    string description;
    string status;  // "Excellent", "Good", "Fair", "Poor"
};

// 综合评估报告
struct EvaluationReport {
    vector<MetricResult> metrics;
    double overall_score;  // 综合得分 0-100
    string summary;
    Mat visualization;  // 可视化结果
};

// ==================== 1. 光度一致性损失 ====================
// Photometric Consistency Loss (基于自动标定的方法)
// 计算重叠区域像素的灰度差异
class PhotometricMetric {
public:
    // 计算两个图像重叠区域的光度损失
    // img1, img2: 输入图像 (已对齐到同一坐标系)
    // mask1, mask2: 有效区域掩码
    // overlap_mask: 重叠区域掩码 (输出)
    static double compute(const Mat& img1, const Mat& img2,
                         const Mat& mask1, const Mat& mask2,
                         Mat& overlap_mask);

    // 归一化到0-100分数 (越高越好)
    static double normalizeScore(double loss);
};

// ==================== 2. 结构相似性指数 (SSIM) ====================
// Structural Similarity Index
// 评估重叠区域的结构一致性
class SSIMMetric {
public:
    // 计算两个图像的SSIM
    // 返回值范围: [-1, 1], 1表示完全相同
    static double compute(const Mat& img1, const Mat& img2,
                         const Mat& overlap_mask);

    // 计算多尺度SSIM
    static double computeMSSIM(const Mat& img1, const Mat& img2,
                               const Mat& overlap_mask);

    // 归一化到0-100分数
    static double normalizeScore(double ssim);
};

// ==================== 3. 梯度一致性 ====================
// Gradient Consistency
// 评估边缘对齐度，对拼接缝质量敏感
class GradientMetric {
public:
    // 计算梯度一致性
    // 使用Sobel算子提取边缘，计算边缘差异
    static double compute(const Mat& img1, const Mat& img2,
                         const Mat& overlap_mask);

    // 基于Canny边缘的匹配度
    static double computeEdgeAlignment(const Mat& img1, const Mat& img2,
                                       const Mat& overlap_mask);
};

// ==================== 4. 拼接缝平滑度 ====================
// Seam Smoothness
// 评估拼接缝处的颜色过渡是否自然
class SeamSmoothnessMetric {
public:
    // 计算拼接缝处的颜色差异
    static double compute(const Mat& stitched_img,
                         const vector<pair<Point, Point>>& seam_lines);

    // 基于相邻区域统计量的差异
    static double computeLocalVariance(const Mat& img,
                                       const Mat& blend_mask);
};

// ==================== 5. 特征点匹配度 ====================
// Feature Matching Score
// 基于ORB/SIFT特征点匹配评估对齐质量
class FeatureMetric {
public:
    struct FeatureMatchResult {
        int num_matches;
        int num_inliers;
        double inlier_ratio;
        double mean_distance;
        double max_distance;
    };

    // 计算特征点匹配质量
    static FeatureMatchResult compute(const Mat& img1, const Mat& img2,
                                      const Mat& overlap_mask);

    // 归一化到0-100分数
    static double normalizeScore(const FeatureMatchResult& result);
};

// ==================== 6. 直线保持度 ====================
// Line Preservation
// 评估去畸变后直线是否保持直线（用于鱼眼相机）
class LineMetric {
public:
    // 检测图像中的直线并计算直线拟合误差
    static double compute(const Mat& img, const Mat& mask);

    // 基于霍夫变换的直线质量评分
    static double computeHoughQuality(const Mat& img, int expected_lines = 10);
};

// ==================== 7. 综合评估器 ====================
class CalibrationEvaluator {
public:
    // 评估相机对（FL-F 或 F-FR）
    static EvaluationReport evaluateCameraPair(
        const Mat& img1, const Mat& img2,
        const Mat& mask1, const Mat& mask2,
        const string& pair_name);

    // 评估完整的前视3相机拼接结果
    static EvaluationReport evaluateFront3Stitching(
        const Mat& stitched_img,
        const Mat& warped_fl, const Mat& warped_f, const Mat& warped_fr,
        const vector<Mat>& blend_masks);

    // 生成评估报告文本
    static string generateReportText(const EvaluationReport& report);

    // 生成可视化图像
    static Mat generateVisualization(const EvaluationReport& report,
                                     const Mat& stitched_img);
};

// ==================== 8. 实时评估显示 ====================
class RealtimeMetricsDisplay {
public:
    // 创建评估指标面板
    static Mat createMetricsPanel(const vector<MetricResult>& metrics,
                                  int width, int height);

    // 更新实时指标
    static void updateMetrics(vector<MetricResult>& metrics,
                              const Mat& current_stitched,
                              const vector<Mat>& camera_images,
                              const vector<Mat>& blend_masks);
};

// ==================== 辅助函数 ====================
namespace Utils {
    // 计算两个图像的有效重叠区域
    Mat computeOverlapMask(const Mat& mask1, const Mat& mask2);

    // 计算图像掩码（非黑色区域）
    Mat computeValidMask(const Mat& img);

    // 计算均方误差
    double computeMSE(const Mat& img1, const Mat& img2, const Mat& mask);

    // 计算平均绝对误差
    double computeMAE(const Mat& img1, const Mat& img2, const Mat& mask);

    // 直方图匹配度
    double computeHistogramSimilarity(const Mat& img1, const Mat& img2,
                                      const Mat& mask);

    // 归一化图像到0-1浮点
    Mat normalizeImage(const Mat& img);
}

} // namespace CalibrationMetrics

#endif // CALIBRATION_METRICS_HPP
