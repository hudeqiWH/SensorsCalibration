// Test shadow sizes
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    int bev_rows = 2000;
    int bev_cols = 2000;
    
    // Current settings
    double blend_start_ratio = 0.48;
    double blend_end_ratio = 0.52;
    int tail_size = 100;
    
    int blend_start = bev_rows * blend_start_ratio;
    int blend_end = bev_rows * blend_end_ratio;
    int center_shadow = blend_end - blend_start;
    
    cout << "=== Current Settings ===" << endl;
    cout << "BEV size: " << bev_rows << "x" << bev_cols << endl;
    cout << "Tail size: " << tail_size << "px" << endl;
    cout << "Blend start: " << blend_start_ratio * 100 << "% (" << blend_start << "px)" << endl;
    cout << "Blend end: " << blend_end_ratio * 100 << "% (" << blend_end << "px)" << endl;
    cout << "Center shadow: " << center_shadow << "x" << center_shadow << " px" << endl;
    cout << "Center shadow ratio: " << (center_shadow * 100.0 / bev_rows) << "%" << endl;
    
    // Calculate effective area
    int effective_size = bev_rows - center_shadow;
    cout << "\nEffective area: ~" << effective_size << "x" << effective_size << " px" << endl;
    
    // Proposed minimal settings
    cout << "\n=== Proposed Minimal Settings ===" << endl;
    tail_size = 30;
    blend_start_ratio = 0.49;
    blend_end_ratio = 0.51;
    
    blend_start = bev_rows * blend_start_ratio;
    blend_end = bev_rows * blend_end_ratio;
    center_shadow = blend_end - blend_start;
    
    cout << "Tail size: " << tail_size << "px" << endl;
    cout << "Blend: " << blend_start_ratio * 100 << "% - " << blend_end_ratio * 100 << "%" << endl;
    cout << "Center shadow: " << center_shadow << "x" << center_shadow << " px" << endl;
    cout << "Center shadow ratio: " << (center_shadow * 100.0 / bev_rows) << "%" << endl;
    effective_size = bev_rows - center_shadow;
    cout << "Effective area: ~" << effective_size << "x" << effective_size << " px" << endl;
    
    // Test image generation
    cout << "\n=== Generating test image ===" << endl;
    Mat test_img(bev_rows, bev_cols, CV_8UC3, Scalar(128, 128, 128));
    
    // Draw center shadow
    blend_start = bev_rows * 0.49;
    blend_end = bev_rows * 0.51;
    rectangle(test_img, Point(blend_start, blend_start), Point(blend_end, blend_end), Scalar(0, 0, 0), -1);
    
    // Draw tail shadows
    rectangle(test_img, Point(0, 0), Point(bev_cols, 30), Scalar(0, 0, 255), -1);  // Top
    rectangle(test_img, Point(0, 0), Point(30, bev_rows), Scalar(0, 0, 255), -1);  // Left
    rectangle(test_img, Point(0, bev_rows-30), Point(bev_cols, bev_rows), Scalar(0, 0, 255), -1);  // Bottom
    rectangle(test_img, Point(bev_cols-30, 0), Point(bev_cols, bev_rows), Scalar(0, 0, 255), -1);  // Right
    
    imwrite("test_shadow_layout.png", test_img);
    cout << "Test image saved: test_shadow_layout.png" << endl;
    cout << "  (Black = center shadow, Red = tail shadows)" << endl;
    
    return 0;
}
