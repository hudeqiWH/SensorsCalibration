// 测试：生成BEV图像，不使用tail，调整KG参数
#include "optimizer.h"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    cout << "==========================================" << endl;
    cout << "BEV投影测试 - 无tail，可调KG" << endl;
    cout << "==========================================" << endl;
    cout << "" << endl;
    
    // 读取图像
    Mat imgf = imread("./2_dog/imgs/ParkFront-1774583477301602000.png");
    Mat imgl = imread("./2_dog/imgs/ParkLeft-1774583477034621000.png");
    Mat imgb = imread("./2_dog/imgs/ParkBack-1774583477301602000.png");
    Mat imgr = imread("./2_dog/imgs/ParkRight-1774583477101370000.png");
    
    if (imgf.empty() || imgl.empty() || imgb.empty() || imgr.empty()) {
        cerr << "错误: 无法读取一个或多个图像!" << endl;
        return 1;
    }
    
    // 旋转Ocam图像
    rotate(imgf, imgf, ROTATE_180);
    rotate(imgl, imgl, ROTATE_90_COUNTERCLOCKWISE);
    rotate(imgr, imgr, ROTATE_90_CLOCKWISE);
    rotate(imgb, imgb, ROTATE_180);
    
    // BEV参数
    int bev_rows = 1000, bev_cols = 1000;
    int camera_model = 1;  // 1=Ocam
    string data_set = "ocam";
    string fixed = "front";
    int coarse_flag = 1;
    
    // 初始化优化器
    Optimizer opt(&imgf, &imgl, &imgb, &imgr, camera_model, bev_rows, bev_cols,
                  fixed, coarse_flag, data_set);
    
    // 加载Ocam参数
    opt.loadOcamParams("./2_dog/param/park_front.json",
                       "./2_dog/param/park_left.json",
                       "./2_dog/param/park_back.json",
                       "./2_dog/param/park_right.json");
    
    // 加载外参
    opt.loadExtrinsicsFromJson("/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5");
    
    // 调整KG参数 - 尝试不同的地面分辨率
    cout << "调整KG参数..." << endl;
    cout << "原始KG:" << endl << opt.KG << endl;
    
    // 尝试1：更高的分辨率（0.5 cm/像素）
    // opt.KG(0, 0) = 1.0 / 0.005;
    // opt.KG(1, 1) = -1.0 / 0.005;
    
    // 尝试2：更低的分辨率（2 cm/像素）
    opt.KG(0, 0) = 1.0 / 0.02;
    opt.KG(1, 1) = -1.0 / 0.02;
    
    cout << "调整后KG:" << endl << opt.KG << endl;
    cout << "" << endl;
    
    // 设置输出目录
    opt.prefix = "./test_bev_no_tail/";
    system("mkdir -p ./test_bev_no_tail/");
    
    // 生成BEV图像（不使用tail）
    cout << "生成BEV图像（不使用tail）..." << endl;
    
    Mat GF = opt.project_on_ground(imgf, opt.extrinsic_front, opt.intrinsic_front,
                                   opt.distortion_params_front, opt.KG, opt.brows,
                                   opt.bcols, opt.hf, "front");
    // 不使用tail: GF = opt.tail(GF, "f");
    imwrite("./test_bev_no_tail/bev_front.png", GF);
    cout << "  ✓ bev_front.png" << endl;
    
    Mat GL = opt.project_on_ground(imgl, opt.extrinsic_left, opt.intrinsic_left,
                                   opt.distortion_params_left, opt.KG, opt.brows,
                                   opt.bcols, opt.hl, "left");
    // GL = opt.tail(GL, "l");
    imwrite("./test_bev_no_tail/bev_left.png", GL);
    cout << "  ✓ bev_left.png" << endl;
    
    Mat GB = opt.project_on_ground(imgb, opt.extrinsic_behind, opt.intrinsic_behind,
                                   opt.distortion_params_behind, opt.KG, opt.brows,
                                   opt.bcols, opt.hb, "behind");
    // GB = opt.tail(GB, "b");
    imwrite("./test_bev_no_tail/bev_back.png", GB);
    cout << "  ✓ bev_back.png" << endl;
    
    Mat GR = opt.project_on_ground(imgr, opt.extrinsic_right, opt.intrinsic_right,
                                   opt.distortion_params_right, opt.KG, opt.brows,
                                   opt.bcols, opt.hr, "right");
    // GR = opt.tail(GR, "r");
    imwrite("./test_bev_no_tail/bev_right.png", GR);
    cout << "  ✓ bev_right.png" << endl;
    
    Mat bev_all = opt.generate_surround_view(GF, GL, GB, GR);
    imwrite("./test_bev_no_tail/before_all_calib.png", bev_all);
    cout << "  ✓ before_all_calib.png" << endl;
    cout << "" << endl;
    
    // 检查文件
    cout << "检查文件大小:" << endl;
    system("ls -lh ./test_bev_no_tail/*.png");
    cout << "" << endl;
    
    cout << "请检查 ./test_bev_no_tail/before_all_calib.png 中的交通线" << endl;
    cout << "如果看不到，说明KG参数或外参转换仍有问题" << endl;
    
    return 0;
}
