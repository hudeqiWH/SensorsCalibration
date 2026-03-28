// 简化测试：只生成BEV图像并保存，不进行纹理提取和优化
#include "optimizer.h"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    cout << "==========================================" << endl;
    cout << "BEV投影测试 - 只生成并保存BEV图像" << endl;
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
    
    cout << "✓ 图像读取成功:" << endl;
    cout << "  Front: " << imgf.cols << "x" << imgf.rows << " channels=" << imgf.channels() << endl;
    cout << "  Left:  " << imgl.cols << "x" << imgl.rows << " channels=" << imgl.channels() << endl;
    cout << "  Back:  " << imgb.cols << "x" << imgb.rows << " channels=" << imgb.channels() << endl;
    cout << "  Right: " << imgr.cols << "x" << imgr.rows << " channels=" << imgr.channels() << endl;
    cout << "" << endl;
    
    // DEBUG: Check if images have content
    cout << "DEBUG: Check image content (mean pixel value):" << endl;
    cout << "  Front mean: " << mean(imgf) << endl;
    cout << "  Left mean: " << mean(imgl) << endl;
    cout << "  Back mean: " << mean(imgb) << endl;
    cout << "  Right mean: " << mean(imgr) << endl;
    cout << "" << endl;
    
    // No image rotation - let the projection handle orientation
    cout << "图像旋转: 跳过 (由投影变换处理方向)" << endl;
    cout << "" << endl;
    
    // BEV参数
    int bev_rows = 1000, bev_cols = 1000;
    int camera_model = 1;  // 1=Ocam
    string data_set = "ocam";
    string fixed = "front";
    int coarse_flag = 1;
    
    cout << "初始化优化器..." << endl;
    cout << "  BEV尺寸: " << bev_rows << "x" << bev_cols << endl;
    cout << "  相机模型: Ocam" << endl;
    cout << "  数据类型: " << data_set << endl;
    cout << "  固定相机: " << fixed << endl;
    cout << "" << endl;
    
    // 初始化优化器
    Optimizer opt(&imgf, &imgl, &imgb, &imgr, camera_model, bev_rows, bev_cols,
                  fixed, coarse_flag, data_set);
    cout << "✓ 优化器初始化完成" << endl;
    cout << "" << endl;
    
    // 加载Ocam参数
    cout << "加载Ocam参数..." << endl;
    opt.loadOcamParams("./2_dog/param/park_front.json",
                       "./2_dog/param/park_left.json",
                       "./2_dog/param/park_back.json",
                       "./2_dog/param/park_right.json");
    cout << "✓ Ocam参数加载完成" << endl;
    cout << "" << endl;
    
    // 加载外参
    cout << "加载外参..." << endl;
    opt.loadExtrinsicsFromJson("/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5");
    cout << "✓ 外参加载完成" << endl;
    cout << "" << endl;
    
    // 设置输出目录
    opt.prefix = "./test_bev_output/";
    system("mkdir -p ./test_bev_output/");
    
    // 打印加载后的外参（调试用）
    cout << "加载后的外参:" << endl;
    cout << "extrinsic_front:" << endl << opt.extrinsic_front << endl;
    cout << "extrinsic_left:" << endl << opt.extrinsic_left << endl;
    cout << "extrinsic_behind:" << endl << opt.extrinsic_behind << endl;
    cout << "extrinsic_right:" << endl << opt.extrinsic_right << endl;
    cout << "" << endl;
    
    // 生成BEV图像并保存
    cout << "生成BEV图像..." << endl;
    cout << "  投影前相机..." << endl;
    Mat GF = opt.project_on_ground(imgf, opt.extrinsic_front, opt.intrinsic_front,
                                   opt.distortion_params_front, opt.KG, opt.brows,
                                   opt.bcols, opt.hf, "front");
    GF = opt.tail(GF, "f");
    imwrite("./test_bev_output/bev_front.png", GF);
    cout << "    ✓ 保存: ./test_bev_output/bev_front.png" << endl;
    
    cout << "  投影左相机..." << endl;
    Mat GL = opt.project_on_ground(imgl, opt.extrinsic_left, opt.intrinsic_left,
                                   opt.distortion_params_left, opt.KG, opt.brows,
                                   opt.bcols, opt.hl, "left");
    GL = opt.tail(GL, "l");
    imwrite("./test_bev_output/bev_left.png", GL);
    cout << "    ✓ 保存: ./test_bev_output/bev_left.png" << endl;
    
    cout << "  投影后相机..." << endl;
    Mat GB = opt.project_on_ground(imgb, opt.extrinsic_behind, opt.intrinsic_behind,
                                   opt.distortion_params_behind, opt.KG, opt.brows,
                                   opt.bcols, opt.hb, "behind");
    GB = opt.tail(GB, "b");
    imwrite("./test_bev_output/bev_back.png", GB);
    cout << "    ✓ 保存: ./test_bev_output/bev_back.png" << endl;
    
    cout << "  投影右相机..." << endl;
    Mat GR = opt.project_on_ground(imgr, opt.extrinsic_right, opt.intrinsic_right,
                                   opt.distortion_params_right, opt.KG, opt.brows,
                                   opt.bcols, opt.hr, "right");
    GR = opt.tail(GR, "r");
    imwrite("./test_bev_output/bev_right.png", GR);
    cout << "    ✓ 保存: ./test_bev_output/bev_right.png" << endl;
    
    cout << "" << endl;
    cout << "生成环绕视图..." << endl;
    Mat bev_all = opt.generate_surround_view(GF, GL, GB, GR);
    imwrite("./test_bev_output/before_all_calib.png", bev_all);
    cout << "  ✓ 保存: ./test_bev_output/before_all_calib.png" << endl;
    cout << "" << endl;
    
    // 检查文件
    cout << "检查生成的文件..." << endl;
    system("ls -lh ./test_bev_output/*.png");
    cout << "" << endl;
    
    cout << "==========================================" << endl;
    cout << "BEV测试完成!" << endl;
    cout << "==========================================" << endl;
    cout << "" << endl;
    cout << "请检查 ./test_bev_output/ 目录中的图像:" << endl;
    cout << "  - bev_front.png / bev_left.png / bev_back.png / bev_right.png" << endl;
    cout << "  - before_all_calib.png (环绕视图)" << endl;
    cout << "" << endl;
    cout << "如果交通线在不同相机间大致对齐，说明BEV投影基本正确。" << endl;
    cout << "如果交通线完全错位，说明需要调整外参或KG参数。" << endl;
    
    return 0;
}
