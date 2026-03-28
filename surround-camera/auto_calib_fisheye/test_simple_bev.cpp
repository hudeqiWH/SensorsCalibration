// 简化测试：只生成BEV图像
#include "optimizer.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() {
  // 读取图像
  Mat imgf = imread("./2_dog/imgs/ParkFront-1774583477301602000.png");
  Mat imgl = imread("./2_dog/imgs/ParkLeft-1774583477034621000.png");
  Mat imgb = imread("./2_dog/imgs/ParkBack-1774583477301602000.png");
  Mat imgr = imread("./2_dog/imgs/ParkRight-1774583477101370000.png");
  
  if (imgf.empty() || imgl.empty() || imgb.empty() || imgr.empty()) {
    cerr << "Error: Could not read images!" << endl;
    return 1;
  }
  
  // 旋转Ocam图像
  rotate(imgf, imgf, ROTATE_180);
  rotate(imgl, imgl, ROTATE_90_COUNTERCLOCKWISE);
  rotate(imgr, imgr, ROTATE_90_CLOCKWISE);
  rotate(imgb, imgb, ROTATE_180);
  
  cout << "Images loaded successfully!" << endl;
  cout << "Front: " << imgf.size() << endl;
  cout << "Left: " << imgl.size() << endl;
  cout << "Back: " << imgb.size() << endl;
  cout << "Right: " << imgr.size() << endl;
  
  return 0;
}
