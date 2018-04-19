#include "seg.h"
using namespace std;
using namespace cv;
// for test
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int main(){
    string prefix = "/home/meiqua/image_segmentation/segmentation/test/";
    Mat rgb = cv::imread(prefix+"test.png");
    pyrDown(rgb, rgb);
    pyrDown(rgb, rgb);
    pyrDown(rgb, rgb);
    Mat lab;
    cvtColor(rgb, lab, CV_BGR2Lab);
    //ori: L 0--100 a: -127--127 b: -127--127, now all 0-255
    seg_helper::Timer timer;
    seg_helper::min_span_tree::Graph graph(lab);
    timer.out();

    cout << "end" << endl;
    return 0;
}
