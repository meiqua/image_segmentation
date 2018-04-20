#include "seg.h"
using namespace std;
using namespace cv;

int main(){
    string prefix = "/home/meiqua/image_segmentation/segmentation/test/test2/";
    Mat rgb = cv::imread(prefix+"test.png");
//    pyrDown(rgb, rgb);

    Mat lab;
    cvtColor(rgb, lab, CV_BGR2Lab);
    //ori: L 0--100 a: -127--127 b: -127--127, now all 0-255
    seg_helper::Timer timer;
    seg_helper::min_span_tree::Graph graph(lab);
    timer.out("MST time");

    Segmentation seg(lab, graph.mst_edges);
    auto lvs = seg.process();
    timer.out("seg time");

    for(int i=0;i<lvs.size();i++){
        auto& lv = lvs[i];

        Mat show = Mat(rgb.size(), CV_8UC3, Scalar(0));
        for(auto& part: lv){
            cv::Vec3d aveColor(0,0,0);
            int count = 0;
            for(int idx: part){
                int row = idx/int(rgb.cols);
                int col = idx%int(rgb.cols);
                aveColor += rgb.at<cv::Vec3b>(row, col);
//                aveColor += lab.at<cv::Vec3b>(row, col);
                count ++;
            }
            aveColor /= count;
            for(int idx: part){
                int row = idx/int(rgb.cols);
                int col = idx%int(rgb.cols);
                show.at<cv::Vec3b>(row, col) = aveColor;
            }
        }
//        cvtColor(show, show, CV_Lab2BGR);
        imshow("level"+to_string(i+1), show);
//        imwrite(prefix + "test_rgb_ave/level"+to_string(i+1)+".png", show);
    }
    waitKey(0);
    cout << "end" << endl;
    return 0;
}
