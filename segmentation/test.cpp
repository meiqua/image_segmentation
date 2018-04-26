#include "seg.h"
using namespace std;
using namespace cv;

int main(){
    string prefix = "/home/meiqua/image_segmentation/segmentation/test/test4/";
    Mat rgb = cv::imread(prefix+"3.jpg");
//    medianBlur(rgb, rgb, 5);
    while (rgb.rows>1000) {
        pyrDown(rgb, rgb);
    }

//    Rect roi(300,560,100,100);
//    Rect roi(700,600,100,100);
//    Rect roi(700,1000,100,100);
//    rgb = rgb(roi);

    imshow("rgb", rgb);
//    waitKey(0);

    Mat lab;
    cvtColor(rgb, lab, CV_BGR2Lab);
    //ori: L 0--100 a: -127--127 b: -127--127, now all 0-255
    seg_helper::Timer timer;
    seg_helper::min_span_tree::Graph graph(lab);
    timer.out("MST time");

    Segmentation seg(lab, graph.mst_edges);
    auto lvs = seg.process();
    timer.out("seg time");

//    cout << "is it 24? " << lvs.back().size() << endl;

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
            cv::Vec3b randColor;
            randColor[0] = rand()%255;
            randColor[1] = rand()%255;
            randColor[2] = rand()%255;
            for(int idx: part){
                int row = idx/int(rgb.cols);
                int col = idx%int(rgb.cols);
                show.at<cv::Vec3b>(row, col) = aveColor;
//                show.at<cv::Vec3b>(row, col) = randColor;
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
