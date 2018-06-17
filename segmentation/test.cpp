#include "seg.h"
#include "edcircle/edcircle.h"
using namespace std;
using namespace cv;

int main(){
    string prefix = "/home/meiqua/image_segmentation/segmentation/test/t7/";
    Mat rgb = cv::imread(prefix+"3.bmp");
    bool ori = false;
//    auto rgb = cv::imread("/home/meiqua/weitu_cmake/test.png");
//    while (rgb.rows>=1000) {
//        pyrDown(rgb, rgb);
//    }
    cv::resize(rgb, rgb, {1024*rgb.cols/rgb.rows, 1024});
    std::cout << "rgb.rows: " <<rgb.rows << std::endl;

//    medianBlur(rgb, rgb, 5);

    imshow("rgb", rgb);
    waitKey(0);

    Mat lab;
    cvtColor(rgb, lab, CV_BGR2Lab);
    //ori: L 0--100 a: -127--127 b: -127--127, now all 0-255
    seg_helper::Timer timer;
    seg_helper::min_span_tree::Graph graph(lab);
    timer.out("MST time");

    Segmentation seg(graph.V_size, graph.super_indices, graph.mst_edges);
    auto lvs = seg.process();
    timer.out("seg time");

    cv::Mat src;
    int i = 0;
//    for(int i=0;i<lvs.size();i++)
    {
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
//        imshow("level"+to_string(i+1), show);
//        imwrite(prefix + "test_rgb_ave/level"+to_string(i+1)+".png", show);
        if(ori){
            cv::cvtColor(rgb, src, CV_BGR2GRAY);
        }else{
            cv::cvtColor(show, src, CV_BGR2GRAY);
            medianBlur(src, src, 7);
            imshow("level"+to_string(i+1), src);
        }

    }

    std::vector<float> radiuses;
    auto centers = edcircle::find_circle(src, radiuses);

    if(centers.empty()){
        std::cout << "nothing detect" << std::endl;
        return 0;
    }

    cv::Point c = {src.cols/2, src.rows/2};
    cv::Point best_p;
    int best_r;
    double closest = std::numeric_limits<double>::max();
    for(int i=0; i<centers.size(); i++){
        auto& p = centers[i];
        auto p2c = p-c;
        double dist = p2c.x*p2c.x + p2c.y*p2c.y;
        if(dist<closest){
            closest = dist;
            best_p = p;
            best_r = int(radiuses[i]);
        }
    }

    bool vis_result = true;
    if(vis_result){
        cv::Mat to_show;
        cv::cvtColor(src, to_show, CV_GRAY2BGR);
        for(int i=0; i<centers.size(); i++){
            cv::circle(to_show, centers[i], radiuses[i], {0, 0, 255}, 2);
        }
        cv::line(to_show, cv::Point(best_p.x, best_p.y-best_r/2), cv::Point(best_p.x, best_p.y+best_r/2)
                        , cv::Scalar(0, 255, 0), 2);
        cv::line(to_show, cv::Point(best_p.x-best_r/2, best_p.y), cv::Point(best_p.x+best_r/2, best_p.y)
                        , cv::Scalar(0, 255, 0), 2);
        // cv::pyrDown(to_show, to_show);
        cv::imshow("hole detect", to_show);
//        cv::waitKey(3000);
    }


    waitKey(0);
    cout << "end" << endl;
    return 0;
}
