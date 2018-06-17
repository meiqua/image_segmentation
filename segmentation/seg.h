#ifndef SEG_H
#define SEG_H

#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "CIEDE2000/CIEDE2000.h"
#include <chrono>

#include <slimage/opencv.hpp>
#include <slimage/algorithm.hpp>
#include <asp/algos.hpp>

namespace seg_helper {
class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    void out(std::string message = ""){
        double t = elapsed();
        std::cout << message << "  elasped time:" << t << "s" << std::endl;
        reset();
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
template<typename T>
std::vector<T> unique(const cv::Mat& input, bool sort = true)
{
    std::vector<T> out;
    for (int y = 0; y < input.rows; ++y)
    {
        auto row_ptr = input.ptr<T>(y);
        for (int x = 0; x < input.cols; ++x)
        {
            T value = row_ptr[x];

            if ( std::find(out.begin(), out.end(), value) == out.end() )
                out.push_back(value);
        }
    }

    if (sort)
        std::sort(out.begin(), out.end());

    return out;
}
namespace min_span_tree {
struct Vertex{
    int id;
};
struct Edge {
    double weight;
    Vertex v1;
    Vertex v2;
    bool operator<(const Edge& rhs) const{
      return weight < rhs.weight;
    }
};
struct DisjointSets {
    std::vector<int> parent, rnk;
    DisjointSets(int n){
        parent.resize(n);
        rnk.resize(n, 0);
        for(int i=0; i<parent.size();i++){
            parent[i] = i;
        }
    }
    int find(int u){
        while(u != parent[u]){
            u = parent[u];
        }
        return u;
    }
    void merge(int x, int y){
        x = find(x);
        y = find(y);
        if(rnk[x] > rnk[y]){
            parent[y] = x;
        }else{
            parent[x] = y;
        }
        if(rnk[x]==rnk[y]){
            rnk[y]++;
        }
    }
};
struct Graph {
    int V_size;
    std::vector<Edge> edges, mst_edges;
    double mst_cost;
    slimage::Image<int,1> super_indices;
//    void add_edge(Vertex& v1, Vertex& v2, cv::Mat& lab){
//        CIEDE2000::LAB lab1, lab2;
//        int cols = lab.cols;
//        auto c1 = lab.at<cv::Vec3b>(v1.id/cols,v1.id%cols);
//        auto c2 = lab.at<cv::Vec3b>(v2.id/cols,v2.id%cols);

//        lab1.l = double(c1[0])/255*100;
//        lab1.a = double(c1[1]) - 128;
//        lab1.b = double(c1[2]) - 128;
//        lab2.l = double(c2[0])/255*100;
//        lab2.a = double(c2[1]) - 128;
//        lab2.b = double(c2[2]) - 128;
//        double weight = CIEDE2000::CIEDE2000(lab1, lab2);
//        weight += std::numeric_limits<double>::epsilon();

//        Edge edge;
//        edge.v1 = v1;
//        edge.v2 = v2;
//        edge.weight = weight;
//        edges.push_back(edge);
//    }
    double kruskalMST(){
        double cost = 0;

        std::sort(edges.begin(), edges.end());

        DisjointSets ds(V_size);
        int counts = 0;
        for(auto& edge: edges){
            int v_id1 = edge.v1.id;
            int v_id2 = edge.v2.id;
            int v_parent1 = ds.find(v_id1);
            int v_parent2 = ds.find(v_id2);
            if(v_parent1 != v_parent2){
                ds.merge(v_parent1, v_parent2);
                cost += edge.weight;
                mst_edges.push_back(edge);
                counts++;
                if(counts==V_size-1){
                    break;
                }
            }
        }
        return cost;
    }
    Graph(cv::Mat& lab){
        auto lab_slimage = slimage::ConvertToSlimage(lab);
        slimage::Image3ub img_color = slimage::anonymous_cast<unsigned char,3>(lab_slimage);
        asp::SuperpixelsSlic(img_color);

        asp::SlicParameters slic_param;
        slic_param.num_superpixels = 1024;
        slic_param.compactness = 0.15f;
        auto seg = asp::SuperpixelsSlic(img_color, slic_param);
        bool vis_idxs = true;
        if(vis_idxs){
            cv::Mat idxs = slimage::ConvertToOpenCv(seg.indices);
            std::vector<int> unik = seg_helper::unique<int>(idxs, true);
            std::map<int, cv::Vec3b> color_map;
            for(auto idx: unik){
                auto color = cv::Vec3b(rand()%255, rand()%255, rand()%255);
                color_map[idx] = color;
            }

            cv::Mat show = cv::Mat(idxs.size(), CV_8UC3, cv::Scalar(0));
            auto show_iter = show.begin<cv::Vec3b>();
            for(auto idx_iter = idxs.begin<int>(); idx_iter<idxs.end<int>();idx_iter++, show_iter++){
                if(*idx_iter>0){
                    auto color = color_map.find(*idx_iter)->second;
                    *show_iter = color;
                }
            }
            imshow("show", show);
//            cv::waitKey(1);
        }

        auto& indices = seg.indices;
        const auto width = indices.width();
        const auto height = indices.height();

        // original indices may jump, so we need unique_id
        std::vector<int> unique_id;
        for(int y=0; y<height; y++) {
            for(int x=0; x<width; x++) {
                int i0 = indices(x,y);
                if(i0 == -1) {
                    continue;
                }
                if (std::find(unique_id.begin(), unique_id.end(), i0) == unique_id.end())
                    unique_id.push_back(i0);
            }
        }
        std::sort(unique_id.begin(), unique_id.end());
        std::map<int, int> id2idx;
        for(int i=0; i<unique_id.size();i++){
            id2idx[unique_id[i]] = i;
        }

        V_size = unique_id.size();

        slimage::Image<int,1> adj_table{uint32_t(unique_id.size()), uint32_t(unique_id.size())};
        for(int y=0; y<height-1; y++) {
            for(int x=0; x<width-1; x++) {

                int i0 = indices(x,y);
                if(i0 == -1) {
                    continue;
                }
                int i1 = indices(x+1,y);
                int i2 = indices(x,y+1);
                if(i0 != i1 && i1 != -1) {
                    int idx0 = id2idx.find(i0)->second;
                    int idx1 = id2idx.find(i1)->second;
                    adj_table(idx0, idx1) += 1;
                    adj_table(idx1, idx0) += 1;
                }
                if(i0 != i2 && i2 != -1) {
                    int idx0 = id2idx.find(i0)->second;
                    int idx2 = id2idx.find(i2)->second;
                    adj_table(idx0, idx2) += 1;
                    adj_table(idx2, idx0) += 1;
                }
            }
        }

        for(int y=0; y<unique_id.size();y++){
            for(int x=y+1; x<unique_id.size();x++){
                if(adj_table(x,y)>0){
                    Edge edge;
                    Vertex v1, v2;
                    v1.id = x;
                    v2.id = y;
                    edge.v1 = v1;
                    edge.v2 = v2;

                    auto c1 = seg.superpixels[unique_id[x]].data.color;
                    auto c2 = seg.superpixels[unique_id[y]].data.color;

                    double weight = CIEDE2000::CIEDE2000(c1, c2);
                    weight += std::numeric_limits<double>::epsilon();
                    edge.weight = weight;

//                    edge.weight = (c1 - c2).squaredNorm()/255*100;

                    edges.push_back(edge);
                }
            }
        }
        mst_cost = kruskalMST();

        super_indices = indices;
        for(int y=0; y<height; y++) {
            for(int x=0; x<width; x++) {
                int i0 = super_indices(x,y);
                if(i0 == -1) {
                    continue;
                }
                super_indices(x,y) = id2idx.find(i0)->second;
            }
        }
    }
};

}


}

class Segmentation {
public:
    Segmentation(int V_size, slimage::Image<int,1>& super_indices,
                 std::vector<seg_helper::min_span_tree::Edge>& edges){
        for(int i=0; i<V_size; i++){
                entry e1;
                e1.id = i;
                e1.parent = e1.id;
                V_list.push_back(e1);
            }
        this->edges = std::move(edges);
        this->super_indices = std::move(super_indices);
        level_recorder.resize(1); //we don't want level 0, just empty
    }
    struct entry{
        int id;
        int parent;
        int count = 1;
        int level=0;
        double merging_cost=0;
    };
    std::vector<entry> V_list;
    std::vector<seg_helper::min_span_tree::Edge> edges;
    std::vector<std::vector<entry>> level_recorder;
    slimage::Image<int,1> super_indices;

    void merge(seg_helper::min_span_tree::Edge& edge);
    int find(entry& u, std::vector<entry>& V_list);
    std::vector<std::vector<std::vector<int>>> process();
};
#endif
