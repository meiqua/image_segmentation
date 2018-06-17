#include "seg.h"

void Segmentation::merge(seg_helper::min_span_tree::Edge &edge)
{
    auto& v1 = edge.v1;
    auto& v2 = edge.v2;
    auto& entry1 = V_list[v1.id];
    auto& entry2 = V_list[v2.id];
    auto parent1_ = find(entry1, V_list);
    auto parent2_ = find(entry2, V_list);
    //we will go on even parent1 == parent2

    //weighted union find
    if(V_list[parent1_].count>V_list[parent2_].count){
        int temp= parent1_;
        parent1_ = parent2_;
        parent2_ = temp;
    }

    auto& parent1 = V_list[parent1_];
    auto& parent2 = V_list[parent2_];
    if(parent1.level == parent2.level){

        parent1.parent = parent2.id;
        parent2.count += parent1.count;

        double stored_cost = std::min(parent1.merging_cost, parent2.merging_cost);
        parent2.merging_cost = edge.weight;

        // high color variation, with a small thresh
        if(edge.weight > 2){
            if(level_recorder.size() <= parent2.level){
                level_recorder.push_back(V_list);
            }
            parent2.level += 1;
        }

        if(parent2.level < level_recorder.size() && parent2.level>0){
            for(int i=parent2.level;i<level_recorder.size();i++){
                auto& V = level_recorder[i];
                V[parent1.id] = parent1;
                V[parent2.id] = parent2;
            }
        }

    }else{
        int high_id, low_id;
        if(parent2.level>parent1.level){
            high_id = parent2.id;
            low_id = parent1.id;
        }else{
            high_id = parent1.id;
            low_id = parent2.id;
        }
        // may break weighted union find, but not too slow when testing
        auto& parent1_h = V_list[low_id];
        auto& parent2_h = V_list[high_id];
        for(int i=parent1_h.level+1;i<parent2_h.level;i++){
            auto& V = level_recorder[i];
            V[parent1_h.id].level = i;
        }
        parent1_h.parent = parent2_h.id;
        parent2_h.count += parent1_h.count;
        parent2_h.merging_cost = edge.weight;

        if(parent2_h.level < level_recorder.size() && parent2_h.level>0){
            for(int i=parent2_h.level;i<level_recorder.size();i++){
                auto& V = level_recorder[i];
                V[parent1_h.id] = parent1_h;
                V[parent2_h.id] = parent2_h;
            }
        }
    }
}

int Segmentation::find(Segmentation::entry &u,std::vector<entry>& V_list)
{
    int u_parent = V_list[u.parent].id;
    while(u_parent != V_list[u_parent].parent){
        u_parent = V_list[V_list[u_parent].parent].id;
    }
    return u_parent;
}

std::vector<std::vector<std::vector<int>>> Segmentation::process()
{
    for(auto& edge: edges){
        merge(edge);
    }

    assert(level_recorder.size()>1);

    // post process
    bool post_process = false;
    if(post_process){
        bool no_size_smaller=false;
        while (!no_size_smaller) {
            no_size_smaller = true;
            for(int i=1; i<level_recorder.size(); i++){
                auto& v_l = level_recorder[i];
                int size_thresh = 1<<(i);  // 2^i

                for(auto& edge: edges){
                    auto& v1 = edge.v1;
                    auto& v2 = edge.v2;
                    auto& entry1 = v_l[v1.id];
                    auto& entry2 = v_l[v2.id];
                    auto parent1_ = find(entry1, v_l);
                    auto parent2_ = find(entry2, v_l);
                    if(parent1_ != parent2_){
                        auto& parent1 = v_l[parent1_];
                        auto& parent2 = v_l[parent2_];
                        if(parent1.count<size_thresh){
                            parent1.parent = parent2.id;
                            no_size_smaller=false;
                        }else if(parent2.count<size_thresh){
                            parent2.parent = parent1.id;
                            no_size_smaller=false;
                        }
                    }
                }
            }
        }
    }

    std::vector<std::vector<std::vector<int>>> lvs;
    for(int i=1;i<level_recorder.size();i++){  // jump level 0

        auto& result = level_recorder[i];
        std::vector<std::vector<int>> segs;
        int segs_size = 0;
        std::map<int, int> parent_idx;
        for(auto& en: result){
            if(en.id == en.parent){
                parent_idx[en.id] = segs_size;
                segs_size ++;
            }
        }

        segs.resize(segs_size);
        for(int y=0; y<super_indices.height(); y++) {
            for(int x=0; x<super_indices.width(); x++) {
                int i0 = super_indices(x,y);
                if(i0 == -1) {
                    continue;
                }
                auto query = parent_idx.find(find(result[i0], result));
                int idx = query->second;
                segs[idx].push_back(x+y*super_indices.width());
            }
        }
        lvs.push_back(std::move(segs));
    }
    return lvs;
}
