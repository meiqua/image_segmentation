#include "seg.h"

void Segmentation::merge(seg_helper::min_span_tree::Edge &edge)
{
    auto& v1 = edge.v1;
    auto& v2 = edge.v2;
    auto& entry1 = V_list[v1.id];
    auto& entry2 = V_list[v2.id];
    auto parent1_ = find(entry1, V_list);
    auto parent2_ = find(entry2, V_list);

    auto& parent1 = V_list[parent1_];
    auto& parent2 = V_list[parent2_];
    if(parent1.level == parent2.level){

        double stored_cost = 0;
        stored_cost = std::min(parent1.merging_cost, parent2.merging_cost);

        if(!(stored_cost + 0.001 > edge.weight)){  // high color variation
            if(level_recorder.size() <= parent1.level){
                level_recorder.push_back(V_list);
            }
            parent2.level += 1;
        }
        parent1.parent = parent2.id;

        parent2.count = parent1.count + parent2.count;


        parent2.merging_cost = edge.weight;


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
    entry u_parent = V_list[u.parent];
    while(u_parent.id != u_parent.parent){
        u_parent = V_list[u_parent.parent];
    }
    return u_parent.id;
}

std::vector<std::vector<std::vector<int>>> Segmentation::process()
{
    for(auto& edge: edges){
        merge(edge);
    }

    // post process
    bool no_size_smaller=false;
    while (!no_size_smaller) {
        no_size_smaller = true;
        for(int i=1; i<level_recorder.size(); i++){
            auto& v_l = level_recorder[i];
            int size_thresh = 1<<i;  // 2^i

            // maybe hard code is better for small obj?
//            if(size_thresh>10){
//                size_thresh = 10;
//            }

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
        for(auto& en: result){
            auto query = parent_idx.find(find(en, result));
            int idx = query->second;
            segs[idx].push_back(en.id);
        }
        lvs.push_back(std::move(segs));
    }
    return lvs;
}
