//
// Created by yaozhuo on 2023/9/25.
//

#ifndef FREENAV_LOCAL_DISTANCE_MAP_H
#define FREENAV_LOCAL_DISTANCE_MAP_H

#include "point.h"

namespace freeNav {

    // assume all path point range from 0 to MAX<int>
    template<Dimension N>
    std::pair<Pointi<N>, Pointi<N> > getBoundingBoxOfPath(const Path<int, N>& path) {
        Pointi<N> min_pt, max_pt;
        min_pt.setAll(MAX<int>), max_pt.setAll(0);
        for(const auto& pt : path) {
            for(int i=0; i<N; i++) {
                if(pt[i] > max_pt[i]) {
                    max_pt[i] = pt[i];
                }
                if (pt[i] < min_pt[i]) {
                    min_pt[i] = pt[i];
                }
            }
        }
        return {min_pt, max_pt};
    }

    template<Dimension N>
    void updateLocalMapDimen(DimensionLength* local_dimen,
                             DimensionLength* global_dimen,
                             const std::pair<Pointi<N>, Pointi<N> >& local_map_bound,
                             const DimensionLength& expaned_width,
                             Pointi<2>& dist_map_offset_ref
    ) {
        for(int i=0; i<N; i++) {
            DimensionLength max_value = std::min(local_map_bound.second[i] + expaned_width, global_dimen[i]);
            DimensionLength min_value = std::max(local_map_bound.first[i] - expaned_width, (DimensionLength)0);
            local_dimen[i] = max_value - min_value;
            dist_map_offset_ref[i] = min_value;
        }
    }

}

#endif //FREENAV_LOCAL_DISTANCE_MAP_H
