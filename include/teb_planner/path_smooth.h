//
// Created by yaozhuo on 2021/9/21.
//

#ifndef FREENAV_PATH_SMOOTH_H
#define FREENAV_PATH_SMOOTH_H
#include "optimal_planner.h"
#include "point.h"

namespace freeNav::RimJump {

    bool pathSmooth(const std::vector<PoseSE2> &pathd);

    /*
     * decompose the line to some small segments
     * no first element
     * */
    std::vector<PoseSE2> lineDiscretize(const PoseSE2 &p1, const PoseSE2 &p2, double interval);


    std::vector<PoseSE2> pathDiscretize(const std::vector<PoseSE2> &pf, double interval);

}

#endif //FREENAV_PATH_SMOOTH_H
