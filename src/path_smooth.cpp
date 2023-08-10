//
// Created by yaozhuo on 2021/9/21.
//

#include "path_smooth.h"

namespace freeNav::RimJump {

    std::vector<PoseSE2> lineDiscretize(const Pointd<2> &p1, const Pointd<2> &p2, double interval) {
        std::vector<PoseSE2> retv;
        if (p1 == p2) {
            std::cerr << __PRETTY_FUNCTION__ << " p1 = p2 error" << std::endl;
            return retv;
        }
        Pointd<2> delta = p2 - p1;
        double length = delta.Norm();
        double temp = 1. / length;
        Pointd<2> unit_delta = delta.multi(1. / length);
        Pointd<2> buff1;
        PoseSE2 buff2;
        double angle = getAngle(delta);
        for (int i = 1; i < floor(length / interval); i++) {
            buff1 = p1 + unit_delta.multi(i * interval);
            buff2.x() = buff1[0];
            buff2.y() = buff1[1];
            buff2.theta() = angle;
            retv.push_back(buff2);
        }
        if (retv.empty()) {
            retv.push_back(PoseSE2(p2[0], p2[1], buff2.theta()));
        }
        if (retv.back().x() != p2[0] || retv.back().y() != p2[1]) {
            retv.push_back(PoseSE2(p2[0], p2[1], buff2.theta()));
        }
        return retv;
    }

    std::vector<PoseSE2> pathDiscretize(const Pointds<2> &pf, double interval) {
        std::vector<PoseSE2> retv;
        PoseSE2 first;
        first.x() = pf.front()[0];
        first.y() = pf.front()[1];
        first.theta() = getAngle(pf[1] - pf[0]);
        retv.push_back(first);
        for (int i = 0; i < pf.size() - 1; i++) {
            std::vector<PoseSE2> discrete_line = lineDiscretize(pf[i], pf[i + 1], interval);
            retv.insert(retv.end(), discrete_line.begin(), discrete_line.end());
        }
        return retv;
    }


    bool pathSmooth(const std::vector<PoseSE2> &pathd) {
        //
        return true;
    }

}