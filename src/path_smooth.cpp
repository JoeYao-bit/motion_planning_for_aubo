//
// Created by yaozhuo on 2021/9/21.
//

#include "path_smooth.h"

namespace freeNav::RimJump {

    std::vector<PoseSE2> lineDiscretize(const PoseSE2 &p1, const PoseSE2 &p2, double interval) {
        std::vector<PoseSE2> retv;
        if (p1.position() == p2.position()) {
            std::cerr << __PRETTY_FUNCTION__ << " p1 = p2 error" << std::endl;
            return retv;
        }
        Eigen::Vector2d delta = p2.position() - p1.position();
        double length = delta.norm();
        double temp = 1. / length;
        Eigen::Vector2d unit_delta = delta*(1. / length);
        Eigen::Vector2d buff1;
        PoseSE2 buff2;
        double angle = acos(delta.x() / delta.norm()) * (delta.y() > 0 ? 1 : -1);
        for (int i = 1; i < floor(length / interval); i++) {
            buff2.position() = p1.position() + unit_delta*(i * interval);
            buff2.theta() = angle;
            retv.push_back(buff2);
        }
        if (retv.empty()) {
            retv.push_back(PoseSE2(p2.x(), p2.y(), buff2.theta()));
        }
        if (retv.back().x() != p2.x() || retv.back().y() != p2.y()) {
            retv.push_back(PoseSE2(p2.x(), p2.y(), buff2.theta()));
        }
        return retv;
    }

    std::vector<PoseSE2> pathDiscretize(const std::vector<PoseSE2> &pf, double interval) {
        std::vector<PoseSE2> retv;
        PoseSE2 first;
        first = pf.front();
        Eigen::Vector2d delta = pf[1].position() -  pf[0].position();
        double angle = acos(delta.x() / delta.norm()) * (delta.y() > 0 ? 1 : -1);
        first.theta() = angle;
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