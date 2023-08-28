#include "common.h"

Point2dContainer transformedFrom(const Point2dContainer& foot_print, const PoseSE2& global_pose) {
    Point2dContainer global_polygon(foot_print.size());
    for (int i = 0; i < global_polygon.size(); i++) {
        PoseSE2 offset(foot_print[i], 0);
        offset.rotateGlobal(global_pose.theta());
        global_polygon[i] = global_pose.position() + offset.position();
    }
    return global_polygon;
}