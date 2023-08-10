//
// Created by yaozhuo on 2021/9/20.
//

#include <Eigen/Dense>
#include "canvas.h"
#include "iomanip"

namespace freeNav {

    Canvas::Canvas(std::string name, int size_x, int size_y, double resolution, int zoom_ratio) :
            canvas_(size_y * zoom_ratio, size_x * zoom_ratio, CV_8UC3, cv::Scalar::all(255)), resolution_(resolution) {
        center_[0] = size_x * zoom_ratio / 2;
        center_[1] = size_y * zoom_ratio / 2;
        name_ = name;
        zoom_ratio_ = zoom_ratio;
        setColorTable();
        cv::namedWindow(name_, CV_WINDOW_NORMAL);
    }

    void Canvas::setColorTable() {
        /* set gradation color table, from blue -> green -> red */
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(0, 255 * (1. - i), 255 * i));
        }
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(255 * (i), 0, 255 * (1. - i)));
        }
        std::vector<cv::Scalar> reverse_color_table(gradation_color_table_.rbegin(), gradation_color_table_.rend());
        gradation_color_table_.insert(gradation_color_table_.end(), reverse_color_table.begin(),
                                      reverse_color_table.end());
    }

    void Canvas::drawLineInt(int x1, int y1, int x2, int y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::line(canvas_,
                 cv::Point2i(x1 * zoom_ratio_, y1 * zoom_ratio_) + cv::Point(offset, offset),
                 cv::Point2i(x2 * zoom_ratio_, y2 * zoom_ratio_) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLineInt(const Fraction& x1, const Fraction& y1, const Fraction& x2, const Fraction& y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;

        int round_x1 = round(x1.toFloat()*zoom_ratio_), round_y1 = round(y1.toFloat()*zoom_ratio_);
        int round_x2 = round(x2.toFloat()*zoom_ratio_), round_y2 = round(y2.toFloat()*zoom_ratio_);

        cv::line(canvas_,
                 cv::Point2i(round_x1, round_y1) + cv::Point(offset, offset),
                 cv::Point2i(round_x2, round_y2) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLine(double x1, double y1, double x2, double y2, int line_width, const cv::Scalar &color) {
        Pointi<2> pti1 = transformToPixel(x1, y1);
        Pointi<2> pti2 = transformToPixel(x2, y2);
        drawLineInt(pti1[0], pti1[1], pti2[0], pti2[1], false, line_width, color);
    }

    void Canvas::drawPointInt(int x, int y, const cv::Vec3b &color) {
        canvas_.at<cv::Vec3b>(x * zoom_ratio_, y * zoom_ratio_) = color;
    }

    void Canvas::drawPoint(double x, double y, const cv::Vec3b &color) {
        Pointi<2> pti = transformToPixel(x, y);
        drawPointInt(pti[0], pti[1], color);
    }

    void Canvas::drawCircleInt(int x, int y, int radius, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::circle(canvas_, cv::Point(x * zoom_ratio_, y * zoom_ratio_) + cv::Point(offset, offset), radius, color, line_width, cv::LINE_AA);
    }

    void Canvas::drawCircleInt(const Fraction& x, const Fraction& y, int radius, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        int round_x = round(x.toFloat()*zoom_ratio_), round_y = round(y.toFloat()*zoom_ratio_);
        cv::circle(canvas_, cv::Point(round_x, round_y) + cv::Point(offset, offset), radius, color, line_width, cv::LINE_AA);
    }

    void Canvas::drawCircle(double x, double y, double radius, int line_width, const cv::Scalar &color) {
        Pointi<2> pti = transformToPixel(x, y);
        int radius_i = radius * resolution_;
        drawCircleInt(pti[0], pti[1], radius_i, false, line_width, color);
    }

    void Canvas::resetCanvas(const cv::Scalar &color) {
        canvas_ = cv::Mat(canvas_.rows, canvas_.cols, CV_8UC3, cv::Scalar::all(255));
    }

    Pointd<2> Canvas::transformToWorld(const Pointi<2> &pt) {
        Pointd<2> v;
        v[0] = (pt[0] - center_[0]) / (resolution_ * zoom_ratio_);
        v[1] = -(pt[1] - center_[1]) / (resolution_ * zoom_ratio_);
        return v;
    }

    void canvasMouseCallBack(int event, int x, int y, int flags, void *canvas) {
        Canvas *canvas_ptr = reinterpret_cast<Canvas *>(canvas);
        int x_zoomed = x / canvas_ptr->zoom_ratio_;
        int y_zoomed = y / canvas_ptr->zoom_ratio_;
        if (canvas_ptr->mouse_call_back_func_ != nullptr) {
            (*(canvas_ptr->mouse_call_back_func_))(event, x_zoomed, y_zoomed, flags, nullptr);
        } else {
            std::cout << " canvas_ptr->mouse_call_back_func_ = nullptr !" << std::endl;
            exit(0);
        }
    }


    void Canvas::setMouseCallBack(void (*func)(int, int, int, int, void *)) {
        mouse_call_back_func_ = func;
        cv::setMouseCallback(name_, canvasMouseCallBack, this);
    }

    int Canvas::show(int ms) {
        cv::imshow(name_, canvas_);
        return cv::waitKey(ms);
    }

    void Canvas::drawAxis(double x_range, double y_range, double wing_length) {

        drawLine(-x_range, 0., x_range, 0., 1, cv::Scalar::all(0));
        drawLine(0., -y_range, 0., y_range, 1, cv::Scalar::all(0));

        drawArrow(x_range, 0., 0., .5, 1, cv::Scalar::all(0));
        drawArrow(0., y_range, M_PI_2, .5, 1, cv::Scalar::all(0));

        for (double i = ceil(-x_range + 1); i <= floor(x_range); i++) {
            if (i == 0) continue;
            drawLine(i, -wing_length, i, wing_length, 1, cv::Scalar::all(0));
        }

        for (double i = ceil(-y_range + 1); i <= floor(y_range); i++) {
            if (i == 0) continue;
            drawLine(-wing_length, i, wing_length, i, 1, cv::Scalar::all(0));
        }

    }

    void
    Canvas::drawArrow(double x, double y, double theta, double arrow_length, int line_width, const cv::Scalar &color) {
        Pointi<2> pti = transformToPixel(x, y);
        drawArrowInt(pti[0], pti[1], theta, arrow_length, line_width, color);
    }

    void
    Canvas::drawArrowInt(int x, int y, double theta, double arrow_length, int line_width, const cv::Scalar &color) {
        cv::Point p1(x, y);
        int arrow_length_i = arrow_length * resolution_;
        cv::Point2i p2 = p1 + cv::Point2i(arrow_length_i * cos(theta), -arrow_length_i * sin(theta));
        cv::arrowedLine(canvas_, p1 * zoom_ratio_, p2 * zoom_ratio_, color, line_width, cv::LINE_AA, 0, .2);
    }

    void
    Canvas::drawArrowInt(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::arrowedLine(canvas_, p1 * zoom_ratio_ + cv::Point(offset, offset),
                        p2 * zoom_ratio_ + cv::Point(offset, offset), color, line_width, cv::LINE_AA, 0, .1);
    }

    void Canvas::drawPathf(const Pointds<2> &pathd, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawLine(pathd[i][0], pathd[i][1], pathd[i + 1][0], pathd[i + 1][1], line_width, color);
        }
    }

    void Canvas::drawPointfs(const Pointds<2> &pathd, double radius, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawCircle(pathd[i][0], pathd[i][1], radius, line_width, color);
        }
        drawCircle(pathd.back()[0], pathd.back()[1], radius, line_width, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, double radius, int line_width, const cv::Scalar &color) {
        if (path.empty()) return;
        for (int i = 0; i < path.size() - 1; i++) {
            drawCircle(path[i].x(), path[i].y(), radius, line_width, color);
        }
        drawCircle(path.back().x(), path.back().y(), radius, line_width, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, const std::vector<double>& time_diffs, double current_time,
                             const Point2dContainer &polygon, int line_width) {
        if (path.empty()) return;
        int color_count = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            drawPolygon(path[i].x(), path[i].y(), path[i].theta(), polygon, line_width,
                        gradation_color_table_[color_count]);
            color_count++;
            color_count = color_count % gradation_color_table_.size();
        }
        drawPolygon(path.back().x(), path.back().y(), path.back().theta(), polygon, line_width,
                    gradation_color_table_[color_count]);
    }

    void Canvas::drawPolygon(double x, double y, double theta, const Point2dContainer &polygon, int line_width,
                             const cv::Scalar &color) {
        PoseSE2 pose(x, y, theta);
        const auto &rotate_matrix = Eigen::Rotation2Dd(pose.theta()).toRotationMatrix();
        Point2dContainer global_polygon(polygon.size());
        for (int i = 0; i < global_polygon.size(); i++) {
            PoseSE2 offset(polygon[i], 0);
            offset.rotateGlobal(theta);
            global_polygon[i] = pose.position() + offset.position();
        }
        for (int i = 0; i < global_polygon.size() - 1; i++) {
            drawLine(global_polygon[i].x(), global_polygon[i].y(),
                     global_polygon[i + 1].x(), global_polygon[i + 1].y(),
                     line_width, color);
        }
        drawLine(global_polygon.front().x(), global_polygon.front().y(),
                 global_polygon.back().x(), global_polygon.back().y(),
                 line_width, color);
    }

    void Canvas::drawGrid(int x, int y, const cv::Vec3b &color) {
        if (x < 0 || x >= canvas_.cols / zoom_ratio_ || y < 0 || y >= canvas_.rows / zoom_ratio_) return;
        for (int i = x * zoom_ratio_; i < (x + 1) * zoom_ratio_; i++) {
            for (int j = y * zoom_ratio_; j < (y + 1) * zoom_ratio_; j++) {
                canvas_.at<cv::Vec3b>(j, i) = color;
            }
        }
    }

    void
    Canvas::drawGridLine(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        double offset = center_offset ? .5 : 0;
        cv::line(canvas_,
                 cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_),
                 cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_),
                 color, line_width, cv::LINE_AA);
        cv::circle(canvas_, cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_), 2, color, -1);
        cv::circle(canvas_, cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_), 2, color, -1);

    }


    void Canvas::drawGridMap(freeNav::DimensionLength *dimension,
                             IS_OCCUPIED_FUNC<2> is_occupied) {
        if (dimension[0] > canvas_.cols * zoom_ratio_ || dimension[1] > canvas_.rows * zoom_ratio_) { return; }
        for (int i = 0; i < dimension[0]; i++) {
            for (int j = 0; j < dimension[1]; j++) {
                freeNav::Pointi<2> pt;
                pt[0] = i;
                pt[1] = j;
                if (is_occupied(pt)) {
                    //std::cout << pt << " is occ " << std::endl;
                    drawGrid(i, j);
                }
                //else { std::cout << pt << " is free " << std::endl; }
            }
        }
    }

    void Canvas::drawPointiCircles(const freeNav::Pointis<2> &pts, const cv::Vec3b &color, int radius, int line_width) {
        for (const auto &pt : pts) {
            //drawGrid(pt.second->pt_[0], pt.second->pt_[1], color);
            drawCircleInt(pt[0], pt[1], radius, false, line_width, cv::Scalar(color[0], color[1], color[2]));
            //drawTextInt(pt.second->pt_[0], pt.second->pt_[1], std::to_string(pt.second->surface_id_).c_str(), cv::Scalar::all(0));
        }
    }

    void Canvas::drawGrids(const freeNav::Pointis<2> &pts, const cv::Vec3b &color) {
        for (const auto &pt :pts) {
            drawGrid(pt[0], pt[1], color);
        }
    }

    void Canvas::drawPaths(const freeNav::Pathis<2> &paths) {
        int color_count = 0;
        for (const auto &path : paths) {
            drawPath(path, true, COLOR_TABLE[color_count % 30]);
            color_count++;
        }
    }

    void Canvas::drawPath(const freeNav::Path<int, 2> &path, bool center_offset, const cv::Scalar &color) {
        if (path.size() <= 1) return;
        for (int i = 0; i < path.size() - 1; i++) {
            //drawGridLine(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1], 1, center_offset, color);
            drawArrowInt(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1], 1, center_offset, color);
        }
        auto arrow_tail = path[path.size() - 2];
        auto arrow_tip = path[path.size() - 1];
        //drawArrowInt(arrow_tail[0], arrow_tail[1], arrow_tip[0], arrow_tip[1], 2, center_offset, color);
    }

    void Canvas::draw_DistMap(freeNav::DimensionLength *dimension, const std::vector<PathLen>& dist_map, int max_dist) {

        for(int id=0; id<dist_map.size(); id++) {
            Pointi<2> pt = IdToPointi<2>(id, dimension);
            if(dist_map[id] == MAX<PathLen>) continue;
            //drawTextInt(pt[0], pt[1], std::to_string((int)dist_map[id]).c_str(), cv::Vec3b(0,255,0));
            if(max_dist != 100) {
                if(fabs(dist_map[id]) < 0.2*M_PI) { continue; }
                drawGrid(pt[0], pt[1], cv::Vec3b(255,0,255));
                //drawGrid(pt[0], pt[1], cv::Vec3b(255,255*dist_map[id]/max_dist,255));
            } else {
                drawGrid(pt[0], pt[1], cv::Vec3b(255,255*dist_map[id]/100 + 50,255));
            }
        }

    }

    void Canvas::draw_Block(const freeNav::Pointi<2>& min_pt, const freeNav::Pointi<2>& max_pt) {
        drawLineInt(min_pt[0], min_pt[1], min_pt[0], max_pt[1], false);
        drawLineInt(min_pt[0], min_pt[1], max_pt[0], min_pt[1], false);
        drawLineInt(min_pt[0], max_pt[1], max_pt[0], max_pt[1], false);
        drawLineInt(max_pt[0], min_pt[1], max_pt[0], max_pt[1], false);
    }

    void Canvas::drawEmptyGrid() {
        if (zoom_ratio_ > 5) {
            for (int i = 0; i < canvas_.cols; i += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(i, 0), cv::Point2i(i, canvas_.rows), COLOR_TABLE[10], 1);
            }
            for (int j = 0; j < canvas_.rows; j += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(0, j), cv::Point2i(canvas_.cols, j), COLOR_TABLE[10], 1);
            }
        }
    }

    void Canvas::drawTextInt(int x, int y, const char *string, const cv::Scalar &color, double scale, bool center_offset) {

        int offset = center_offset ? .5 * zoom_ratio_ : 0;


        cv::putText(canvas_,
                    cv::String(string),
                    cv::Point2i((x) * zoom_ratio_ + offset, (y) * zoom_ratio_ + offset),
                    cv::FONT_HERSHEY_COMPLEX,
                    scale, color, 2);
    }


    void Canvas::draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2],
                                    double scale) {
        freeNav::DimensionLength internal_dimen[2];
        internal_dimen[0] = dimen[0] + 1; // ENL_SVG internal setting
        internal_dimen[1] = dimen[1] + 2; // ENL_SVG internal setting
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, internal_dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

}







