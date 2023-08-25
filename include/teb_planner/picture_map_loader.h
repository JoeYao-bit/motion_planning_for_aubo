#include "point.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"//draw a line
#include "opencv2/opencv.hpp"

namespace freeNav {

    template <Dimension N>
    class Grid_Loader {

    public:

        Grid_Loader() {}

        virtual bool isOccupied(const Pointi<N> & pt) const = 0;

        virtual void setOccupied(const Pointi<N> & pt) = 0;

        virtual DimensionLength* getDimensionInfo() {
            return dimen_;
        }

    protected:

        DimensionLength dimen_[N];

    };

    enum GridState {
        OCCUPIED,
        FREE
    };

    typedef std::vector<GridState> GridMap;

    class PictureLoader : public Grid_Loader<2> {

    public:
        /* load grid map from a picture, keep the raw picture and a writeable copy */
        explicit PictureLoader(std::string file, bool (*f1)(const cv::Vec3b& color)) : Grid_Loader() {
            raw_map_ = cv::imread(file);
            if(raw_map_.cols==0 || raw_map_.rows==0) {
                std::cout << "-- error in map file!" << std::endl;
                exit(0);
            }
            dimen_[0] = raw_map_.cols; // x
            dimen_[1] = raw_map_.rows; // y
            grid_map_ = GridMap(dimen_[0]*dimen_[1], FREE);
            for(int i=0; i<dimen_[0]; i++) {
                for (int j = 0; j < dimen_[1]; j++) {
                    if(f1(raw_map_.at<cv::Vec3b>(j, i))) grid_map_[j * dimen_[0] + i] = OCCUPIED;
                }
            }
        }

        bool isOccupied(const Pointi<2> & pt) const override {
            if(pt[0] < 0 || pt[0] >= dimen_[0] || pt[1] < 0 || pt[1] >= dimen_[1]) { return true; }
            auto index = pt[1]*dimen_[0] + pt[0];
            return grid_map_[index] != FREE;
        }

        void setOccupied(const Pointi<2> & pt) override {
            auto index = pt[1]*dimen_[0] + pt[0];
            if(index >= dimen_[0]*dimen_[1]) return;
            grid_map_[ pt[1]*dimen_[0] + pt[0] ] = OCCUPIED;
        }

        cv::Mat getOriginPicture() const {
            return raw_map_;
        }

    private:
        /* the original map, do not edit it */
        cv::Mat raw_map_;

        GridMap grid_map_;

    };

}