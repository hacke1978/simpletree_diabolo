#ifndef FPFHTOCV23_H
#define FPFHTOCV23_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "opencv2/opencv.hpp"

class PCLtoOpenCV
{


public:
    PCLtoOpenCV();

    cv::Mat mat(PointCloudS::Ptr fpfhs);
};

#endif // FPFHTOCV23_H
