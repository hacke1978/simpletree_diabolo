#include "fpfhtocv.h"

cv::Mat PCLtoOpenCV::mat(PointCloudS::Ptr fpfhs) const
{
    cv::Mat openCVPointCloud( fpfhs->points.size(), 1,  CV_64UF(3));
//    for(int i=0; i < fpfhs->points.size();i++)
//    {
//        PointS p = fpfhs->points.at(i);
//        openCVPointCloud.at<cv::Vec<float,3> >(i,0)[0] = p.eigen1;
//        openCVPointCloud.at<cv::Vec<float,3> >(i,0)[1] = p.eigen2;
//        openCVPointCloud.at<cv::Vec<float,3> >(i,0)[2] = p.eigen3;
//    }
    return openCVPointCloud;
}

PCLtoOpenCV::PCLtoOpenCV()
{


}
