#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>

class CornerPoint
{
public:
    CornerPoint();

    void setPoints(const cv::Point2f &pt0, const  cv::Point2f &pt1, const cv::Point2f &pt2);

    int x();
    int y();

    double getAngle();

    void getVec(std::array<cv::Point2f, 2> &vecs);
private:

    cv::Point2f m_pt0, m_pt1, m_pt2;

};

void cornerPointTransf(const std::vector<cv::Point> &approxPoints, std::vector<CornerPoint> &cornerPoints);

bool fit(const cv::Size &imgSize,
         const cv::Mat &originDst,
         std::vector<CornerPoint> dstCornerPoints,
         std::vector<std::vector<CornerPoint>> unitCornerPoints,
         std::vector<bool> isUsed, 
         int unitSize, 
         std::vector<bool> &isReversed,
         std::vector<std::vector<cv::Point>> &resultUnitPos);

#endif // CORNERPOINT_H
