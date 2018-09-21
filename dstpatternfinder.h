#ifndef DSTPATTERNFINDER_H
#define DSTPATTERNFINDER_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

class dstPatternFinder
{
public:
    dstPatternFinder(const cv::Mat &binaryImg);

    int getCorners(std::vector<cv::Point> &cornerPoints);

private:
    cv::Mat m_img;
};

#endif // DSTPATTERNFINDER_H
