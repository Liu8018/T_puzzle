#include "dstpatternfinder.h"

dstPatternFinder::dstPatternFinder(const cv::Mat &binaryImg)
{
    binaryImg.copyTo(m_img);
}

int dstPatternFinder::getCorners(std::vector<cv::Point> &cornerPoints)
{
    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    if(contours.size() != 1)
        return -1;

    //多边形拟合
    std::vector<cv::Point> approx_points;
    cv::approxPolyDP(*contours.begin(), approx_points, 5, 1);

    //test
    /*cv::Mat findContoursTestImg(m_img.size(), CV_8UC3, cv::Scalar(255,255,255));
    cv::drawContours(findContoursTestImg, contours, -1, cv::Scalar(255));
    cv::Mat approxTestImg;
    findContoursTestImg.copyTo(approxTestImg);
    int order = 0;
    for(auto c : approx_points)
    {
        cv::circle(approxTestImg, c, 5, cv::Scalar(25,230,30));
        cv::putText(approxTestImg, std::to_string(order++), c, 1, 1, cv::Scalar(30,30,250), 1);
    }
    cv::namedWindow("approxTestImg",0);
    cv::imshow("approxTestImg",approxTestImg);
    cv::waitKey();*/

    //返回
    cornerPoints.assign(approx_points.begin(),approx_points.end());
}
