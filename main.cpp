#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "dstpatternfinder.h"
#include "cornerpoint.h"

int main()
{
    //time
    double runTime = (double)cv::getTickCount();
    
    //读入图像, 并预处理
    cv::Mat src = cv::imread("../T_puzzle/dstPatterns/10.jpg",0);

    int resizeLength = 400;
    cv::resize(src,src,cv::Size(resizeLength,resizeLength*src.rows/src.cols));

    cv::threshold(src, src, 0, 255, CV_THRESH_OTSU);
    //if(src.at<uchar>(src.rows/2,src.cols/2) == 0)
    if(cv::countNonZero(src) > src.rows*src.cols/2)
        cv::threshold(src,src,127,255,CV_THRESH_BINARY_INV);

/*    //test
    cv::namedWindow("threshImg",0);
    cv::resizeWindow("threshImg",600,600);
    cv::imshow("threshImg",src);
    cv::waitKey();
*/

    //提取目标图案
    dstPatternFinder dstFinder(src);

    std::vector<cv::Point> dstApproxPoints;
    dstFinder.getCorners(dstApproxPoints);

    //角点转化为CornerPoint类
    std::vector<CornerPoint> dstCornerPoints;
    cornerPointTransf(dstApproxPoints, dstCornerPoints);

/*    //test
    cv::Mat cornerPointsTestImg;
    src.copyTo(cornerPointsTestImg);
    cv::cvtColor(cornerPointsTestImg,cornerPointsTestImg,cv::COLOR_GRAY2BGR);
    for(int i=0;i<cornerPoints.size();i++)
    {
        cv::Point pt(cornerPoints[i].x(),cornerPoints[i].y());
        cv::circle(cornerPointsTestImg,pt,5,cv::Scalar(250,100,30),2);

        cv::putText(cornerPointsTestImg,std::to_string(cornerPoints[i].getAngle()),pt,1, 1, cv::Scalar(80,100,250),2);
    }
    cv::namedWindow("cornerPointsTestImg",0);
    cv::resizeWindow("cornerPointsTestImg",600,600);
    cv::imshow("cornerPointsTestImg",cornerPointsTestImg);
    cv::waitKey();
*/

    //读入单元块并提取单元块的CornerPoint
    cv::Mat units = cv::imread("../T_puzzle/unitPatterns/units.jpg",0);
    float ratio = sqrt(cv::countNonZero(src)/(float)cv::countNonZero(units));
    int unitSize = 4;
    std::vector<std::vector<CornerPoint>> unitCornerPoints;
    for (unsigned int i=1; i<=unitSize; i++)
    {
        std::string file = "../T_puzzle/unitPatterns/" + std::to_string(i) + ".jpg";
        cv::Mat unit = cv::imread(file,0);
        cv::threshold(unit,unit,127,255,CV_THRESH_BINARY);

        cv::resize(unit,unit,cv::Size(unit.cols*ratio,unit.rows*ratio));

        dstPatternFinder unitFinder(unit);

        std::vector<cv::Point> unitApproxPoints;
        unitFinder.getCorners(unitApproxPoints);

        std::vector<CornerPoint> unitCornerPoint;
        cornerPointTransf(unitApproxPoints, unitCornerPoint);
        unitCornerPoints.push_back(unitCornerPoint);
    }

    std::vector<bool> isReversed(unitSize,false);
    std::vector<bool> isUsed(unitSize,false);
    std::vector<std::vector<cv::Point>> resultUnitPos(unitSize);
    fit(src.size(), src, dstCornerPoints, unitCornerPoints, isUsed, unitSize, isReversed, resultUnitPos);
    
    //time
    runTime = ((double)cv::getTickCount() - runTime) / cv::getTickFrequency();
    std::cout << runTime << std::endl;
    
    cv::Mat resultTestImg;
    src.copyTo(resultTestImg);
    cv::cvtColor(resultTestImg,resultTestImg,cv::COLOR_GRAY2BGR);
    cv::drawContours(resultTestImg,resultUnitPos,-1,cv::Scalar(255,0,0));
    
    cv::imshow("resultTestImg",resultTestImg);
    cv::waitKey();

    return 0;
}




