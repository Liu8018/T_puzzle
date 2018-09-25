#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "dstpatternfinder.h"
#include "cornerpoint.h"

void preprocess(const cv::Mat &src, cv::Mat &result);
void myThreshold(const cv::Mat &src, cv::Mat &result);
void rotate(const cv::Mat &src, cv::Mat &result, int angle);
bool solve_Tpuzzle(const cv::Mat &srcPic, const cv::Mat &unitsPic, std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitPos);

int main()
{
    //time
    double runTime = (double)cv::getTickCount();
    
    //读入目标图案, 并预处理
    cv::Mat src = cv::imread("../T_puzzle/dstPatterns/11.jpg",0);

    cv::Mat src_pre;
    preprocess(src,src_pre);
    
    //cv::imshow("src_pre",src_pre);
    //cv::waitKey();

    //读入单元块图像
    cv::Mat units = cv::imread("../T_puzzle/unitPatterns/units.jpg",0);
    myThreshold(units,units);
    
    //求解(注意此处可以用多线程对不同旋转角度的图案进行处理)
    std::vector<bool> isReversed;
    std::vector<std::vector<cv::Point>> resultUnitPos;
    bool solved = solve_Tpuzzle(src_pre,units,isReversed,resultUnitPos);

    std::cout<<"solved:"<<solved<<std::endl;
    
    //time
    runTime = ((double)cv::getTickCount() - runTime) / cv::getTickFrequency();
    std::cout << runTime << std::endl;
    
    if(solved)
    {
        cv::Mat resultTestImg;
        src_pre.copyTo(resultTestImg);
        cv::cvtColor(resultTestImg,resultTestImg,cv::COLOR_GRAY2BGR);
        cv::drawContours(resultTestImg,resultUnitPos,-1,cv::Scalar(255,0,0));
        
        cv::imshow("resultTestImg",resultTestImg);
        if(cv::waitKey() == 's')
            cv::imwrite("result.jpg",resultTestImg);
    }

    return 0;
}

void preprocess(const cv::Mat &src, cv::Mat &result)
{
    int resizeLength = 200;
    cv::resize(src,result,cv::Size(resizeLength,resizeLength*src.rows/src.cols));

    myThreshold(result,result);
}

void myThreshold(const cv::Mat &src, cv::Mat &result)
{
    cv::threshold(src, result, 0, 255, CV_THRESH_OTSU);
    
    if((int)result.at<uchar>(0,0)
       +(int)result.at<uchar>(0,result.cols-1)
       +(int)result.at<uchar>(result.rows-1,0)
       +(int)result.at<uchar>(result.rows-1,result.cols-1)
       > 1000)
        cv::threshold(result,result,127,255,CV_THRESH_BINARY_INV);
}

void rotate(const cv::Mat &src, cv::Mat &result, int angle)
{
    int len=std::max(src.cols,src.rows);
    cv::Mat rot_mat=getRotationMatrix2D(cv::Point(len/2,len/2),angle,1.0);
    warpAffine(src,result,rot_mat,cv::Size(len,len));
}

bool solve_Tpuzzle(const cv::Mat &src, const cv::Mat &units, std::vector<bool> &isReversed, std::vector<std::vector<cv::Point> > &resultUnitPos)
{
    //提取目标图案
    dstPatternFinder dstFinder(src);

    std::vector<cv::Point> dstApproxPoints;
    dstFinder.getCorners(dstApproxPoints);

    //角点转化为CornerPoint类
    std::vector<CornerPoint> dstCornerPoints;
    cornerPointTransf(dstApproxPoints, dstCornerPoints);
    
    //计算面积比例
    float ratio = sqrt(cv::countNonZero(src)/(float)cv::countNonZero(units));
    
    //提取单元块
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(units, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    int unitSize = contours.size();
    std::vector<std::vector<CornerPoint>> unitCornerPoints;
    for (int i=0; i<unitSize; i++)
    {
        cv::Mat unit(units.size(),CV_8U,cv::Scalar(0));
        cv::drawContours(unit, contours, i, cv::Scalar(255), -1);

        cv::resize(unit,unit,cv::Size(unit.cols*ratio,unit.rows*ratio));
        cv::threshold(unit, unit, 0, 255, CV_THRESH_OTSU);

        dstPatternFinder unitFinder(unit);

        std::vector<cv::Point> unitApproxPoints;
        unitFinder.getCorners(unitApproxPoints);

        std::vector<CornerPoint> unitCornerPoint;
        cornerPointTransf(unitApproxPoints, unitCornerPoint);
        unitCornerPoints.push_back(unitCornerPoint);
    }
    
    //匹配角点
    std::vector<bool> tmp_isReversed(unitSize,false);
    isReversed.assign(tmp_isReversed.begin(),tmp_isReversed.end());
    std::vector<bool> isUsed(unitSize,false);
    resultUnitPos.resize(unitSize);
    bool fitted = fit(src.size(), src, dstCornerPoints, unitCornerPoints, isUsed, unitSize, isReversed, resultUnitPos);
    
    return fitted;
}



