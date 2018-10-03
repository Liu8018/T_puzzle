#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "t_puzzlesolver.h"

int main()
{
    //读入目标图案
    cv::Mat src = cv::imread("../T_puzzle/dstPatterns/10.jpg",0);
    cv::imshow("src",src);

    //读入单元块图像
    cv::Mat units = cv::imread("../T_puzzle/unitPatterns/units.jpg",0);

    //求解(注意此处可以用多线程对不同旋转角度的图案进行处理)
    T_puzzleSolver solver;
    solver.setImgs(src,units);

    bool solved = solver.solve();
    std::cout<<"solved: "<<solved<<std::endl;

    std::vector<bool> isReversed;
    std::vector<std::vector<cv::Point>> resultUnitPos;

    solver.getResultPos(isReversed,resultUnitPos);

    if(solved)
    {
        double runtime = solver.runtime();
        std::cout<<"runtime: "<<runtime<<std::endl;
        
        cv::Mat resultTestImg(3000,3000,CV_8UC3,cv::Scalar(0,0,0));
        //src.copyTo(resultTestImg);
        //cv::cvtColor(resultTestImg,resultTestImg,cv::COLOR_GRAY2BGR);
        cv::drawContours(resultTestImg,resultUnitPos,-1,cv::Scalar(255,0,0));
        
        cv::imshow("resultTestImg",resultTestImg);
        if(cv::waitKey() == 's')
            cv::imwrite("result.jpg",resultTestImg);
    }

    return 0;
}

