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
    
    solver.setDistortionPara(0.999);

    bool solved = solver.solve();
    std::cout<<"solved: "<<solved<<std::endl;

    if(solved)
    {
        std::vector<std::vector<cv::Point>> unitsPos;
        solver.getUnitsPos(unitsPos);
        for(int i=0;i<unitsPos.size();i++)
        {
            cv::putText(units,std::to_string(i),unitsPos[i][0],1,9,200,20);
            
            for(int j=0;j<unitsPos[i].size();j++)
                cv::putText(units,std::to_string(j),unitsPos[i][j],1,5,127,10);
        }
        
        std::vector<bool> isReversed;
        std::vector<std::vector<cv::Point>> resultUnitPos;
        solver.getResultPos(isReversed,resultUnitPos);
        
        double runtime = solver.runtime();
        std::cout<<"runtime: "<<runtime<<std::endl;
        
        cv::Mat resultTestImg(3000,3000,CV_8UC3,cv::Scalar(0,0,0));
        cv::drawContours(resultTestImg,resultUnitPos,-1,cv::Scalar(255,0,0), 3);
        
        for(int i=0;i<resultUnitPos.size();i++)
        {
            cv::putText(resultTestImg,std::to_string(i),resultUnitPos[i][0],1,9,200,20);
            
            for(int j=0;j<resultUnitPos[i].size();j++)
                cv::putText(resultTestImg,std::to_string(j),resultUnitPos[i][j],1,3,127,5);
        }
        
        cv::resize(units,units,cv::Size(units.cols/5,units.rows/5));
        cv::imshow("units",units);
        cv::resize(resultTestImg,resultTestImg,cv::Size(1500,1500));
        cv::imshow("resultTestImg",resultTestImg);
        if(cv::waitKey() == 's')
            cv::imwrite("result.jpg",resultTestImg);
    }

    return 0;
}

