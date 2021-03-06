#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "t_puzzlesolver.h"

int main()
{
    //读入目标图案
    cv::Mat src = cv::imread("../T_puzzle/dstPatterns/3.jpg",0);
    cv::namedWindow("src",0);
    cv::resizeWindow("src",600,600);
    cv::imshow("src",src);

    //读入单元块图像
    cv::Mat units = cv::imread("../T_puzzle/unitPatterns/units.jpg",0);
    cv::namedWindow("units",0);
    cv::resizeWindow("units",600,600);
    cv::imshow("units",units);

    //求解(注意此处可以用多线程对不同旋转角度的图案进行处理)
    T_puzzleSolver solver;
    solver.setImgs(src,units);
    
    solver.setDistortionPara(0.999);//设定允许变形的程度

    bool solved = solver.solve();
    std::cout<<"solved: "<<solved<<std::endl;

    if(solved)
    {
        //得到运行时间
        double runtime = solver.runtime();
        std::cout<<"runtime: "<<runtime<<std::endl;
        
        //得到单元块原来的位置信息
        std::vector<std::vector<cv::Point>> unitsPos;
        solver.getUnitsPos(unitsPos);
        
        //得到返回的单元块位置信息,以及是否需要翻转
        std::vector<bool> isReversed;
        std::vector<std::vector<cv::Point>> resultUnitsPos;
        solver.getResultPos(isReversed,resultUnitsPos);
        
        //计算每个单元块的平移矢量和旋转角度,注意只有当不需要翻转时结果才有意义
        std::vector<cv::Vec2i> translationVectors;
        std::vector<float> angles;
        solver.getTranslationAndRotation(translationVectors,angles);
        
        //以红色显示原来的位置，蓝色显示新的位置,绿色显示平移矢量,紫色显示顺时针旋转角度
        cv::Mat resultTestImg(3000,3000,CV_8UC3,cv::Scalar(0,0,0));
        cv::drawContours(resultTestImg,unitsPos,-1,cv::Scalar(0,0,255),3);
        cv::drawContours(resultTestImg,resultUnitsPos,-1,cv::Scalar(255,0,0), 3);
        
        for(int i=0;i<unitsPos.size();i++)
        {
            cv::Point cPt1 = solver.getCenter(unitsPos[i]);
            cv::Point cPt2 = cv::Point(cPt1.x+translationVectors[i][0],cPt1.y+translationVectors[i][1]);
            
            cv::line(resultTestImg,cPt1,cPt2,cv::Scalar(0,255,0),3);
            cv::circle(resultTestImg,cPt2,10,cv::Scalar(0,255,0),3);
            float angle = solver.rad2angle(angles[i]);
            cv::putText(resultTestImg,std::to_string(angle).substr(0,5),cPt2,1,6,cv::Scalar(255,0,255),6);
            cv::putText(resultTestImg,std::to_string(isReversed[i]),cPt1,1,6,cv::Scalar(0,250,250),6);
        }
        
        cv::namedWindow("resultTestImg",0);
        cv::resizeWindow("resultTestImg",600,600);
        cv::imshow("resultTestImg",resultTestImg);
        if(cv::waitKey() == 's')
            cv::imwrite("result.jpg",resultTestImg);
    }

    return 0;
}

