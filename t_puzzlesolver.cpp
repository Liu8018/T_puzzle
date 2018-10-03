#include "t_puzzlesolver.h"

T_puzzleSolver::T_puzzleSolver()
{
    m_solved = false;
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
void preprocess(const cv::Mat &src, cv::Mat &result)
{
    int resizeLength = 200;
    cv::resize(src,result,cv::Size(resizeLength,resizeLength*src.rows/src.cols));

    myThreshold(result,result);
}
void rotate(const cv::Mat &src, cv::Mat &result, int angle)
{
    int len=std::max(src.cols,src.rows);
    cv::Mat rot_mat=getRotationMatrix2D(cv::Point(len/2,len/2),angle,1.0);
    warpAffine(src,result,rot_mat,cv::Size(len,len));
}

void T_puzzleSolver::setImgs(const cv::Mat &dstImg, const cv::Mat &unitsImg)
{
    //读入目标图案并预处理
    preprocess(dstImg,m_dstImg);
    
    //读入单元块图案并预处理
    myThreshold(unitsImg,m_unitsImg);
    
    //将目标图案缩放至与单元块总面积相等，再同时缩小两图案，加快处理速度
    resizeImgs(200);
    
    //提取目标图案并转化为CornerPoint类
    dstPatternFinder dstFinder(m_dstImg);
    dstFinder.getCorners(m_dstPattern);
    cornerPointTransf(m_dstPattern, m_dstCornerPoints);
    
    //提取单元块并转化为CornerPoint类
    cv::findContours(m_unitsImg, m_unitsPos, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    int unitSize = m_unitsPos.size();
    m_unitCornerPoints.clear();
    for (int i=0; i<unitSize; i++)
    {
        cv::Mat unit(m_unitsImg.size(),CV_8U,cv::Scalar(0));
        cv::drawContours(unit, m_unitsPos, i, cv::Scalar(255), -1);

        dstPatternFinder unitFinder(unit);

        unitFinder.getCorners(m_unitsPos[i]);

        std::vector<CornerPoint> tmpUnitCornerPoint;
        cornerPointTransf(m_unitsPos[i], tmpUnitCornerPoint);
        m_unitCornerPoints.push_back(tmpUnitCornerPoint);
    }
}
void T_puzzleSolver::resizeImgs(int size)
{
    //计算面积比例
    float tmpRatio = sqrt(cv::countNonZero(m_unitsImg)/(float)cv::countNonZero(m_dstImg));
    
    //缩放目标图案
    cv::Size newDstSize = cv::Size(m_dstImg.cols*tmpRatio,m_dstImg.rows*tmpRatio);
    cv::resize(m_dstImg, m_dstImg, newDstSize);
    
    //缩放两图案
    int maxSide = m_dstImg.cols > m_dstImg.rows ? m_dstImg.cols:m_dstImg.rows;
    if(maxSide < size)
    {
        m_ratio = 1;
        return;
    }
    m_ratio = size/(float)maxSide;
    
    newDstSize = cv::Size(m_dstImg.cols*m_ratio,m_dstImg.rows*m_ratio);
    cv::resize(m_dstImg, m_dstImg, newDstSize);
    
    cv::Size newUnitsSize = cv::Size(m_unitsImg.cols*m_ratio,m_unitsImg.rows*m_ratio);
    cv::resize(m_unitsImg,m_unitsImg,newUnitsSize);
}

void T_puzzleSolver::getDstPattern(std::vector<cv::Point> &dstPattern)
{
    for(int i=0;i<m_dstPattern.size();i++)
    {
        m_dstPattern[i].x *= 1/m_ratio;
        m_dstPattern[i].y *= 1/m_ratio;
    }
    
    dstPattern.assign(m_dstPattern.begin(),m_dstPattern.end());
}
void T_puzzleSolver::getUnitsPos(std::vector<std::vector<cv::Point>> &unitsPos)
{
    for(int i=0;i<m_unitsPos.size();i++)
    {
        for(int j=0;j<m_unitsPos[i].size();j++)
        {
            m_unitsPos[i][j].x *= 1/m_ratio;
            m_unitsPos[i][j].y *= 1/m_ratio;
        }
        
    }
    
    unitsPos.assign(m_unitsPos.begin(),m_unitsPos.end());
}

bool T_puzzleSolver::solve()
{
//std::cout<<"m_unitsPos.size():"<<m_unitsPos.size()<<std::endl;
    std::vector<bool> isUsed(m_unitsPos.size(),false);
    m_resultUnitsPos.resize(m_unitsPos.size());
    m_isReversed.resize(m_unitsPos.size());
    
    m_runtime = (double)cv::getTickCount();
    m_solved = fit(m_dstImg.size(), m_dstImg, m_dstCornerPoints, m_unitCornerPoints, isUsed, m_unitsPos.size(), m_isReversed, m_resultUnitsPos);
    m_runtime = ((double)cv::getTickCount() - m_runtime) / cv::getTickFrequency();
    
    return m_solved;
}

void T_puzzleSolver::getResultPos(std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitsPos)
{
    if(!m_solved)
    {
        std::cout<<"Not solved!"<<std::endl;
        return;
    }
    
    for(int i=0;i<m_resultUnitsPos.size();i++)
    {
        for(int j=0;j<m_resultUnitsPos[i].size();j++)
        {
            m_resultUnitsPos[i][j].x *= 1/m_ratio;
            m_resultUnitsPos[i][j].y *= 1/m_ratio;
        }
        
    }
    resultUnitsPos.assign(m_resultUnitsPos.begin(),m_resultUnitsPos.end());
    isReversed.assign(m_isReversed.begin(),m_isReversed.end());
}

double T_puzzleSolver::runtime()
{
    return m_runtime;
}
