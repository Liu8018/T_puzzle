#ifndef T_PUZZLESOLVER_H
#define T_PUZZLESOLVER_H

#include "cornerpoint.h"
#include "dstpatternfinder.h"

class T_puzzleSolver
{
public:
    T_puzzleSolver();
    
    void setImgs(const cv::Mat &dstImg, const cv::Mat &unitsImg);
    
    void setDistortionPara(float areaRatio);
    
    void getDstPattern(std::vector<cv::Point> &dstPattern);
    void getUnitsPos(std::vector<std::vector<cv::Point>> &unitsPos);
    
    bool solve();
    
    void getResultPos(std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitsPos);
    
    double runtime();
    
private:
    cv::Mat m_dstImg;
    cv::Mat m_unitsImg;
    
    float m_areaRatio;
    
    void resizeImgs(int size);
    float m_ratio;
    
    std::vector<cv::Point> m_dstPattern;
    std::vector<std::vector<cv::Point>> m_unitsPos;
    
    std::vector<bool> m_isReversed;
    std::vector<std::vector<cv::Point>> m_resultUnitsPos;
    
    std::vector<CornerPoint> m_dstCornerPoints;
    std::vector<std::vector<CornerPoint>> m_unitCornerPoints;
    
    bool m_solved;
    
    double m_runtime;
    
    //求解过程中用到的变量和函数
    //std::vector<std::vector<cv::Point>> m_s_dstCornerPionts;
    //std::vector<std::vector<cv::Point>> m_s_unitCornerPionts;
    //std::vector<bool> m_isUsed;
    
    void cornerPointTransf(const std::vector<cv::Point> &approxPoints, std::vector<CornerPoint> &cornerPoints);
    float getNorm(cv::Point vec);
    cv::Point2f getRVec(cv::Point2f vec1, cv::Point2f vec2, cv::Point2f vec3);
    bool cutPattern(const cv::Mat &pattern, const std::vector<cv::Point2f> &pts, cv::Mat &nextPattern);
    bool fit(const cv::Size &imgSize,
             std::vector<CornerPoint> dstCornerPoints,
             std::vector<std::vector<CornerPoint>> unitCornerPoints,
             std::vector<bool> isUsed, 
             int unitSize,
             std::vector<bool> &isReversed, 
             std::vector<std::vector<cv::Point> > &resultUnitPos);
    
};

#endif // T_PUZZLESOLVER_H
