#ifndef T_PUZZLESOLVER_H
#define T_PUZZLESOLVER_H

#include "cornerpoint.h"
#include "dstpatternfinder.h"

class T_puzzleSolver
{
public:
    T_puzzleSolver();
    
    void setImgs(const cv::Mat &dstImg, const cv::Mat &unitsImg);
    
    int getUnitSize();
    
    void setDistortionPara(float areaRatio);
    
    void getDstPattern(std::vector<cv::Point> &dstPattern);
    void getUnitsPos(std::vector<std::vector<cv::Point>> &unitsPos);
    
    cv::Point getCenter(const std::vector<cv::Point> &pts);
    float getRotateAngle(const std::vector<cv::Point> &pts0, const std::vector<cv::Point> &pts1);
    
    bool solve();
    
    void getResultPos(std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitsPos);
    
    double runtime();
    
    void getTranslationAndRotation(std::vector<cv::Vec2i> &translationVectors, std::vector<float> &angles);
    
    float rad2angle(float rad);
    
private:
    cv::Mat m_dstImg;
    cv::Mat m_unitsImg;
    
    int m_unitSize;
    
    float m_areaDistortionRatio;
    
    void resizeImgs(int size);
    float m_dstResizeRatio;
    
    std::vector<cv::Point> m_dstPattern;
    std::vector<std::vector<cv::Point>> m_unitsPos;
    
    std::vector<bool> m_isReversed;
    std::vector<std::vector<cv::Point>> m_resultUnitsPos;
    
    std::vector<CornerPoint> m_dstCornerPoints;
    std::vector<std::vector<CornerPoint>> m_unitCornerPoints;
    
    bool m_solved;
    double m_runtime;
    
    //求解过程中用到的变量和函数
    void cornerPointTransf(const std::vector<cv::Point> &approxPoints, std::vector<CornerPoint> &cornerPoints);
    float getNorm(cv::Point vec);
    void getRVec(cv::Point2f vecA1, cv::Point2f vecA2, cv::Point2f vecB1, cv::Point2f &vecB2);
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
