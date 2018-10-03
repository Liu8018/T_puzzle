#ifndef T_PUZZLESOLVER_H
#define T_PUZZLESOLVER_H

#include "cornerpoint.h"
#include "dstpatternfinder.h"

class T_puzzleSolver
{
public:
    T_puzzleSolver();
    
    void setImgs(const cv::Mat &dstImg, const cv::Mat &unitsImg);
    
    void getDstPattern(std::vector<cv::Point> &dstPattern);
    void getUnitsPos(std::vector<std::vector<cv::Point>> &unitsPos);
    
    bool solve();
    
    void getResultPos(std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitsPos);
    
    double runtime();
    
private:
    cv::Mat m_dstImg;
    cv::Mat m_unitsImg;
    
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
    
};

#endif // T_PUZZLESOLVER_H
