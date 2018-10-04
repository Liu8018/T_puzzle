#include "cornerpoint.h"
#include "dstpatternfinder.h"

CornerPoint::CornerPoint()
{
    
}

void CornerPoint::setPoints(const cv::Point2f &pt0, const cv::Point2f &pt1, const cv::Point2f &pt2)
{
    m_pt0 = pt0;
    m_pt1 = pt1;
    m_pt2 = pt2;
}

int CornerPoint::x()
{
    return m_pt0.x;
}
int CornerPoint::y()
{
    return m_pt0.y;
}

void CornerPoint::getVec(std::array<cv::Point2f, 2> &vecs)
{
    vecs[0] = cv::Point(m_pt1.x-m_pt0.x, m_pt1.y-m_pt0.y);
    vecs[1] = cv::Point(m_pt2.x-m_pt0.x, m_pt2.y-m_pt0.y);
}

double CornerPoint::getAngle()
{
    double x1 = m_pt1.x-m_pt0.x,
           y1 = m_pt1.y-m_pt0.y,
           x2 = m_pt2.x-m_pt0.x,
           y2 = m_pt2.y-m_pt0.y;

    double cos_theta = (x1*x2 + y1*y2) / (std::sqrt(x1*x1 + y1*y1) * std::sqrt(x2*x2 + y2*y2));

    cos_theta = (cos_theta > 1.0) ? 1.0 : ( (cos_theta < -1.0)? -1.0 : cos_theta );

    double angle = std::acos(cos_theta)*180.0/3.1415926;

    int judge = (m_pt0.x-m_pt1.x)*(m_pt2.y-m_pt1.y) - (m_pt0.y-m_pt1.y)*(m_pt2.x-m_pt1.x);

    angle = judge<0? angle:(360.0-angle);

    return angle;
}
