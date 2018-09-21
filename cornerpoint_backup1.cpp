#include "cornerpoint.h"

CornerPoint::CornerPoint()
{
}

void CornerPoint::setPoints(const cv::Point pt0, const cv::Point pt1, const cv::Point pt2)
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

void CornerPoint::getVec(std::array<cv::Point, 2> &vecs)
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

void cornerPointTransf(const std::vector<cv::Point> &approxPoints, std::vector<CornerPoint> &cornerPoints)
{
    int len = approxPoints.size();
    if(len < 3)
    {
        std::cout<<"cornerPointTransf error!"<<std::endl;
        exit(0);
    }
    cornerPoints.resize(len);

    for(int i=1;i<len-1;i++)
        cornerPoints[i].setPoints(approxPoints[i], approxPoints[i-1], approxPoints[i+1]);

    cornerPoints[0].setPoints(approxPoints[0], approxPoints[len-1], approxPoints[1]);
    cornerPoints[len-1].setPoints(approxPoints[len-1], approxPoints[len-2], approxPoints[0]);
}

float getNorm(cv::Point vec)
{
    return sqrt(vec.x*vec.x + vec.y*vec.y);
}

void fit(cv::Size imgSize,
         std::vector<CornerPoint> dstCornerPoints,
         std::vector<std::vector<CornerPoint>> unitCornerPoints,
         std::vector<bool> isUsed, int unitSize)
{
    //绘制目标图案
    std::vector<cv::Point> dstContour;
    for(unsigned int i=0;i<dstCornerPoints.size();i++)
        dstContour.push_back(cv::Point(dstCornerPoints[i].x(), dstCornerPoints[i].y() ));
    std::vector<std::vector<cv::Point>> dstContours;
    dstContours.push_back(dstContour);
    cv::Mat dstPattern(imgSize,CV_8U,cv::Scalar(0));
    cv::drawContours(dstPattern,dstContours,0,cv::Scalar(255),-1);
    
    //test
//    cv::namedWindow("testImg0",0);
//    cv::imshow("testImg0",dstPattern);
//    cv::waitKey();
    
    //遍历
    for(int i=0;i<dstCornerPoints.size();i++)//目标图案每个角点
    {
        for(int j=0;j<unitSize;j++)//每个单元块
        {
            //若该单元块已使用过，则跳过
//            if(isUsed[j] == true)
//                continue;

            for(int k=0;k<unitCornerPoints[j].size();k++)//单元块每个角点
            {
                //若目标角小于此角，则跳过
//                if(dstCornerPoints[i].getAngle() < unitCornerPoints[j][k].getAngle() - 8)
//                    continue;

                for(int q=0;q<=1;q++)//角点两个放置方向
                {
std::cout<<"q="<<q<<std::endl;
                    //定义容器存储点集
                    std::vector<cv::Point> pts;
                    pts.push_back(cv::Point(dstCornerPoints[i].x(),dstCornerPoints[i].y()));
                    
cv::Mat unitTest(dstPattern.size(),CV_8U,cv::Scalar(0));
std::vector<cv::Point> upts;
for(int tt1=0;tt1<unitCornerPoints[j].size();tt1++)
    upts.push_back(cv::Point(unitCornerPoints[j][tt1].x(),unitCornerPoints[j][tt1].y()));
std::vector<std::vector<cv::Point>> uptss;
uptss.push_back(upts);
cv::drawContours(unitTest,uptss,-1,cv::Scalar(255));
                    
cv::Mat tmpImg;
dstPattern.copyTo(tmpImg);
cv::circle(tmpImg,pts[0],5,127,2);
int order=0;
cv::putText(unitTest,std::to_string(order),cv::Point(unitCornerPoints[j][k].x(),unitCornerPoints[j][k].y()),1,1,127);
cv::putText(tmpImg,std::to_string(order++),pts[0],1,1,127);
cv::imshow("tmpImg",tmpImg);
                    
                    //定位下一个点
                    std::array<cv::Point,2> dstVecs;
                    dstCornerPoints[i].getVec(dstVecs);
                    std::array<cv::Point,2> unitVecs;
                    unitCornerPoints[j][k].getVec(unitVecs);

                    float dstVecNorm = getNorm(dstVecs[1]);
                    float unitVecNorm = getNorm(unitVecs[q]);
                    float ratio = unitVecNorm/dstVecNorm;

                    cv::Point vec,pt;
                    vec.x = dstVecs[1].x * ratio;
                    vec.y = dstVecs[1].y * ratio;
                    pt.x = vec.x + dstCornerPoints[i].x();
                    pt.y = vec.y + dstCornerPoints[i].y();

if(q==1)
{
    int ttmp=k+1;
    if(k==unitCornerPoints[j].size()-1)
        ttmp=0;
    cv::putText(unitTest,std::to_string(order),cv::Point(unitCornerPoints[j][ttmp].x(),unitCornerPoints[j][ttmp].y()),1,1,127);
}
if(q==0)
{
    int ttmp=k-1;
    if(k==0)
        ttmp=unitCornerPoints[j].size()-1;
    cv::putText(unitTest,std::to_string(order),cv::Point(unitCornerPoints[j][ttmp].x(),unitCornerPoints[j][ttmp].y()),1,1,127);
}
cv::imshow("unitTest",unitTest);

cv::circle(tmpImg,pt,5,127,2);
cv::putText(tmpImg,std::to_string(order++),pt,1,1,127);
cv::imshow("tmpImg",tmpImg);
                    pts.push_back(pt);

                    //若在目标图案外，则跳过
//                    cv::Mat ROI1 = dstPattern(cv::Range(pt.y-4,pt.y+4),cv::Range(pt.x-4,pt.x+4));
//                    if(cv::countNonZero(ROI1) == 0)
//                        continue;

                    //继续追踪点
                    for(int k1=k+2;k1<unitCornerPoints[j].size();k1++)
                    {
                        std::array<cv::Point,2> unitVecs2;
                        unitCornerPoints[j][k1].getVec(unitVecs2);
                        double x1 = unitVecs2[1-q].x;
                        double y1 = unitVecs2[1-q].y;
                        
                        double x2 = -vec.x;
                        double y2 = -vec.y;
                        
                        double sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
                        double cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
                        
                        double x3 = unitVecs2[q].x;
                        double y3 = unitVecs2[q].y;
                        
                        cv::Point nextVec;
                        nextVec.y = y3*cos_theta - x3*sin_theta;
                        nextVec.x = y3*sin_theta + x3*cos_theta;
                        
                        vec.x = nextVec.x;
                        vec.y = nextVec.y;
                        
                        pt.x += nextVec.x;
                        pt.y += nextVec.y;
cv::circle(tmpImg,pt,5,127,2);
cv::putText(tmpImg,"1_"+std::to_string(order++),pt,1,1,127);
cv::imshow("tmpImg",tmpImg);
                        pts.push_back(pt);
                    }
                    for(int k2=(k==unitCornerPoints[j].size()-1?1:0);k2<=k-1;k2++)
                    {

                        std::array<cv::Point,2> unitVecs2;
                        unitCornerPoints[j][k2].getVec(unitVecs2);

                        double x1 = unitVecs2[1-q].x;
                        double y1 = unitVecs2[1-q].y;

                        double x2 = -vec.x;
                        double y2 = -vec.y;

                        double sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
                        double cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);

                        double x3 = unitVecs2[q].x;
                        double y3 = unitVecs2[q].y;

                        cv::Point nextVec;
                        nextVec.y = y3*cos_theta - x3*sin_theta;
                        nextVec.x = y3*sin_theta + x3*cos_theta;
                        
                        vec.x = nextVec.x;
                        vec.y = nextVec.y;
                        
                        pt.x += nextVec.x;
                        pt.y += nextVec.y;
cv::circle(tmpImg,pt,5,127,2);
cv::putText(tmpImg,"2_"+std::to_string(order++),pt,1,1,127);
cv::imshow("tmpImg",tmpImg);
                        pts.push_back(pt);
                    }

                    //test
                    cv::Mat testImg1;
                    dstPattern.copyTo(testImg1);
                    std::vector<std::vector<cv::Point>> contours;
                    contours.push_back(pts);
                    cv::drawContours(testImg1,contours,-1,127,-1);
                    cv::namedWindow("testImg1",0);
                    cv::imshow("testImg1",testImg1);
                    if(cv::waitKey() == 27)
                        exit(0);

                }
            }
        }
    }
}
 
