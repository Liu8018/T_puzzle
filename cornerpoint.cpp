#include "cornerpoint.h"
#include "dstpatternfinder.h"

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
        std::cout<<"approxPoints.size()="<<len<<std::endl;
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

cv::Point getRVec(cv::Point vec1, cv::Point vec2, cv::Point vec3)
{
    double x1 = vec1.x;
    double y1 = vec1.y;
    
    double x2 = vec2.x;
    double y2 = vec2.y;
    
    double sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
    double cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
    
    double x3 = vec3.x;
    double y3 = vec3.y;
    
    cv::Point nextVec;
    nextVec.y = y3*cos_theta - x3*sin_theta;
    nextVec.x = y3*sin_theta + x3*cos_theta;
    
    return nextVec;
}

bool cutPattern(const cv::Mat &pattern, const std::vector<cv::Point> &pts, cv::Mat &nextPattern)
{
    std::vector<std::vector<cv::Point>> contour;
    contour.push_back(pts);
    
    cv::Mat tmpImg(pattern.size(),CV_8U,cv::Scalar(1));
    cv::drawContours(tmpImg, contour, 0, 255, -1);
    
    cv::imshow("pattern",pattern);
    cv::imshow("tmpImg",tmpImg);
    
    cv::Mat tmpImg2 = (tmpImg == pattern);
    
    tmpImg = (tmpImg != cv::Mat(tmpImg.size(),CV_8U,cv::Scalar(1)));
    
    cv::imshow("tmpImg2",tmpImg2);
    //cv::waitKey();
    
    float ratio = cv::countNonZero(tmpImg2) / (float)cv::countNonZero(tmpImg);
    
    std::cout<<"ratio="<<ratio<<std::endl;

    if(ratio < 0.95)
        return false;
    
    pattern.copyTo(nextPattern);
    cv::drawContours(nextPattern, contour, 0, 0, -1);
    
    return true;
}

bool fit(const cv::Size &imgSize, 
         const cv::Mat &originDst,
         std::vector<CornerPoint> dstCornerPoints,
         std::vector<std::vector<CornerPoint>> unitCornerPoints,
         std::vector<bool> isUsed, 
         int unitSize,
         std::vector<bool> &isReversed, 
         std::vector<std::vector<cv::Point> > &resultUnitPos)
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
            if(isUsed[j] == true)
                continue;
            for(int rev=0;rev<=1;rev++)//单元块两个放置面
            {
                bool iR = rev==0 ? false : true;
                if(iR != isReversed[j])
                {
                    isReversed[j] = iR;
                    
                    std::vector<cv::Point> tmpPoints;
                    for(int nn=0;nn<unitCornerPoints[j].size();nn++)
                    {
                        tmpPoints.push_back(cv::Point(unitCornerPoints[j][nn].x(),unitCornerPoints[j][nn].y()));
                        
                        tmpPoints[nn].x = dstPattern.cols - tmpPoints[nn].x;
                    }
                    
                    std::vector<CornerPoint> tmpUnitPoints;
                    cornerPointTransf(tmpPoints,tmpUnitPoints);
                    
                    unitCornerPoints[j].assign(tmpUnitPoints.begin(),tmpUnitPoints.end());
                }

                for(int k=0;k<unitCornerPoints[j].size();k++)//单元块每个角点
                {
//std::cout<<"k="<<k<<std::endl;
                    //若目标角小于此角，则跳过
                    if(dstCornerPoints[i].getAngle() < unitCornerPoints[j][k].getAngle() - 8)
                        continue;
    
                    for(int q=0;q<=1;q++)//角点两个放置方向
                    {
//std::cout<<"    q="<<q<<std::endl;
                        for(int r=0;r<=1;r++)//目标角两条侧边
                        {
//std::cout<<"        r="<<r<<std::endl;
                            //定义容器存储点集
                            std::vector<cv::Point> pts;
                            pts.push_back(cv::Point(dstCornerPoints[i].x(),dstCornerPoints[i].y()));

                            //定位下一个点
                            std::array<cv::Point,2> dstVecs;
                            dstCornerPoints[i].getVec(dstVecs);
                            std::array<cv::Point,2> unitVecs;
                            unitCornerPoints[j][k].getVec(unitVecs);

                            float dstVecNorm = getNorm(dstVecs[q]);
                            float unitVecNorm = getNorm(unitVecs[r]);
                            float ratio = unitVecNorm/dstVecNorm;

                            cv::Point vec,pt;
                            vec.x = dstVecs[q].x * ratio;
                            vec.y = dstVecs[q].y * ratio;
                            pt.x = vec.x + dstCornerPoints[i].x();
                            pt.y = vec.y + dstCornerPoints[i].y();

                            pts.push_back(pt);

                            //若在目标图案外，则跳过
                            int radius = 4;
                            if(pt.x <= radius || pt.y <= radius || pt.x >= dstPattern.cols - radius || pt.y >= dstPattern.rows - radius)
                                continue;
                            cv::Mat ROI = dstPattern(cv::Range(pt.y-radius,pt.y+radius),cv::Range(pt.x-radius,pt.x+radius));
                            if(cv::countNonZero(ROI) == 0)
                                continue;

                            bool isOutOfRange = false;
                            //继续追踪点
                            for(int k1=1;k1<unitCornerPoints[j].size()-1;k1++)
                            {
                                int num = r==1? (k + k1)%unitCornerPoints[j].size() : (unitCornerPoints[j].size() + k - k1)%unitCornerPoints[j].size();
                                
                                std::array<cv::Point,2> unitVecs2;
                                unitCornerPoints[j][num].getVec(unitVecs2);
                                
                                cv::Point nextVec = getRVec(unitVecs2[1-r],-vec,unitVecs2[r]);
                                
                                vec.x = nextVec.x;
                                vec.y = nextVec.y;
                                
                                pt.x += nextVec.x;
                                pt.y += nextVec.y;
                                
                                //判定是否越界
                                if(pt.x <= radius || pt.y <= radius || pt.x >= dstPattern.cols - radius || pt.y >= dstPattern.rows - radius)
                                {
                                    isOutOfRange = true;
                                    break;
                                }
                                
                                cv::Mat ROI = dstPattern(cv::Range(pt.y-radius,pt.y+radius),cv::Range(pt.x-radius,pt.x+radius));
                                if(cv::countNonZero(ROI) == 0)
                                {
                                    isOutOfRange = true;
                                    break;
                                }
        
                                pts.push_back(pt);
                            }
                            if(isOutOfRange == true)
                                continue;

                            cv::Mat nextPattern;
                            bool outOfRangeJudge = cutPattern(dstPattern,pts,nextPattern);
                            if(outOfRangeJudge == false)
                                continue;
                            
                            resultUnitPos[j].assign(pts.begin(),pts.end());
                            
                            dstPatternFinder finder(nextPattern);
                            std::vector<cv::Point> nextDstPoints;
                            int judge = finder.getCorners(nextDstPoints);
                            if(judge == -1)
                                continue;
                            
                            std::vector<bool> nextIsUsed;
                            nextIsUsed.assign(isUsed.begin(),isUsed.end());
                            nextIsUsed[j] = true;
                            
                            std::vector<CornerPoint> nextDstCornerPoints;
                            cornerPointTransf(nextDstPoints, nextDstCornerPoints);
                            
                            //若单元块被全部使用，则终止递归
                            int sum=0;
                            for(int a=0;a<unitSize;a++)
                                sum += nextIsUsed[a];
                            std::cout<<"used="<<sum<<std::endl;
                            
                            //test
                            std::cout<<"originDst/nextPattern:"<<cv::countNonZero(originDst)/cv::countNonZero(nextPattern)<<std::endl;
                            cv::namedWindow("nextPattern",0);
                            cv::imshow("nextPattern",nextPattern);
                            if(cv::waitKey(1) == 27)
                                exit(0);

                            if(sum == unitSize && cv::countNonZero(originDst)/cv::countNonZero(nextPattern) > 10 )
                                return true;
                            
                            bool finalJudge = fit(dstPattern.size(), originDst, nextDstCornerPoints,unitCornerPoints,nextIsUsed,unitSize,isReversed, resultUnitPos);
                            if(finalJudge == true)
                                return true;
                        }
    
                    }
                }
            }
        }
    }
    
    return false;
}
