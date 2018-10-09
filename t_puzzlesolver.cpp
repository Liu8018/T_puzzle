#include "t_puzzlesolver.h"

T_puzzleSolver::T_puzzleSolver()
{
    m_solved = false;
    
    m_areaDistortionRatio = 0.99;
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
    
    m_unitSize = m_unitsPos.size();
    m_unitCornerPoints.clear();
    for (int i=0; i<m_unitSize; i++)
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
        m_dstResizeRatio = 1;
        return;
    }
    m_dstResizeRatio = size/(float)maxSide;
    
    newDstSize = cv::Size(m_dstImg.cols*m_dstResizeRatio,m_dstImg.rows*m_dstResizeRatio);
    cv::resize(m_dstImg, m_dstImg, newDstSize);
    
    cv::Size newUnitsSize = cv::Size(m_unitsImg.cols*m_dstResizeRatio,m_unitsImg.rows*m_dstResizeRatio);
    cv::resize(m_unitsImg,m_unitsImg,newUnitsSize);
}

void T_puzzleSolver::getDstPattern(std::vector<cv::Point> &dstPattern)
{
    for(int i=0;i<m_dstPattern.size();i++)
    {
        m_dstPattern[i].x *= 1/m_dstResizeRatio;
        m_dstPattern[i].y *= 1/m_dstResizeRatio;
    }
    
    dstPattern.assign(m_dstPattern.begin(),m_dstPattern.end());
}
void T_puzzleSolver::getUnitsPos(std::vector<std::vector<cv::Point>> &unitsPos)
{
    for(int i=0;i<m_unitsPos.size();i++)
    {
        for(int j=0;j<m_unitsPos[i].size();j++)
        {
            m_unitsPos[i][j].x *= 1/m_dstResizeRatio;
            m_unitsPos[i][j].y *= 1/m_dstResizeRatio;
        }
        
    }
    
    unitsPos.assign(m_unitsPos.begin(),m_unitsPos.end());
}

cv::Point T_puzzleSolver::getCenter(const std::vector<cv::Point> &pts)
{
    cv::RotatedRect rRect = cv::minAreaRect(pts);
    return rRect.center;
}

int T_puzzleSolver::getUnitSize()
{
    return m_unitSize;
}

bool T_puzzleSolver::solve()
{
    std::vector<bool> isUsed(m_unitsPos.size(),false);
    m_resultUnitsPos.resize(m_unitsPos.size());
    m_isReversed.resize(m_unitsPos.size());
    for(int i=0;i<m_unitSize;i++)
        m_isReversed[i] = false;
    
    m_runtime = (double)cv::getTickCount();
    m_solved = fit(m_dstImg.size(), m_dstCornerPoints, m_unitCornerPoints, isUsed, m_unitsPos.size(), m_isReversed, m_resultUnitsPos);
    m_runtime = ((double)cv::getTickCount() - m_runtime) / cv::getTickFrequency();
    
    if(m_solved)
        for(int i=0;i<m_resultUnitsPos.size();i++)
        {
            for(int j=0;j<m_resultUnitsPos[i].size();j++)
            {
                m_resultUnitsPos[i][j].x *= 1/m_dstResizeRatio;
                m_resultUnitsPos[i][j].y *= 1/m_dstResizeRatio;
            }
            
        }
    
    return m_solved;
}

void T_puzzleSolver::getResultPos(std::vector<bool> &isReversed, std::vector<std::vector<cv::Point>> &resultUnitsPos)
{
    if(!m_solved)
    {
        std::cout<<"Not solved!"<<std::endl;
        return;
    }
    
    resultUnitsPos.assign(m_resultUnitsPos.begin(),m_resultUnitsPos.end());
    isReversed.assign(m_isReversed.begin(),m_isReversed.end());
}

void vecsTransf(const std::vector<cv::Point> &pts, cv::Point center, std::vector<cv::Vec2i> &vecs)
{
    vecs.clear();
    for(int i=0;i<pts.size();i++)
    {
        cv::Vec2i vec;
        vec[0] = pts[i].x - center.x;
        vec[1] = pts[i].y - center.y;
        
        vecs.push_back(vec);
    }
}

void T_puzzleSolver::rotatePts(const std::vector<cv::Point> &pts, float angle, std::vector<cv::Point> &pts2)
{
    cv::Point cPt = getCenter(pts);
    
    std::vector<cv::Vec2i> vecs;
    vecsTransf(pts,cPt,vecs);
    
    float cos_theta = std::cos(angle);
    float sin_theta = std::sin(angle);
    
    pts2.clear();
    for(int i=0;i<vecs.size();i++)
    {
        cv::Point pt;
        pt.x = cPt.x +  vecs[i][0]*cos_theta + vecs[i][1]*sin_theta;
        pt.y = cPt.y + -vecs[i][0]*sin_theta + vecs[i][1]*cos_theta;
        
        pts2.push_back(pt);
    }
}

float T_puzzleSolver::getRotateAngle(const std::vector<cv::Point> &pts, const std::vector<cv::Point> &pts2)
{
    //找唯一角度的角
    /*std::vector<float> angleArray;
    std::vector<bool> repetitive(pts.size(),false);
    
    for(int i=0;i<pts.size();i++)
    {
        int pre = (i - 1 + pts.size())%pts.size();
        int nex = (i + 1)%pts.size();
        
        CornerPoint tmpCornerPt;
        tmpCornerPt.setPoints(pts[i],pts[pre],pts[nex]);
        angleArray.push_back(tmpCornerPt.getAngle());
        
        for(int j=0;j<angleArray.size();j++)
        {
            if(j == i)
                continue;
            
            if(std::abs(angleArray[i] - angleArray[j]) < 0.01)
            {
                repetitive[i] = true;
                repetitive[j] = true;
            }
        }
    }*/
    
    //找唯一长度的线段
    
    //若无唯一角度的角或唯一长度的线段，则遍历旋转角度找最佳值
    float minNotCoverRatio = 10;
    float correctTheta;
    
    cv::Mat tmpBgImg(3000,3000,CV_8U,cv::Scalar(0));
    std::vector<std::vector<cv::Point>> tmpPts;
    tmpPts.push_back(pts2);
    cv::drawContours(tmpBgImg,tmpPts,0,cv::Scalar(255),-1);
            
    //粗搜索
    for(float theta=0;theta<2*CV_PI;theta+=0.2)
    {
        std::vector<std::vector<cv::Point>> tmpUnitPos(1);
        rotatePts(pts,theta,tmpUnitPos[0]);

        cv::Mat tmpBgImg2(3000,3000,CV_8U,cv::Scalar(0));
        cv::drawContours(tmpBgImg2,tmpUnitPos,0,cv::Scalar(255),-1);
        
        cv::Mat notCoverImg = tmpBgImg != tmpBgImg2;
        float notCoverRatio = cv::countNonZero(notCoverImg)/(float)cv::countNonZero(tmpBgImg);
        
        if(notCoverRatio < minNotCoverRatio)
        {
            minNotCoverRatio = notCoverRatio;
            correctTheta = theta;
        }
        
    }
    
    //细搜索
    for(float theta = correctTheta - 0.2;theta<correctTheta + 0.2;theta+=0.02)
    {
        std::vector<std::vector<cv::Point>> tmpUnitPos(1);
        rotatePts(pts,theta,tmpUnitPos[0]);

        cv::Mat tmpBgImg2(3000,3000,CV_8U,cv::Scalar(0));
        cv::drawContours(tmpBgImg2,tmpUnitPos,0,cv::Scalar(255),-1);
        
        cv::Mat notCoverImg = tmpBgImg != tmpBgImg2;
        float notCoverRatio = cv::countNonZero(notCoverImg)/(float)cv::countNonZero(tmpBgImg);
        
        if(notCoverRatio < minNotCoverRatio)
        {
            minNotCoverRatio = notCoverRatio;
            correctTheta = theta;
        }
        
        //test
        /*cv::Mat testImg;
        cv::resize(tmpBgImg,testImg,cv::Size(600,600));
        cv::imshow("tmpBgimg",testImg);
        cv::resize(tmpBgImg2,testImg,cv::Size(600,600));
        cv::imshow("tmpBgimg2",testImg);
        if(cv::waitKey() == 'q')
            break;
        std::cout<<notCoverRatio<<std::endl;
        std::cout<<"theta:"<<theta<<std::endl;*/
        
    }
    
    return correctTheta;
}

void T_puzzleSolver::getTranslationAndRotation(std::vector<cv::Vec2i> &translationVectors, std::vector<float> &angles)
{
    //计算平移向量
    translationVectors.clear();
    
    for(int i=0;i<m_unitSize;i++)
    {
        cv::Point cPt1 = getCenter(m_unitsPos[i]);
        cv::Point cPt2 = getCenter(m_resultUnitsPos[i]);
        
        translationVectors.push_back(cv::Vec2i(cPt2.x-cPt1.x, cPt2.y-cPt1.y));
    }

    //计算旋转角度
    std::vector<std::vector<cv::Point>> unitsPos2;
    unitsPos2.assign(m_unitsPos.begin(),m_unitsPos.end());
    
    for(int i=0;i<unitsPos2.size();i++)
        for(int j=0;j<unitsPos2[i].size();j++)
        {
            unitsPos2[i][j].x += translationVectors[i][0];
            unitsPos2[i][j].y += translationVectors[i][1];
        }
    
    for(int i=0;i<unitsPos2.size();i++)
    {
        float angle = getRotateAngle(unitsPos2[i],m_resultUnitsPos[i]);
        angles.push_back(angle);
    }
}

float T_puzzleSolver::rad2angle(float rad)
{
    return rad*180/(float)CV_PI;
}

double T_puzzleSolver::runtime()
{
    return m_runtime;
}

void T_puzzleSolver::cornerPointTransf(const std::vector<cv::Point> &approxPoints, std::vector<CornerPoint> &cornerPoints)
{
    int len = approxPoints.size();
    if(len < 3)
    {
        std::cout<<"approxPoints.size()="<<len<<std::endl;
        std::cout<<"cornerPointTransf error!"<<std::endl;
        cv::waitKey();
        exit(0);
    }
    cornerPoints.resize(len);

    for(int i=1;i<len-1;i++)
        cornerPoints[i].setPoints(approxPoints[i], approxPoints[i-1], approxPoints[i+1]);

    cornerPoints[0].setPoints(approxPoints[0], approxPoints[len-1], approxPoints[1]);
    cornerPoints[len-1].setPoints(approxPoints[len-1], approxPoints[len-2], approxPoints[0]);
}

float T_puzzleSolver::getNorm(cv::Point vec)
{
    return sqrt(vec.x*vec.x + vec.y*vec.y);
}

cv::Point2f T_puzzleSolver::getRVec(cv::Point2f vec1, cv::Point2f vec2, cv::Point2f vec3)
{
    double x1 = vec1.x;
    double y1 = vec1.y;
    
    double x2 = vec2.x;
    double y2 = vec2.y;
    
    double sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
    double cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
    
    double x3 = vec3.x;
    double y3 = vec3.y;
    
    cv::Point2f nextVec;
    nextVec.y = y3*cos_theta - x3*sin_theta;
    nextVec.x = y3*sin_theta + x3*cos_theta;
    
    return nextVec;
}

void T_puzzleSolver::setDistortionPara(float areaRatio)
{
    m_areaDistortionRatio = areaRatio;
}

bool T_puzzleSolver::cutPattern(const cv::Mat &pattern, const std::vector<cv::Point2f> &pts, cv::Mat &nextPattern)
{
    std::vector<std::vector<cv::Point>> contour(1);
    for(int i=0;i<pts.size();i++)
    {
        cv::Point tmpPt = pts[i];
        contour[0].push_back(tmpPt);
    }
    
    cv::Mat tmpImg(pattern.size(),CV_8U,cv::Scalar(1));
    cv::drawContours(tmpImg, contour, 0, 255, -1);
    
    cv::Mat tmpImg2 = (tmpImg == pattern);
    tmpImg = (tmpImg != cv::Mat(tmpImg.size(),CV_8U,cv::Scalar(1)));
    float ratio = cv::countNonZero(tmpImg2) / (float)cv::countNonZero(tmpImg);
    
    //std::cout<<"ratio="<<ratio<<std::endl;
    
    if(ratio < m_areaDistortionRatio)
        return false;
    
    pattern.copyTo(nextPattern);
    cv::drawContours(nextPattern, contour, 0, cv::Scalar(0), -1);
    
    //cv::imshow("nextPattern",nextPattern);
    
    //删除零碎部分
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(nextPattern, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int deletedPartsNum=0;
    for(int i=0;i<contours.size();i++)
    {
        double area = cv::contourArea(contours[i]);
        if(area < 200)
        {
            cv::drawContours(nextPattern,contours,i,cv::Scalar(0),-1);
            deletedPartsNum++;
        }
        else
        {
            cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            cv::Point2f pts[4];
            rect.points(pts);
            int width = std::sqrt((pts[0].x-pts[1].x)*(pts[0].x-pts[1].x)+(pts[0].y-pts[1].y)*(pts[0].y-pts[1].y)) + 1;
            int height = std::sqrt((pts[2].x-pts[1].x)*(pts[2].x-pts[1].x)+(pts[2].y-pts[1].y)*(pts[2].y-pts[1].y)) + 1;
            
            if(width / height > 6 || height / width > 6)
            {
                cv::drawContours(nextPattern,contours,i,cv::Scalar(0),-1);
                deletedPartsNum++;
            }
        }
    }
    if(contours.size() - deletedPartsNum > 1)
        return false;
    
    //cv::imshow("nextPattern1",nextPattern);
    //cv::waitKey();
    
    return true;
}

bool T_puzzleSolver::fit(const cv::Size &imgSize, 
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
        dstContour.push_back(cv::Point(dstCornerPoints[i].x(), dstCornerPoints[i].y()));
    std::vector<std::vector<cv::Point>> dstContours;
    dstContours.push_back(dstContour);
    cv::Mat dstPattern(imgSize,CV_8U,cv::Scalar(0));
    cv::drawContours(dstPattern,dstContours,0,cv::Scalar(255),-1);

    //test
    //cv::namedWindow("testImg0",0);
    //cv::imshow("testImg0",dstPattern);
    //cv::waitKey();

    //遍历
    for(int i=0;i<dstCornerPoints.size();i++)//目标图案每个角点
    {
//std::cout<<"i="<<i<<std::endl;
        for(int j=0;j<unitSize;j++)//每个单元块
        {
//std::cout<<"j="<<j<<std::endl;
            //若该单元块已使用过，则跳过
            if(isUsed[j] == true)
                continue;
            for(int rev=0;rev<=1;rev++)//单元块两个放置面
            {
                bool iR = rev==0 ? false : true;
//std::cout<<"iR="<<iR<<std::endl;
//std::cout<<"isReversed["<<j<<"]="<<isReversed[j]<<std::endl;
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
//std::cout<<"--isReversed["<<j<<"]="<<isReversed[j]<<std::endl;
                
                for(int k=0;k<unitCornerPoints[j].size();k++)//单元块每个角点
                {
//std::cout<<"k="<<k<<std::endl;
                    for(int q=0;q<=1;q++)//角点两个放置方向
                    {
//std::cout<<"    q="<<q<<std::endl;
                        for(int r=0;r<=1;r++)//目标角两条侧边
                        {
//std::cout<<"        r="<<r<<std::endl;
                            //定义容器存储点集
                            std::vector<cv::Point2f> pts;
                            pts.push_back(cv::Point2f(dstCornerPoints[i].x(),dstCornerPoints[i].y()));

                            //定位下一个点
                            std::array<cv::Point2f,2> dstVecs;
                            dstCornerPoints[i].getVec(dstVecs);
                            std::array<cv::Point2f,2> unitVecs;
                            unitCornerPoints[j][k].getVec(unitVecs);

                            float dstVecNorm = getNorm(dstVecs[q]);
                            float unitVecNorm = getNorm(unitVecs[r]);
                            float ratio = unitVecNorm/dstVecNorm;

                            cv::Point2f vec,pt;
                            vec.x = dstVecs[q].x * ratio;
                            vec.y = dstVecs[q].y * ratio;
                            pt.x = vec.x + dstCornerPoints[i].x();
                            pt.y = vec.y + dstCornerPoints[i].y();

                            pts.push_back(pt);

                            //若在目标图案外，则跳过
                            int radius = 5;
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
                                
                                std::array<cv::Point2f,2> unitVecs2;
                                unitCornerPoints[j][num].getVec(unitVecs2);
                                
                                cv::Point2f nextVec = getRVec(unitVecs2[1-r],-vec,unitVecs2[r]);
                                
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
                            {
                                if(nextDstPoints.empty())
                                    return true;
                                
                                continue;
                            }
                            
                            std::vector<bool> nextIsUsed;
                            nextIsUsed.assign(isUsed.begin(),isUsed.end());
                            nextIsUsed[j] = true;
                            
                            std::vector<CornerPoint> nextDstCornerPoints;
                            cornerPointTransf(nextDstPoints, nextDstCornerPoints);
                            
                            //若单元块被全部使用，则终止递归
                            int sum=0;
                            for(int a=0;a<unitSize;a++)
                                sum += nextIsUsed[a];
                            
                            //test
                            /*std::cout<<"used="<<sum<<std::endl;
                            cv::namedWindow("nextPattern",0);
                            cv::imshow("nextPattern",nextPattern);
                            if(cv::waitKey(1) == 27)
                                exit(0);*/
                            
                            if(sum >= unitSize)
                                return true;
                            
                            bool finalJudge = fit(dstPattern.size(), nextDstCornerPoints,unitCornerPoints,nextIsUsed,unitSize,isReversed, resultUnitPos);
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
