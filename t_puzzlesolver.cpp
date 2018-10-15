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
        m_isReversed[i] = true;
    
    m_runtime = (double)cv::getTickCount();
    m_solved = fit(m_dstImg.size(), m_dstCornerPoints, m_unitCornerPoints, isUsed, m_unitsPos.size(), m_isReversed, m_resultUnitsPos);
    m_runtime = ((double)cv::getTickCount() - m_runtime) / cv::getTickFrequency();
    
    if(m_solved)
    {
        for(int i=0;i<m_unitSize;i++)
        {
            for(int j=0;j<m_resultUnitsPos[i].size();j++)
            {
                m_resultUnitsPos[i][j].x *= 1/m_dstResizeRatio;
                m_resultUnitsPos[i][j].y *= 1/m_dstResizeRatio;
            }
            
            m_isReversed[i] = !m_isReversed[i];
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

cv::Point T_puzzleSolver::getCenter(const std::vector<cv::Point> &pts)
{
    cv::Moments mu = cv::moments(pts);
    return cv::Point( mu.m10/mu.m00, mu.m01/mu.m00);
}

float T_puzzleSolver::getRotateAngle(const std::vector<cv::Point> &pts0, const std::vector<cv::Point> &pts1)
{
    //平移使质心重合
    cv::Point cPt0 = getCenter(pts0);
    cv::Point cPt1 = getCenter(pts1);
    cv::Vec2i transVec = cv::Vec2i(cPt1.x-cPt0.x, cPt1.y-cPt0.y);
    
    std::vector<cv::Point> pts;
    pts.assign(pts1.begin(),pts1.end());
    
    for(auto pt : pts)
    {
        pt.x += transVec[0];
        pt.y += transVec[1];
    }
    
    //获取中心向量
    cv::Vec2i centerVec0;
    std::vector<cv::Vec2i> centerVecs1;
    
    centerVec0 = cv::Vec2i(pts0[0].x-cPt0.x, pts0[0].y-cPt0.y);
    
    for(auto pt : pts1)
        centerVecs1.push_back(cv::Vec2i(pt.x-cPt1.x,pt.y-cPt1.y));
    
    //旋转并记录重合率,找到centerVec0对应的向量
    int minNotIntersec = 3000*3000;
    int correspondVecId = 0;
    for(int i=0;i<centerVecs1.size();i++)
    {
        std::vector<cv::Point> pts2;
        
        //旋转
        for(int j=0;j<centerVecs1.size();j++)
        {
            //计算单位向量
            float abs_v1 = std::sqrt(centerVecs1[i][0]*centerVecs1[i][0] + centerVecs1[i][1]*centerVecs1[i][1]);
            float abs_v2 = std::sqrt(centerVec0[0]*centerVec0[0] + centerVec0[1]*centerVec0[1]);

            cv::Point2f vec2;
            getRVec(cv::Point(100*centerVecs1[i][0]/abs_v1,100*centerVecs1[i][1]/abs_v1),
                    cv::Point(100*centerVec0[0]/abs_v2, 100*centerVec0[1]/abs_v2),
                    cv::Point(centerVecs1[j][0], centerVecs1[j][1]),
                    vec2);
            
            pts2.push_back(cv::Point(cPt0.x+vec2.x, cPt0.y+vec2.y));
        }
        
        //计算不重合部分面积
        cv::Mat tmpImg1(3000,3000,CV_8U,cv::Scalar(0));
        cv::Mat tmpImg2(3000,3000,CV_8U,cv::Scalar(0));
        
        std::vector<std::vector<cv::Point>> tmpPts;
        tmpPts.push_back(pts0);
        cv::drawContours(tmpImg1,tmpPts,0,cv::Scalar(255),-1);
        tmpPts[0].assign(pts2.begin(),pts2.end());
        cv::drawContours(tmpImg2,tmpPts,0,cv::Scalar(255),-1);
        
        int notIntersec = cv::countNonZero(tmpImg1 != tmpImg2);
        if(notIntersec < minNotIntersec)
        {
            minNotIntersec = notIntersec;
            correspondVecId = i;
        }
        
        /*cv::resize(tmpImg1,tmpImg1,cv::Size(1200,1200));
        cv::resize(tmpImg2,tmpImg2,cv::Size(1200,1200));
        cv::imshow("tmpImg1",tmpImg1);
        cv::imshow("tmpImg2",tmpImg2);
        cv::waitKey();*/
    }
    
    //计算旋转角度    
    float abs_v0 = std::sqrt(centerVec0[0]*centerVec0[0] + centerVec0[1]*centerVec0[1]);
    float abs_v1 = std::sqrt(centerVecs1[correspondVecId][0]*centerVecs1[correspondVecId][0] + 
                    centerVecs1[correspondVecId][1]*centerVecs1[correspondVecId][1]);
    
    int judge = centerVec0[0]*centerVecs1[correspondVecId][1] - centerVec0[1]*centerVecs1[correspondVecId][0];
    float cos_theta = (centerVec0[0]*centerVecs1[correspondVecId][0] + centerVec0[1]*centerVecs1[correspondVecId][1])/(abs_v0*abs_v1);
    float theta = std::acos(cos_theta);
    theta = (judge > 0)? theta:2*CV_PI-theta;
    return theta;
}

void T_puzzleSolver::getTranslationAndRotation(std::vector<cv::Vec2i> &translationVectors, std::vector<float> &angles)
{
    translationVectors.clear();
    
    for(int i=0;i<m_unitSize;i++)
    {
        //获取轮廓质心
        cv::Point srcCenter = getCenter(m_unitsPos[i]);
        cv::Point dstCenter = getCenter(m_resultUnitsPos[i]);
        
        //计算平移向量
        cv::Vec2i transVec;
        transVec[0] = dstCenter.x - srcCenter.x;
        transVec[1] = dstCenter.y - srcCenter.y;
        translationVectors.push_back(transVec);
        
        //计算旋转角度
        float angle = getRotateAngle(m_unitsPos[i], m_resultUnitsPos[i]);
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

void T_puzzleSolver::getRVec(cv::Point2f vec1, //旋转前的向量a
                             cv::Point2f vec2, //旋转后的向量a
                             cv::Point2f vec3, //旋转前的向量b
                             cv::Point2f &vec4) //旋转后的向量b
{
    double x1 = vec1.x;
    double y1 = vec1.y;
    
    double x2 = vec2.x;
    double y2 = vec2.y;
    
    double sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
    double cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
    
    double x3 = vec3.x;
    double y3 = vec3.y;
    
    vec4.y = y3*cos_theta - x3*sin_theta;
    vec4.x = y3*sin_theta + x3*cos_theta;
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
                    for(int q=0;q<=1;q++)//角点两个放置方向
                    {
                        for(int r=0;r<=1;r++)//目标角两条侧边
                        {
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
                                
                                cv::Point2f nextVec;
                                getRVec(unitVecs2[1-r],-vec,unitVecs2[r],nextVec);
                                
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
                            
                            dstPatternFinder finder(nextPattern);
                            std::vector<cv::Point> nextDstPoints;
                            bool judge = finder.getCorners(nextDstPoints);
                            if(judge == false && !nextDstPoints.empty())
                                continue;
                            
                            //为下一轮递归的isUsed赋值
                            std::vector<bool> nextIsUsed;
                            nextIsUsed.assign(isUsed.begin(),isUsed.end());
                            nextIsUsed[j] = true;
                            
                            //为单元块返回位置信息赋值
                            resultUnitPos[j].assign(pts.begin(),pts.end());
                            
                            //计算已放置的单元块数目
                            int sum=0;
                            for(int a=0;a<unitSize;a++)
                                sum += nextIsUsed[a];
                            //test
                            /*std::cout<<"used="<<sum<<std::endl;
                            cv::cvtColor(nextPattern,nextPattern,cv::COLOR_GRAY2BGR);
                            for(int t1=0;t1<m_unitSize;t1++)
                            {
                                if(nextIsUsed[t1] == true)
                                {
                                    cv::drawContours(nextPattern,resultUnitPos,t1,cv::Scalar(255,0,0));
                                    cv::putText(nextPattern,std::to_string(isReversed[t1]),getCenter(resultUnitPos[t1]),1,1,cv::Scalar(0,200,200));
                                }
                            }
                            cv::imshow("nextPattern",nextPattern);
                            if(cv::waitKey(1) == 27)
                                exit(0);*/
                            
                            //若单元块放置完毕，则终止递归
                            if(sum == unitSize)
                                return true;
                            
                            std::vector<CornerPoint> nextDstCornerPoints;
                            cornerPointTransf(nextDstPoints, nextDstCornerPoints);
                            
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
