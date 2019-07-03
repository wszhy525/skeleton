#include <iostream>
#include<skeleton.h>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

Skeleton::Skeleton()
{
    MAXRANGE = 2500;  //设置最大边框检测范围
    MINRANGE = 100; //设置最小边框检测范围
    _DIFFER_DISTANCE = 500; //设置最小变动范围
    MAXMAXRANGE = 6000;  //整个扇叶的外包矩形面积
    _isfund=false;      //初始化是否寻找到打击中心标志量为false
    Lead=0;                //set the lead
    Offset=0.001;
    _isclockwise = 0;      //shunshizheng  clockwise
    //_isclockwise = 1;       //nishizheng   anticlockwise
    //    createTrackbar("large threshold","parameter of the large threshold",&MAXRANGE,4000,large_threshold);
    //    large_threshold(0,0);
    //    void large_threshold(int, void*)
    //    {

    //    }
}

Skeleton::~Skeleton()
{

}
Mat Skeleton::loadImg(Mat frame_)
{
    _srcImg = frame_;
    return _srcImg;
}

cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1)
{
    assert(src.type() == CV_8UC1);
    cv::Mat dst;
    int width = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    int count = 0;  //记录迭代次数
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
            break;
        std::vector<uchar *> mFlag; //用于标记需要删除的点
        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }

        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
    }
    return dst;
}

/**
* @brief 对骨骼化图数据进行过滤，实现两个点之间至少隔一个空白像素
* @param thinSrc为输入的骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
*/
void filterOver(cv::Mat thinSrc)
{
    assert(thinSrc.type() == CV_8UC1);
    int width = thinSrc.cols;
    int height = thinSrc.rows;
    for (int i = 0; i < height; ++i)
    {
        uchar * p = thinSrc.ptr<uchar>(i);
        for (int j = 0; j < width; ++j)
        {
            // 实现两个点之间至少隔一个像素
            //  p9 p2 p3
            //  p8 p1 p4
            //  p7 p6 p5
            uchar p1 = p[j];
            if (p1 != 1) continue;
            uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
            uchar p8 = (j == 0) ? 0 : *(p + j - 1);
            uchar p2 = (i == 0) ? 0 : *(p - thinSrc.step + j);
            uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - thinSrc.step + j + 1);
            uchar p9 = (i == 0 || j == 0) ? 0 : *(p - thinSrc.step + j - 1);
            uchar p6 = (i == height - 1) ? 0 : *(p + thinSrc.step + j);
            uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + thinSrc.step + j + 1);
            uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + thinSrc.step + j - 1);
            if (p2 + p3 + p8 + p9 >= 1)
            {
                p[j] = 0;
            }
        }
    }
}

/**
* @brief 从过滤后的骨骼化图像中寻找端点和交叉点
* @param thinSrc为输入的过滤后骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
* @param raudis卷积半径，以当前像素点位圆心，在圆范围内判断点是否为端点或交叉点
* @param thresholdMax交叉点阈值，大于这个值为交叉点
* @param thresholdMin端点阈值，小于这个值为端点
* @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
*/
std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis = 4, unsigned int thresholdMax = 6, unsigned int thresholdMin = 4)
{
    assert(thinSrc.type() == CV_8UC1);
    int width = thinSrc.cols;
    int height = thinSrc.rows;
    cv::Mat tmp;
    thinSrc.copyTo(tmp);
    std::vector<cv::Point> points;
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (*(tmp.data + tmp.step * i + j) == 0)
            {
                continue;
            }
            int count=0;
            for (int k = i - raudis; k < i + raudis+1; k++)
            {
                for (int l = j - raudis; l < j + raudis+1; l++)
                {
                    if (k < 0 || l < 0||k>height-1||l>width-1)
                    {
                        continue;

                    }
                    else if (*(tmp.data + tmp.step * k + l) == 1)
                    {
                        count++;
                    }
                }
            }

            if (count > thresholdMax||count<thresholdMin)
            {
                Point point(j, i);
                points.push_back(point);
            }
        }
    }
    return points;
}

double Skeleton:: Max(double a, double b, double c)
{
    double x = a;
    if (a > b)
        x = a;
    else
        x = b;
    if (x > c)
        x = x;
    else
        x = c;
    return x;
}
double Skeleton:: Min(double a, double b, double c)
{
    double x = a;
    if (a < b)
        x = a;
    else
        x = b;
    if (x < c)
        x = x;
    else
        x = c;
    return x;
}
void Skeleton:: calculateTheHSV(double R, double G, double B)
{
    double max = Max(R, G, B);
    double min = Min(R, G, B);
    V = Max(R, G, B);
    S = (max - min) / max;


    if (R == max) H = (G - B) / (max - min) * 60;
    if (G == max) H = 120 + (B - R) / (max - min) * 60;
    if (B == max) H = 240 + (R - G) / (max - min) * 60;
    if (H < 0) H = H + 360;
}
bool Skeleton:: getDistance( const Vec3b& color)
{
    int judgement = 0;
    calculateTheHSV(color[2], color[1], color[0]);

    //	双重检测，首先去掉过于亮或者暗的区域，再用HSV判断来精确定位
    //    (color[2] < _largeThreshold)&&(color[1] < _largeThreshold)&&(color[0] < _largeThreshold)&&
    //    if ((color[2] > _smallThreshold)&&(color[2] > _smallThreshold)&&(0<H<100||200<H<360)
    //            &&(S > 0.3)&&(V > 16))  //去掉全是白光的灯光噪声   //V 的范围  //S的范围  //H的范围 //R必须大于100才是需要的地方
    if(color[0]>160&&color[1]>160&&color[2]>160)
    {
        return 0;
    }
    else
    {
        if(((color[2]-color[0]) > 90||(color[2]-color[0]) < -90)&&(0<H<100||200<H<360)&&(S > 0.3)&&(V > 16))

        {
            return 1;
        }

        else
            return 0;
    }
}
void Skeleton:: Detect()
{
    int  newLongth=0;

    cv::Point2f points4[4];
    int count=0;
    RotatedRect rotated_rect;

    //threshold(_srcImg,_srcImg,205,255,THRESH_BINARY);

    Mat _processedImg(_srcImg.size(), CV_8U, Scalar(0)); //初始化一张和原图等大的二值化图来装载红色结果区域。

    Mat_ <cv::Vec3b> ::const_iterator it = _srcImg.begin<Vec3b>();
    Mat_ <cv::Vec3b> ::const_iterator itend = _srcImg.end<Vec3b>();
    Mat_ <uchar>::iterator itout = _processedImg.begin<uchar>();
    //        Mat prc=_srcImg.clone();
    //        Mat_ <cv::Vec3b> ::const_iterator it1 = prc.begin<Vec3b>();
    //        Mat_ <cv::Vec3b> ::const_iterator itend1 = prc.end<Vec3b>();

    for (it; it != itend; ++it, ++itout)
    {

        if (getDistance(*it))
        {
            *itout = 255;
        }
        else
        {
            *itout = 0;
        }
    }

    imshow("threshold", _processedImg);
    waitKey(1);
    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    dilate(_processedImg,_processedImg,element);//通过膨胀使得轮廓更加清晰
    // #ifdef DEBUG
    imshow("hsv", _processedImg);
    waitKey(1);
    //#endif
    cv::Mat dst = thinImage(_processedImg);
    //过滤细化后的图像
    filterOver(dst);
    //查找端点和交叉点
    std::vector<cv::Point> points = getPoints(dst,6,9,6);
    //二值图转化成灰度图，并绘制找到的点
   // dst = dst * 255;
    _processedImg = _processedImg * 255;
    vector<cv::Point>::iterator itt = points.begin();
    for (;itt != points.end(); itt++)
    {
        circle(dst, *itt,4,255, 1);
    }
    imwrite("dst.jpg", dst);
    //显示图像
    cv::namedWindow("src1", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("dst1", CV_WINDOW_AUTOSIZE);
    cv::imshow("src1", _processedImg);
    cv::imshow("dst1", dst);
    findContours(_processedImg, contours, RETR_TREE, CV_CHAIN_APPROX_NONE);
    vector<vector<Point> >::iterator itc = contours.begin();
    while (itc!=contours.end())
    {
        if(contourArea(*itc) > MAXMAXRANGE) //删除大扇叶里的三个小轮廓
        {
            itc++;
            while((itc!=contours.end())&&(contourArea(*itc) < MAXRANGE))
            {
                itc = contours.erase(itc);
            }
        }
        else if(contourArea(*itc) < MINRANGE) //删除杂质轮廓
        {
            itc = contours.erase(itc);
        }
        else
        {
            itc++;
        }
    }


    Mat _processedImg1(_srcImg.size(), CV_8U, Scalar(0));


    drawContours(_processedImg1, contours, -1, Scalar(255), 1);

#ifdef DEBUG
    imshow("contours imshow", _processedImg1);
    waitKey(1);
#endif

}
int main()
{
    Skeleton windmill_detector;

    VideoCapture capture("/home/zhy/Desktop/4.avi");
    Mat src;
    capture >> src;
    // DEBUG_VIDEO
    // cv::Mat src=imread("/home/zhy/Desktop/123.png",0);
    //获取图像
    while(1){
        windmill_detector.loadImg(src);
        windmill_detector.Detect();
        waitKey(20);

        //图像细化，骨骼化


    }
    return 0;
}

