#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

class cmp
{
public:
    bool operator()(const Point a, const Point b)
    {
        return a.x < b.x;
    }
};

void contrast_ratio(Mat& frame)
{
    int rows = frame.rows;
    int cols = frame.cols;
    double alpha = 1.2;
    int beta = 50;

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            frame.ptr<Vec3b>(i)[j][0] = saturate_cast<uchar>(alpha * frame.ptr<Vec3b>(i)[j][0] + beta);
            frame.ptr<Vec3b>(i)[j][1] = saturate_cast<uchar>(alpha * frame.ptr<Vec3b>(i)[j][1] + beta);
            frame.ptr<Vec3b>(i)[j][2] = saturate_cast<uchar>(alpha * frame.ptr<Vec3b>(i)[j][2] + beta);
        }
    }
}

vector<Point> FindCoutours(Mat& frame_edge)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(frame_edge, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    vector<Point> hand_contours;
    double max_Area = 0;
    double Area=0;
    for(int i = 0; i < (int)contours.size(); i++)
    {
        Area = contourArea(contours[i]);
        if(Area > max_Area)
        {
            max_Area = Area;
            hand_contours = contours[i];
        }
    }
    return hand_contours;
}


pair<vector<Point>, vector<int>> FindConvexHull(vector<Point>& contour)
{
    vector<Point> PointHull;
    vector<int> intHull;
    convexHull(contour, PointHull, true);
    convexHull(contour, intHull, true);
    return pair<vector<Point>, vector<int>>(PointHull, intHull);
}

/*
HullType FindConvexHull(vector<Point>& contour)
{
    if(contour.empty())
    {
        return 0;
    }
    HullType Hull;
    convexHull(contour, Hull.PointHull, false);
    convexHull(contour, Hull.intHull, false);
    return Hull;
}
*/

vector<Vec4i> FindConvexityDefects(vector<Point>& contour, vector<int>& intHull)
{
    vector<Vec4i> defects;
    convexityDefects(contour, intHull, defects);
    return defects;
}

int Judge01(vector<Point>& contour, vector<Point>& PointHull)
{
    double areacon = contourArea(contour);
    double areahul = contourArea(PointHull) / 256;
    cout << areacon << endl << areahul << endl;
    return areacon > areahul ?  -1 :  0;
}


double PointRange(Point a, Point b)
{
    return abs(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

int Denoising(set<Point, cmp>& far_array)
{
    set<Point, cmp>::iterator it = far_array.begin();
    set<Point, cmp>::iterator next = ++far_array.begin();
    while(next != far_array.end())
    {
        if(PointRange(*it, *next) < 300)
        {
            cout << far_array.size() << endl;
            cout << *it << endl;

            far_array.erase(it);
        }
        it = next;
        ++next;
    }
    return (int)far_array.size();
}

int JudgeHand(Mat& frame, vector<Point>& contour, vector<Vec4i>& defects, vector<Point>& PointHull)
{
    int finger = 0;
    set<Point, cmp> far_array;
    for(size_t i = 0; i < defects.size(); i++)
    {
        double farRange = defects[i][3] / 256;

        Point start = contour[defects[i][0]];
        Point end = contour[defects[i][1]];
        Point far = contour[defects[i][2]];

        double a = norm(end - start);
        double b = norm(far - start);
        double c = norm(end - far);

        double angle = acos((b * b + c * c - a * a) / (2 * b * c)) * 57;
        //cout << angle << endl;

        if(angle <= 90 && farRange > 50 && farRange < 250)
        {
            cout << start << end << far << endl;
            cout << farRange << endl;
            circle(frame, far, 3, Scalar(255, 0, 0), -1);
            far_array.insert(far);
            ++finger;
        }
    }

    if(finger == 0)
    {
        finger = Judge01(contour, PointHull);
    }
    else
        finger = Denoising(far_array);
    return ++finger;
}

int main()
{
    VideoCapture capture;
    Mat frame, frame_gray, frame_edge;

    //capture.open(0, CAP_DSHOW);
    capture.open("f.mp4");
    size_t fps = capture.get(CAP_PROP_FRAME_COUNT);
    cout << fps << endl;

/*
    for(int i = 0; i < 30; i++)
    {
        capture.read(frame);
        flip(frame, frame, 1);
        imshow("frame", frame);
    }
*/

    size_t i = 0;
    while(i < fps)
    {
        capture.read(frame);        //3通道
        if(frame.empty())
            break;
        //高斯金字塔-向下采样
        //pyrDown(frame, frame);
        pyrDown(frame, frame);
        //灰度转换
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        //imshow("gray", frame_gray);
        //滤波
        //bilateralFilter(frame_gray, frame_edge, 9, 100, 100);
        GaussianBlur(frame_gray, frame_gray, Size(5, 5), 0, 0);
        //边缘检测
        Canny(frame_gray, frame_edge, 50, 150);
        //adaptiveThreshold(frame_gray, frame_edge, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 7, 5);
        imshow("canny_test", frame_edge);
        //寻找轮廓
        vector<Point> contour = FindCoutours(frame_edge);
        if(contour.empty())
        {
            continue;
        }
        else
        {
            //轮廓逼近
            double epsilon = 0.001 * arcLength(contour, true);
            approxPolyDP(contour, contour, epsilon, true);
            //查找凸包
            pair<vector<Point>, vector<int>> Hull = FindConvexHull(contour);
            //单调
            sort(Hull.second.begin(), Hull.second.end());
            //查找凸缺陷
            vector<Vec4i> defects = FindConvexityDefects(contour, Hull.second);
            //绘制多边形
            polylines(frame, Hull.first, true,Scalar::all(255), 2, LINE_8, 0);
            //判断手势
            int finger = JudgeHand(frame, contour, defects, Hull.first);
            //打印
            char str[30] = "";
            sprintf(str, "finger:%d", finger);
            putText(frame, str, Point(1, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, LINE_8, false);
        }

        namedWindow("frame", WINDOW_AUTOSIZE);
        imshow("frame", frame);
        waitKey(15);
        i++;
    }
    destroyAllWindows();

/*
    Mat frame, frame_gray, frame_edge;
    frame = imread("f5a.jpg");
    //高斯金字塔-向下采样
    pyrDown(frame, frame);
    pyrDown(frame, frame);
    //灰度转换
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    //imshow("gray", frame_gray);
    //滤波
    //bilateralFilter(frame_gray, frame_edge, 9, 100, 100);
    GaussianBlur(frame_gray, frame_gray, Size(5, 5), 0, 0);
    //边缘检测
    Canny(frame_gray, frame_edge, 50, 150);
    //adaptiveThreshold(frame_gray, frame_edge, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 7, 5);
    imshow("canny_test", frame_edge);
    //寻找轮廓
    vector<Point> contour = FindCoutours(frame_edge);
    //轮廓逼近
    double epsilon = 0.001 * arcLength(contour, true);
    approxPolyDP(contour, contour, epsilon, true);
    //查找凸包
    pair<vector<Point>, vector<int>> Hull = FindConvexHull(contour);
    //HullType Hull = FindConvexHull(contour);
    //单调
    sort(Hull.second.begin(), Hull.second.end());
    //查找凸缺陷
    vector<Vec4i> defects = FindConvexityDefects(contour, Hull.second);
    //绘制多边形
    polylines(frame, Hull.first, true,Scalar::all(255), 2, LINE_8, 0);
    //判断手势
    int finger = JudgeHand(frame, contour, defects, Hull.first);
    //打印
    char str[30] = "";
    sprintf(str, "finger:%d", finger);
    putText(frame, str, Point(1, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, LINE_8, false);

    namedWindow("frame", WINDOW_AUTOSIZE);
    imshow("frame", frame);
*/
    waitKey(0);
    return 0;
}
