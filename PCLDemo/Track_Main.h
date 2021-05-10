#pragma once
#ifndef _TRACK_MAIN_H
#define _TRACK_MAIN_H
#include<iostream>
#include<ctime>
#include<math.h>	
#include<opencv2/opencv.hpp>
#define PI 3.1415926535897932384626433832
#define DistErr 6
using namespace std;
using namespace cv;
///结构体变量
struct TargetBox
{
	int Left_Top_Pos_x;
	int Left_Top_Pos_y;
	int width;
	int height;
};

struct ParamInt2D
{
	int x;
	int y;
	ParamInt2D() :x(0), y(0) {};
	ParamInt2D(int x_p, int y_p) :x(x_p), y(y_p) {};
};
struct ParamFloat2D
{
	float x;
	float y;
	ParamFloat2D() :x(0), y(0) {};
	ParamFloat2D(float x_p, float y_p) :x(x_p), y(y_p) {};
};


class myTrack
{
public:
	myTrack();
	void InitParam();
	ParamInt2D GetDepthImageTargetPix(float myDist, ParamFloat2D TargetSize);
	int GetDistTimeFromDepth(float myDist, int offset);
	float GetProbability(float myDist, float offset, int sigma);
	Mat Track(float myDist, Mat Src, TargetBox &TargetBeginBox);
	void cvShowImage(Mat res, Rect rect);
	void TrackMain( Mat src);
	void AddRectangularBox();
	void detactByIntense(Mat &depth, Mat &intense);
	void showImageColor(cv::Mat mat, string winname);
	ushort getCurrentDepth(Mat &depth, Point2i targetCenter);
	ushort targetDepth;		//目标在距离像中的深度值
	Rect targetBox;
	Point2i targetCenter;
public:
	float Fov;             //雷达视场角
	int ImagePixRes;     ///图像分辨率(64*64)
	ParamFloat2D TargetRealSize;    //目标真实大小
	ParamInt2D TargetImageSize;     //距离像中目标成像大小
	float Begin_P ;   ///初始目标像素点出现的概率;
	float End_p ;   //结束目标像素点出现的概率;
	float Begin_Dist;     ///目标初始相对距离
	float End_Dist;   //目标结束相对距离
	float Target_Velocity;   //目标运动速度
	int APD_FrameRato;    //APD成像帧率；
	Mat srcImage;
	Mat resImage;

	TargetBox TargetBefBox;    //初始划取(前一帧)的目标框
	TargetBox TargetTrackBox;      //跟踪后的目标框
	ParamFloat2D centerPos;    //目标中心坐标

	TargetBox RectangularBox;  //画矩形框
	ParamFloat2D RectangularCenterPos;    //矩形框中心坐标
	
	float padding;    //扩大包围框
	int THRE;

	bool FirstFrame;  //跟踪首帧标志
	int boxLeftTopX;
	int boxLeftTopY;
	int boxWidth;
	int boxHeight;
};


#endif