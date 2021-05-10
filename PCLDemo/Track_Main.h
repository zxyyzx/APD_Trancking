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
///�ṹ�����
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
	ushort targetDepth;		//Ŀ���ھ������е����ֵ
	Rect targetBox;
	Point2i targetCenter;
public:
	float Fov;             //�״��ӳ���
	int ImagePixRes;     ///ͼ��ֱ���(64*64)
	ParamFloat2D TargetRealSize;    //Ŀ����ʵ��С
	ParamInt2D TargetImageSize;     //��������Ŀ������С
	float Begin_P ;   ///��ʼĿ�����ص���ֵĸ���;
	float End_p ;   //����Ŀ�����ص���ֵĸ���;
	float Begin_Dist;     ///Ŀ���ʼ��Ծ���
	float End_Dist;   //Ŀ�������Ծ���
	float Target_Velocity;   //Ŀ���˶��ٶ�
	int APD_FrameRato;    //APD����֡�ʣ�
	Mat srcImage;
	Mat resImage;

	TargetBox TargetBefBox;    //��ʼ��ȡ(ǰһ֡)��Ŀ���
	TargetBox TargetTrackBox;      //���ٺ��Ŀ���
	ParamFloat2D centerPos;    //Ŀ����������

	TargetBox RectangularBox;  //�����ο�
	ParamFloat2D RectangularCenterPos;    //���ο���������
	
	float padding;    //�����Χ��
	int THRE;

	bool FirstFrame;  //������֡��־
	int boxLeftTopX;
	int boxLeftTopY;
	int boxWidth;
	int boxHeight;
};


#endif