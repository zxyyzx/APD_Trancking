#include"Track_Main.h"
//#include"SerialTestDlg.h"


myTrack::myTrack()
{
	Fov = 0.9;             //�״��ӳ���
	ImagePixRes = 64;     ///ͼ��ֱ���(64*64)
	TargetRealSize.x = 3; //Ŀ����ʵ��С
	TargetRealSize.y = 2.5;
	Begin_P = 0.4;   ///��ʼĿ�����ص���ֵĸ���;
	End_p = 0.8;   //����Ŀ�����ص���ֵĸ���;
	Begin_Dist = 3000.0;     ///Ŀ���ʼ��Ծ���
	End_Dist = 400.0;   //Ŀ�������Ծ���
	Target_Velocity = 8;   //Ŀ���˶��ٶ�
	APD_FrameRato = 20;    //APD����֡�ʣ�
	srcImage = Mat(64, 64, CV_16UC1, Scalar(512));
	resImage = Mat(64, 64, CV_16UC1, Scalar(512));
	padding = 3.0;    //�����Χ��

	TargetBefBox.Left_Top_Pos_x = 29;
	TargetBefBox.Left_Top_Pos_y = 30;
	TargetBefBox.width = 6;
	TargetBefBox.height = 4;

	//����Ŀ������
	centerPos.x = TargetBefBox.Left_Top_Pos_x + (TargetBefBox.width - 1) / 2.0;  //ǰһ֡���ĵ��������
	centerPos.y = TargetBefBox.Left_Top_Pos_y + (TargetBefBox.height - 1) / 2.0;

	FirstFrame = true;
	boxLeftTopX=20;
	boxLeftTopY=20;
	boxWidth=14;
	boxHeight=14;
}
void myTrack::InitParam()
{
	Fov = 0.9;             //�״��ӳ���
	ImagePixRes = 64;     ///ͼ��ֱ���(64*64)
	TargetRealSize.x = 3; //Ŀ����ʵ��С
	TargetRealSize.y = 2.5;
	Begin_P = 0.3;   ///��ʼĿ�����ص���ֵĸ���;
	End_p = 0.8;   //����Ŀ�����ص���ֵĸ���;
	Begin_Dist = 3000.0;     ///Ŀ���ʼ��Ծ���
	End_Dist = 500.0;   //Ŀ�������Ծ���
	Target_Velocity = 20;   //Ŀ���˶��ٶ�
	APD_FrameRato = 20;    //APD����֡�ʣ�
	padding = 1.0;    //�����Χ��

	TargetBefBox.Left_Top_Pos_x = 29;
	TargetBefBox.Left_Top_Pos_y = 30;
	TargetBefBox.width = 6;
	TargetBefBox.height = 4;
}
ParamInt2D myTrack::GetDepthImageTargetPix(float myDist,ParamFloat2D TargetSize)
{
	float viewRealSize;
	viewRealSize = myDist * tan(Fov* PI/ 360.0)*2;
	float viewRealRes = viewRealSize / ImagePixRes;
	ParamInt2D TargetImagePixSize;
	TargetImagePixSize.x =(int)(TargetSize.x / viewRealRes);
	TargetImagePixSize.y= (int)(TargetSize.y / viewRealRes);
	return TargetImagePixSize;
}
int myTrack::GetDistTimeFromDepth(float myDist,int offset)
{
	int DistTime = (int)(myDist / 0.15)- offset;
	return DistTime;
}


float myTrack::GetProbability(float myDist,float offset,int sigma)
{
	float p = Begin_P + (Begin_Dist - myDist) / (Begin_Dist - End_Dist+4)*(End_p - Begin_P); //����Խ��Ŀ����ֵĸ���ԽС�������仯�Ƚ�С
	if (sigma >= 2) {

		p = p * exp(-pow((offset / sigma), 2));
	}
	return p;
}

Mat myTrack::Track(float myDist,Mat Src, TargetBox &TargetBeginBox)
{
	//srand((int)time(0));  //�������ʼ��

	//����Ŀ������
	centerPos.x = TargetBefBox.Left_Top_Pos_x + (TargetBefBox.width - 1) / 2.0;  //ǰһ֡���ĵ��������
	centerPos.y = TargetBefBox.Left_Top_Pos_y + (TargetBefBox.height - 1) / 2.0;
	//����Ŀ�����ش�С
	TargetImageSize = GetDepthImageTargetPix(myDist, TargetRealSize);
	///Ŀ������
	TargetTrackBox.width = TargetImageSize.x;  //�����С����
	TargetTrackBox.height = TargetImageSize.y;
	TargetTrackBox.Left_Top_Pos_x = centerPos.x - (TargetTrackBox.width - 1) / 2;
	TargetTrackBox.Left_Top_Pos_y = centerPos.y - (TargetTrackBox.height - 1) / 2;
//	int Dist_Time = GetDistTimeFromDepth(myDist, 0);
	int Dist_Time =100;
	for (int i = TargetTrackBox.Left_Top_Pos_x; i < TargetTrackBox.Left_Top_Pos_x + TargetTrackBox.width; i++) {
		int temp_i = i + rand() % 3 - 1;
		if (i > 63)temp_i = 63;
		else if (i < 0)temp_i = 0;
		for (int j = TargetTrackBox.Left_Top_Pos_y; j < TargetTrackBox.Left_Top_Pos_y + TargetTrackBox.height; j++)
		{
			int temp_j = j;
			if (j > 63)temp_j = 63;
			else if (j < 0)temp_j = 0;
			float p = GetProbability(myDist, abs(temp_i - centerPos.x) + abs(temp_j - centerPos.y), min(TargetTrackBox.width, TargetTrackBox.height) / 2);
			if ((rand() % 100) / 100.0 < p)  //��P�����ڸ�λ�ó����趨�����أ���1-P��������Ϊԭ����ֵ
			{
				int offset = rand() % DistErr;   //ǰ��0.45�׾����������ֵ��
				int Dist_Real_Time = Dist_Time + offset - DistErr/2;   //�������ǰ�����ľ�������ֵ
				Src.at<ushort>(temp_j, temp_i) = Dist_Real_Time;
			}
		}
	}
	TargetBefBox = TargetTrackBox;  //����ǰһ֡Ŀ������
	return Src;
}
void myTrack::cvShowImage(Mat res, Rect rect)
{
    cv::rectangle(res, rect, cv::Scalar::all(0xffff), 1);
	const string winname = "Result Image";
	cv::namedWindow(winname, CV_WINDOW_NORMAL);   //CV_WINDOW_NORMAL
	cv::resizeWindow(winname, 640, 640);
	cv::imshow(winname, res*5);
	cv::waitKey(1);
}
void myTrack::AddRectangularBox()
{
	int boxLeftTopX = (int)(centerPos.x - (TargetTrackBox.width+padding) / 2)+rand()%3-1;
	int boxLeftTopY = (int)(centerPos.y - (TargetTrackBox.height+padding) / 2) + rand() % 3 - 1;
	int boxWidth = (int)(TargetTrackBox.width+padding) + 1;
	int boxHeight = (int)(TargetTrackBox.height+padding) + 1;

	RectangularCenterPos.x = boxLeftTopX + (boxWidth - 1) / 2.0;
	RectangularCenterPos.y = boxLeftTopY + (boxHeight - 1) / 2.0;

	int boxRightBottomX = (boxLeftTopX + boxWidth) > 63 ? 63 : (boxLeftTopX + boxWidth);
	int boxRightBottomY = (boxLeftTopY + boxHeight) > 63 ? 63 : (boxLeftTopY + boxHeight);
	boxLeftTopX = boxLeftTopX <= 0 ? 0 : boxLeftTopX;
	boxLeftTopY = boxLeftTopY <= 0 ? 0 : boxLeftTopY;

	cv::Rect rect(boxLeftTopX, boxLeftTopY, boxRightBottomX - boxLeftTopX + 1, boxRightBottomY - boxLeftTopY + 1);
	//	srcImage = resImage;   //����ǰһ֡ͼ��
	cv::rectangle(srcImage, rect, cv::Scalar::all(0xffff), 1);

	

}
///��������
void myTrack::TrackMain(Mat src)
{
	srcImage = src.clone();
	int frameNum = 0;
//	float Target_Dist= Begin_Dist;
	
//	while (Target_Dist > 500)   //�޶��������500��
	{
		resImage = srcImage.clone();
		resImage = Track(Begin_Dist, resImage, TargetBefBox);
		int boxLeftTopX = (int)(centerPos.x - TargetTrackBox.width*padding / 2);
		int boxLeftTopY = (int)(centerPos.y - TargetTrackBox.height*padding / 2);
		int boxWidth = (int)TargetTrackBox.width*padding + 1;
		int boxHeight = (int)TargetTrackBox.height*padding + 1;
		int boxRightBottomX = (boxLeftTopX + boxWidth) > 63 ? 63 : (boxLeftTopX + boxWidth);
		int boxRightBottomY= (boxLeftTopY + boxHeight) > 63 ? 63 : (boxLeftTopY + boxHeight);
		boxLeftTopX = boxLeftTopX <= 0 ? 0 : boxLeftTopX;
		boxLeftTopY = boxLeftTopY <= 0 ? 0 : boxLeftTopY;
		
		cv::Rect rect(boxLeftTopX, boxLeftTopY, boxRightBottomX- boxLeftTopX +1, boxRightBottomY - boxLeftTopY +1);
	//	srcImage = resImage;   //����ǰһ֡ͼ��
		cv::rectangle(resImage, rect, cv::Scalar::all(0xffff), 1);
		//cvShowImage(resImage, rect);

		cout << "֡����" << frameNum << endl;
		cout << "���룺" << Begin_Dist << endl;
		frameNum++;
//		Target_Dist = Target_Dist - Target_Velocity / APD_FrameRato;  //���¾���ֵ
	}

}

ushort myTrack::getCurrentDepth(Mat &depth, Point2i targetCenter) {
	vector<ushort> depth_list;
	vector<int> x_pos = { -2, -1, 0, 1, 2 };
	vector<int> y_pos = { -2, -1, 0, 1, 2 };
	for (int x = 0; x < x_pos.size(); x++) {
		for (int y = 0; y < y_pos.size(); y++) {
			int xx = targetCenter.x + x_pos[x];
			if (xx > 63 || xx < 0) continue;
			int yy = targetCenter.y + x_pos[y];
			if (yy > 63 || yy < 0) continue;
			depth_list.push_back(depth.at<ushort>(yy, xx));
		}
	}
	sort(depth_list.begin(), depth_list.end());
	return depth_list[(depth_list.size() + 1) / 2];
}

void myTrack::detactByIntense(Mat &depth, Mat &intense)
{
	int point_count = 0;
	int x_sum = 0, y_sum = 0;
	int targetWidth, targetHeight;
	vector<int> max_x_pos, max_y_pos;
	//��ǿ������쳣�㶼��0
	vector<int> x_pos = { 45, 44, 46, 45, 4 };
	vector<int> y_pos = { 13, 14, 14, 15, 34 };
	vector<ushort> v_depth, v_intense;
	for (int i = 0; i < x_pos.size(); i++) {
		intense.at<ushort>(y_pos[i], x_pos[i]) = 0;
	}

	//medianBlur(intense, intense, 3);
	Mat intense_trans = intense.clone();
	double vmin, vmax, alpha;
	minMaxLoc(intense_trans, &vmin, &vmax);
	alpha = (255.0 / (vmax - vmin))*0.85;
	intense_trans.convertTo(intense_trans, CV_8U, alpha, -vmin * alpha);

	minMaxLoc(intense_trans, &vmin, &vmax);
	for (int x = 0; x < 64; x++) {
		for (int y = 0; y < 64; y++) {
			if (intense_trans.at<uchar>(y, x) == (uchar)vmax) {
				max_x_pos.push_back(x);
				max_y_pos.push_back(y);
			}
		}
	}

	medianBlur(intense_trans, intense_trans, 3);
	for (int i = 0; i < max_x_pos.size(); i++) {
		intense_trans.at<uchar>(max_y_pos[i], max_x_pos[i]) = (uchar)vmax;
	}

	for (int x = 0; x < 64; x++) {
		for (int y = 0; y < 64; y++) {
			if (intense_trans.at<uchar>(y, x) > THRE) {
				x_sum += x;
				y_sum += y;
				point_count++;
			}
		}
	}
	targetCenter.x = x_sum / point_count;
	targetCenter.y = y_sum / point_count;
	//Ŀ����С������취��
	targetWidth = 14;
	targetHeight = 14;
	targetBox.x = (int)(targetCenter.x - targetWidth / 2);
	targetBox.y = (int)(targetCenter.y - targetHeight / 2);

	////////////////////���Ǳ�����
	boxWidth = targetWidth;
	boxHeight = targetHeight;
	if (FirstFrame)
	{
		boxLeftTopX = targetBox.x;
		boxLeftTopY = targetBox.y;
		FirstFrame = false;
	}
	
	else
	{
		////ÿ֡Ŀ���ֻ�仯һ������

	//if (abs(matching_leftTop_Pos.x - target_leftTop_pos.x) > abs(matching_leftTop_Pos.y - target_leftTop_pos.y))
		{
			if (targetBox.x > boxLeftTopX)
			{
				boxLeftTopX++;
			}
			else if (targetBox.x < boxLeftTopX)
			{
				boxLeftTopX--;
			}
		}
		//	else
		{
			if (targetBox.y > boxLeftTopY)
			{
				boxLeftTopY++;
			}
			else if (targetBox.y < boxLeftTopY)
			{
				boxLeftTopY--;
			}
		}
	}

	//RectangularCenterPos.x = boxLeftTopX + (boxWidth - 1) / 2.0;
	//RectangularCenterPos.y = boxLeftTopY + (boxHeight - 1) / 2.0;

	int boxRightBottomX = (boxLeftTopX + boxWidth) > 63 ? 63 : (boxLeftTopX + boxWidth);
	int boxRightBottomY = (boxLeftTopY + boxHeight) > 63 ? 63 : (boxLeftTopY + boxHeight);
	boxLeftTopX = boxLeftTopX <= 0 ? 0 : boxLeftTopX;
	boxLeftTopY = boxLeftTopY <= 0 ? 0 : boxLeftTopY;

	cv::Rect rect(boxLeftTopX, boxLeftTopY, boxRightBottomX - boxLeftTopX + 1, boxRightBottomY - boxLeftTopY + 1);

	cv::minMaxLoc(intense, &vmin, &vmax);
	cv::rectangle(intense, rect, cv::Scalar::all(int(vmax)), 1);
	cv::rectangle(depth, rect, cv::Scalar::all(int(65480)), 1);
	targetDepth = getCurrentDepth(depth, targetCenter);
	return;
}

void myTrack::showImageColor(cv::Mat mat, string winname)
{
	double vmin, vmax, alpha;
	cv::minMaxLoc(mat, &vmin, &vmax);
	alpha = (255.0 / (vmax - vmin))*0.85;
	mat.convertTo(mat, CV_8U, alpha, -vmin * alpha);
	cv::Mat im_color;
	applyColorMap(mat, im_color, cv::COLORMAP_JET);
	cv::namedWindow(winname.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(winname.c_str(), 640, 640);
	cv::imshow(winname.c_str(), im_color);
	cv::waitKey(1);
}

//
//int main()
//{
//	Mat src = Mat(64, 64, CV_16UC1, Scalar(512));
//	myTrack track;
//	track.InitParam(src);
//	track.TrackMain();
//	return 0;
//}