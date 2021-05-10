
// PCLDemoDlg.h : 头文件
//
#include"udpServer.h"
#include"Track_Main.h"
#include<ctime>

#include <opencv2/opencv.hpp> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> 
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>  

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#pragma once



// CPCLDemoDlg 对话框
class CPCLDemoDlg : public CDialogEx
{
// 构造
public:
	CPCLDemoDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PCLDEMO_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg LRESULT OnShowCloudMsg(WPARAM wParam, LPARAM lParam);
	DECLARE_MESSAGE_MAP()

public:
	UDPServer server;
	myTrack track;
	cv::Mat depth_img;
	cv::Mat force_img;
	CString LeftTopX;
	CString LeftTopY;
	CString boxWidth;
	CString boxHeight;
	CButton BestTrack;
	CString beginDistInput;
	bool add_Target;
	bool startTracking;		//是否开始跟踪
	bool isThreadbegin;
	HANDLE h1;				//线程句柄
	bool isPortOpen;		//网口开启标志位
	bool getPointPosInImg;  //是否得到取点坐标位置
	CPoint showPoint;
	static DWORD WINAPI ImageProcessThread(LPVOID lpParam);
	void Mat16to8(cv::Mat &src, cv::Mat &dst);
	int Intensity_Min;
	int Intensity_Max;
	// 选择跟踪模式
	CComboBox Track_Mode;
	UINT TrackModeFlag;
	int THRE;
	int ImgTargetDepth;
	bool GetDataStartMutex;
	bool AutoSetDelayByDepth; //是否根据深度值变化改变延时
	bool getPreDepth;			//打钩时设置preDepth
	UINT ShowDist;
	LPVOID lpParamTmp;
	bool showBoundingBox;
//	void OnChangeTrigDelay();
	//afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
//	void showPointDistance(Mat &depthImg);	//显示图像取点处的距离值
//	void showPointDistance5T5(Mat &depthImg);


	void DrawMat(cv::Mat & img, UINT nControlID, bool bRoomToControlSize);

	//static DWORD WINAPI ImageProcessThread(LPVOID lpParam);
	afx_msg void OnBnClickedCancel();
	afx_msg void OnUDPPortOpen();
	afx_msg void OnUDPPortClose();
	afx_msg void OnStartReceive();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnStopReceive();
	afx_msg void OnStartTracking();
	afx_msg void OnStopTracking();
	afx_msg void OnBnClickedCheck2();
	afx_msg void OnStartSave();
	afx_msg void OnStopSave();
};
