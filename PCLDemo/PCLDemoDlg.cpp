// PCLDemoDlg.cpp : 实现文件
//
#include "stdafx.h"
#include "PCLDemo.h"
#include "PCLDemoDlg.h"
#include "afxdialogex.h"

#define MAXPIXELVAL 16383
#define IMAGE_H 256
#define IMAGE_W 256
#define FrameEnd 0xAA

#define FPSRatio 2  //显示的帧率是实际帧率的倍数
#define TrueTargetDist 2350   ///实际中车距离APD最大真实距离

//using namespace pcl;
using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
//typedef pcl::PointCloud<PointXYZRGB> PointCloudRGB;

vtkSmartPointer<vtkRenderWindow> m_win;
vtkSmartPointer<vtkRenderWindowInteractor> m_iren;


PointCloudT::Ptr cloud(new PointCloudT);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer", false));
float HorDistRes=0.01;    //深度图横向坐标扩大
float VerDistRes = 0.01;  ///深度图纵向坐标扩大
float DistZ_Res = 0.008; ///深度图深度值扩大倍数 

//boost::mutex updateModelMutex;
bool Online = true;
bool Dist_Syn = TRUE;  //  加目标点函数所用的距离值是否与距离门限（触发延时）设置的值同步
static bool AutoSetTrigDelayFlag = false;
int AutoSpinTime = 500;
float DistShowRatio = 1100.0 / (TrueTargetDist - 2000);

DWORD imshow1DThread(LPDWORD lpdwParam);


//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPCLDemoDlg 对话框



CPCLDemoDlg::CPCLDemoDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_PCLDEMO_DIALOG, pParent)
	, startTracking(false)
	, showBoundingBox(false)
	, THRE(100)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPCLDemoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CPCLDemoDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_MESSAGE(WM_SHOW_CLOUD, &CPCLDemoDlg::OnShowCloudMsg)
	ON_BN_CLICKED(IDCANCEL, &CPCLDemoDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON1, &CPCLDemoDlg::OnUDPPortOpen)
	ON_BN_CLICKED(IDC_BUTTON3, &CPCLDemoDlg::OnUDPPortClose)
	ON_BN_CLICKED(IDC_BUTTON5, &CPCLDemoDlg::OnStartReceive)
	ON_BN_CLICKED(IDC_CHECK1, &CPCLDemoDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BUTTON6, &CPCLDemoDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON4, &CPCLDemoDlg::OnStopReceive)
	ON_BN_CLICKED(IDC_BUTTON2, &CPCLDemoDlg::OnStartTracking)
	ON_BN_CLICKED(IDC_BUTTON7, &CPCLDemoDlg::OnStopTracking)
	ON_BN_CLICKED(IDC_CHECK2, &CPCLDemoDlg::OnBnClickedCheck2)
	ON_BN_CLICKED(IDC_BUTTON8, &CPCLDemoDlg::OnStartSave)
	ON_BN_CLICKED(IDC_BUTTON9, &CPCLDemoDlg::OnStopSave)
END_MESSAGE_MAP()


// CPCLDemoDlg 消息处理程序

BOOL CPCLDemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	//==========================将viewer窗口结合到MFC图像控件上===========================
	CRect rect;
	m_iren = vtkRenderWindowInteractor::New();//重新申请地址
	m_win = viewer->getRenderWindow();//获取viewer的句柄
	GetDlgItem(IDC_FRAME)->GetClientRect(&rect);//获取图像控件的实际大小
	m_win->SetSize(rect.right - rect.left, rect.bottom - rect.top);//将viewer大小设置为图像控件大小
	m_win->SetParentId(GetDlgItem(IDC_FRAME)->m_hWnd);//将vtk窗口结合到MFC窗口中
	viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作
	m_iren->SetRenderWindow(m_win);
	viewer->createInteractor();//由于初始化设置为false，该处重新创建PCL风格的Interactor
	m_win->Render();
	viewer->addCoordinateSystem();
	//viewer->setBackgroundColor(255, 255, 255);
	viewer->setBackgroundColor(0, 0, 0);
	//========================================================================================

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CPCLDemoDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CPCLDemoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CPCLDemoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

afx_msg LRESULT CPCLDemoDlg::OnShowCloudMsg(WPARAM wParam, LPARAM lParam)
{
	CPCLDemoDlg *pView = (CPCLDemoDlg*)lpParamTmp;
	//viewer->removeAllPointClouds();
	viewer->removePointCloud("Apd3DData");
	viewer->addPointCloud(cloud, "Apd3DData");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Apd3DData");
	
	if (pView->startTracking)
	{
		float min_x, min_y, min_z, max_x, max_y, max_z;
		min_x = (63-pView->track.targetCenter.x - 7 - 32)*HorDistRes;
		max_x = (63-pView->track.targetCenter.x + 7 - 32)*HorDistRes;
		min_y = (pView->track.targetCenter.y - 7 - 32)*VerDistRes;
		max_y = (pView->track.targetCenter.y + 7 - 32)*VerDistRes;
		min_z = (pView->track.targetDepth - 10 - 256)*DistZ_Res ;
		max_z = (pView->track.targetDepth + 10 - 256)*DistZ_Res;
		viewer->removeAllShapes();
		if (pView->showBoundingBox) {
			viewer->removeAllShapes();
			int a = 6;
			viewer->addLine(pcl::PointXYZ(min_x, min_y, min_z), pcl::PointXYZ(min_x, max_y, min_z), 255, 255, 255, "line1");
			viewer->addLine(pcl::PointXYZ(min_x, min_y, min_z), pcl::PointXYZ(max_x, min_y, min_z), 255, 255, 255, "line2");
			viewer->addLine(pcl::PointXYZ(max_x, max_y, min_z), pcl::PointXYZ(min_x, max_y, min_z), 255, 255, 255, "line3");
			viewer->addLine(pcl::PointXYZ(max_x, max_y, min_z), pcl::PointXYZ(max_x, min_y, min_z), 255, 255, 255, "line4");

			viewer->addLine(pcl::PointXYZ(min_x, min_y, max_z), pcl::PointXYZ(min_x, max_y, max_z), 255, 255, 255, "line5");
			viewer->addLine(pcl::PointXYZ(min_x, min_y, max_z), pcl::PointXYZ(max_x, min_y, max_z), 255, 255, 255, "line6");
			viewer->addLine(pcl::PointXYZ(max_x, max_y, max_z), pcl::PointXYZ(min_x, max_y, max_z), 255, 255, 255, "line7");
			viewer->addLine(pcl::PointXYZ(max_x, max_y, max_z), pcl::PointXYZ(max_x, min_y, max_z), 255, 255, 255, "line8");

			viewer->addLine(pcl::PointXYZ(min_x, min_y, max_z), pcl::PointXYZ(min_x, min_y, min_z), 255, 255, 255, "line9");
			viewer->addLine(pcl::PointXYZ(max_x, min_y, max_z), pcl::PointXYZ(max_x, min_y, min_z), 255, 255, 255, "line10");
			viewer->addLine(pcl::PointXYZ(min_x, max_y, max_z), pcl::PointXYZ(min_x, max_y, min_z), 255, 255, 255, "line11");
			viewer->addLine(pcl::PointXYZ(max_x, max_y, max_z), pcl::PointXYZ(max_x, max_y, min_z), 255, 255, 255, "line12");

			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line1");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line2");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line3");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line4");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line5");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line6");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line7");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line8");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line9");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line10");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line11");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, a, "line12");
		}
		
		

		//viewer->addCube(min_x, max_x, min_y, max_y, min_z, max_z, 255, 255, 255, "cube");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cube");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "line");
	}

	CRect rect;
	GetClientRect(&rect);//实时获取MFC窗口大小
	m_win = viewer->getRenderWindow();
	m_win->SetSize(rect.right - rect.left, rect.bottom - rect.top);
	m_win->SetParentId(this->m_hWnd);//将vtk窗口结合到MFC窗口中
	m_iren->SetRenderWindow(m_win);
	m_win->Render();
	return 0;
}
void CPCLDemoDlg::DrawMat(cv::Mat & img, UINT nControlID, bool bRoomToControlSize)
{
	cv::Mat imgTmp;
	CRect rect;
	GetDlgItem(nControlID)->GetClientRect(&rect);  // 获取控件大小
	if (bRoomToControlSize)
	{
		cv::resize(img, imgTmp, cv::Size(rect.Width(), rect.Height()));// 缩小或放大Mat并备份
	}
	else
	{
		img.copyTo(imgTmp);
	}

	switch (imgTmp.channels())
	{
	case 1:
		cv::cvtColor(imgTmp, imgTmp, CV_GRAY2BGRA); // GRAY单通道
		break;
	case 3:
		cv::cvtColor(imgTmp, imgTmp, CV_BGR2BGRA);  // BGR三通道
		break;
	default:
		break;
	}

	int pixelBytes = imgTmp.channels()*(imgTmp.depth() + 1); // 计算一个像素多少个字节
															 // 制作bitmapinfo(数据头)
	BITMAPINFO bitInfo;
	bitInfo.bmiHeader.biBitCount = 8 * pixelBytes;
	bitInfo.bmiHeader.biWidth = imgTmp.cols;
	bitInfo.bmiHeader.biHeight = -imgTmp.rows;
	bitInfo.bmiHeader.biPlanes = 1;
	bitInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bitInfo.bmiHeader.biCompression = BI_RGB;
	bitInfo.bmiHeader.biClrImportant = 0;
	bitInfo.bmiHeader.biClrUsed = 0;
	bitInfo.bmiHeader.biSizeImage = 0;
	bitInfo.bmiHeader.biXPelsPerMeter = 0;
	bitInfo.bmiHeader.biYPelsPerMeter = 0;
	// Mat.data + bitmap数据头 -> MFC
	CDC *pDC = GetDlgItem(nControlID)->GetDC();
	pDC->SetStretchBltMode(COLORONCOLOR);
	if (bRoomToControlSize)
	{
		::StretchDIBits(pDC->GetSafeHdc()
			, 0, 0, rect.Width(), rect.Height()
			, 0, 0, imgTmp.cols, imgTmp.rows,
			imgTmp.data, &bitInfo, DIB_RGB_COLORS, SRCCOPY);
	}
	else
	{
		int minWidth = min(imgTmp.cols, rect.Width());
		int minHeight = min(imgTmp.rows, rect.Height());
		::StretchDIBits(
			pDC->GetSafeHdc(),
			0, 0, minWidth, minHeight,
			0, 0, minWidth, minHeight,
			imgTmp.data,
			&bitInfo,
			DIB_RGB_COLORS,
			SRCCOPY
		);
	}
	ReleaseDC(pDC);
}
void CPCLDemoDlg::Mat16to8(cv::Mat &src, cv::Mat &dst) {
	for (int Y = 0; Y < 64; Y++) {
		for (int X = 0; X < 64; X++) {

			dst.at<uchar>(X, Y) = (uchar)(src.at<ushort>(X, Y) / 2);
		}
	}
}

void Depth2PointCloud(Mat src,Mat RgbImg)
{
	cloud->clear();
	for (int i = 0; i < 63; i++)
	{
		for (int j = 0; j < 63; j++)
		{
			PointT Point3D;
			Point3D.x = (j-32) * HorDistRes;
			Point3D.y = (i-32) * VerDistRes;
			Point3D.z = (src.at<ushort>(j, i) - 256)*DistZ_Res;
			Point3D.b = RgbImg.at<Vec3b>(j, i)[0];
			Point3D.g = RgbImg.at<Vec3b>(j, i)[1];
			Point3D.r = RgbImg.at<Vec3b>(j, i)[2];
			cloud->push_back(Point3D);
		}
	}
}


DWORD WINAPI CPCLDemoDlg::ImageProcessThread(LPVOID lpParam) {
	CPCLDemoDlg *pView = (CPCLDemoDlg*)lpParam;
	pView->lpParamTmp = lpParam;
	CString str;
	if (nullptr == pView)
	{
		AfxMessageBox(_T("获取指针失败!"));
		return 0;
	}
	clock_t startTime, endTime;
	srand((int)time(0));  //随机数初始化
	int frameNum = 0;
	while (pView->isThreadbegin) {
		startTime = clock();
		if (Online)
			pView->server.receiveData(pView->depth_img, pView->force_img);		//在线
		else
			pView->server.readRAWData(pView->depth_img, pView->force_img);	//离线
		Mat imgShowDepth;
		Mat TempDepth = pView->depth_img.clone();
		Mat resForce = pView->force_img.clone();
		Mat TempDepth_trans;
		TempDepth_trans = pView->depth_img.clone();
		cv::transpose(TempDepth_trans, TempDepth_trans);
		flip(TempDepth_trans, TempDepth_trans, 0);

		if (pView->startTracking) 
		{
			pView->track.THRE = pView->THRE;
			Mat TempDepth = pView->depth_img.clone();
			Mat resForce = pView->force_img.clone();



			pView->track.detactByIntense(TempDepth, resForce);
			pView->ImgTargetDepth = pView->track.targetDepth;
			//pView->SetDlgItemInt(IDC_EDIT24, pView->ImgTargetDepth);

			str.Format(_T("%.2f"), (pView->ImgTargetDepth*0.15 + pView->ShowDist - 189 * 0.15));
			pView->SetDlgItemText(IDC_EDIT8, str);
			str.Format(_T("%d"), (int)pView->track.targetCenter.x);
			pView->SetDlgItemText(IDC_EDIT9, str);
			str.Format(_T("%d"), (int)pView->track.targetCenter.y);
			pView->SetDlgItemText(IDC_EDIT10, str);
			//强度图归一化彩色显示
			double vmin, vmax, alpha;
			cv::minMaxLoc(resForce, &vmin, &vmax);
			alpha = (255.0 / (vmax - vmin))*0.85;
			resForce.convertTo(resForce, CV_8U, alpha, -vmin * alpha);
			cv::Mat imgShowForce;
			applyColorMap(resForce, imgShowForce, cv::COLORMAP_JET);
			pView->SetDlgItemInt(IDC_EDIT1, (int)vmin);
			pView->SetDlgItemInt(IDC_EDIT2, (int)vmax);

			///////////////////
			//深度图量化彩色
			Mat resDepth(64, 64, CV_8UC1, Scalar(0));// = track.resImage.clone();
			pView->Mat16to8(TempDepth, resDepth);

			applyColorMap(resDepth, imgShowDepth, COLORMAP_JET);



			//显示图像逆时针旋转90度
			cv::transpose(imgShowDepth, imgShowDepth);
			flip(imgShowDepth, imgShowDepth, 0);
			cv::transpose(imgShowForce, imgShowForce);
			flip(imgShowForce, imgShowForce, 0);

			pView->DrawMat(imgShowDepth, IDC_Pic_STATIC, true);
			pView->DrawMat(imgShowForce, IDC_force_STATIC, true);

			

			Depth2PointCloud(TempDepth_trans, imgShowDepth);
			::SendMessage(pView->m_hWnd, WM_SHOW_CLOUD, 0, 0);

		}
		else {
			//强度图归一化彩色显示
			double vmin, vmax, alpha;
			cv::minMaxLoc(resForce, &vmin, &vmax);
			alpha = (255.0 / (vmax - vmin))*0.85;
			resForce.convertTo(resForce, CV_8U, alpha, -vmin * alpha);
			cv::Mat imgShowForce;
			applyColorMap(resForce, imgShowForce, cv::COLORMAP_JET);
			pView->SetDlgItemInt(IDC_EDIT1, (int)vmin);
			pView->SetDlgItemInt(IDC_EDIT2, (int)vmax);

			//深度图量化彩色
			Mat resDepth(64, 64, CV_8UC1, Scalar(0));// = track.resImage.clone();
			pView->Mat16to8(TempDepth, resDepth);

			applyColorMap(resDepth, imgShowDepth, COLORMAP_JET);

			//显示图像逆时针旋转90度
			cv::transpose(imgShowDepth, imgShowDepth);
			flip(imgShowDepth, imgShowDepth, 0);
			cv::transpose(imgShowForce, imgShowForce);
			flip(imgShowForce, imgShowForce, 0);

			pView->DrawMat(imgShowDepth, IDC_Pic_STATIC, true);
			pView->DrawMat(imgShowForce, IDC_force_STATIC, true);

			Depth2PointCloud(TempDepth_trans, imgShowDepth);

			::SendMessage(pView->m_hWnd, WM_SHOW_CLOUD, 0, 0);

		}
		

		endTime = clock();
		double process_time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
		double FPS = 1 / process_time;
		str.Format(_T("%d"), frameNum);
		pView->SetDlgItemText(IDC_EDIT5, str);

		str.Format(_T("%.4fs"), process_time / FPSRatio);  //显示的处理时间除以FPSRatio
		pView->SetDlgItemText(IDC_EDIT4, str);
		str.Format(_T("%.2fFPS"), FPS*FPSRatio);  //显示的显示的帧率乘以FPSRatio
		pView->SetDlgItemText(IDC_EDIT6, str);
		frameNum++;

	}
	return 0;
}




void CPCLDemoDlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	/*m_iren->Delete();
	m_win->Finalize();
	m_win->Delete();
	viewer->removeAllPointClouds();*/
	CDialogEx::OnCancel();
}



void CPCLDemoDlg::OnUDPPortOpen()
{
	// TODO: 在此添加控件通知处理程序代码
	if (!isPortOpen) {
		isPortOpen = true;
		bool port = server.startServer();
		if (port) {
			SetDlgItemText(IDC_EDIT3, _T("网口开启"));
		}
		else {
			SetDlgItemText(IDC_EDIT3, _T("网口开启错误"));
		}
	}
}


void CPCLDemoDlg::OnUDPPortClose()
{
	// TODO: 在此添加控件通知处理程序代码
	if (isPortOpen) {
		//在关闭网口之前先结束线程
		GetDataStartMutex = false;
		isThreadbegin = false;
		WaitForSingleObject(h1, 1000);
		GetDlgItem(IDC_Pic_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_force_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic1)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic2)->ShowWindow(FALSE);

		isPortOpen = false;
		server.closeServer();
		SetDlgItemText(IDC_EDIT3, _T("网口关闭"));
	}


}


void CPCLDemoDlg::OnStartReceive()
{
	// TODO: 在此添加控件通知处理程序代码
	if (!GetDataStartMutex && (isPortOpen || !Online))
	{
		GetDataStartMutex = true;  //加锁
		isThreadbegin = true;
		GetDlgItem(IDC_Pic_STATIC)->ShowWindow(TRUE);
		GetDlgItem(IDC_force_STATIC)->ShowWindow(TRUE);
		GetDlgItem(IDC_ColorMapPic1)->ShowWindow(TRUE);
		GetDlgItem(IDC_ColorMapPic2)->ShowWindow(TRUE);
		Mat ColorMapImage = imread("colorscale_jet.jpg");
		DrawMat(ColorMapImage, IDC_ColorMapPic1, true);
		DrawMat(ColorMapImage, IDC_ColorMapPic2, true);

		// TODO: 在此添加控件通知处理程序代码
		DWORD dw1;     //创建新的线程，与UI主线程分开，免得一直在运行处理函数，而UI程序一直没有退出，而卡死。
		h1 = CreateThread(NULL, 0, ImageProcessThread, this, 0, &dw1);
		CloseHandle(h1);
	}

}
void CPCLDemoDlg::OnStopReceive()
{
	// TODO: 在此添加控件通知处理程序代码
	if (GetDataStartMutex == true) {
		GetDataStartMutex = false;
		// TODO: 在此添加控件通知处理程序代码
		isThreadbegin = false;
		WaitForSingleObject(h1, 1000);
		GetDlgItem(IDC_Pic_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_force_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic1)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic2)->ShowWindow(FALSE);
	}
}

void CPCLDemoDlg::OnBnClickedCheck1()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton *pCheckbox = (CButton*)GetDlgItem(IDC_CHECK1);
	if (pCheckbox->GetCheck())
	{
		Online = false;
	}
	else
	{
		Online = true;
	}

}


void CPCLDemoDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	 //?设置过滤器???
	TCHAR szFilter[] = _T("文本文件(*.raw)|*.raw|所有文件(*.*)|*.*||");
	//?构造打开文件对话框???
	CFileDialog fileDlg(TRUE,_T("raw"), NULL, 0, szFilter, this);
	CString strFilePath;

	//?显示打开文件对话框???
	if (IDOK == fileDlg.DoModal())
	{
		//?如果点击了文件对话框上的“打开”按钮，则将选择的文件路径显示到编辑框里???
		strFilePath = fileDlg.GetPathName();
		SetDlgItemText(IDC_EDIT7, strFilePath);
	}
	server.image_file = CT2A(strFilePath.GetString());

}

void CPCLDemoDlg::OnStartTracking()
{
	// TODO: 在此添加控件通知处理程序代码
	startTracking = true;
	//track.InitParam();

}


void CPCLDemoDlg::OnStopTracking()
{
	// TODO: 在此添加控件通知处理程序代码
	startTracking = false;
	track.FirstFrame = true;  ///跟踪的首帧重新置位true
}

void CPCLDemoDlg::OnBnClickedCheck2()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton *pCheckbox = (CButton*)GetDlgItem(IDC_CHECK2);
	if (pCheckbox->GetCheck())
	{
		showBoundingBox = true;
	}
	else
	{
		showBoundingBox = false;
	}
}

void CPCLDemoDlg::OnStartSave()
{
	// TODO: 在此添加控件通知处理程序代码
	server.can_save = true;
	server.is_new_file = true;
}

void CPCLDemoDlg::OnStopSave()
{
	// TODO: 在此添加控件通知处理程序代码
	server.can_save = false;
	server.out_RAW.close();
}
