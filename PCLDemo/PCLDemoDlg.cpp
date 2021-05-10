// PCLDemoDlg.cpp : ʵ���ļ�
//
#include "stdafx.h"
#include "PCLDemo.h"
#include "PCLDemoDlg.h"
#include "afxdialogex.h"

#define MAXPIXELVAL 16383
#define IMAGE_H 256
#define IMAGE_W 256
#define FrameEnd 0xAA

#define FPSRatio 2  //��ʾ��֡����ʵ��֡�ʵı���
#define TrueTargetDist 2350   ///ʵ���г�����APD�����ʵ����

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
float HorDistRes=0.01;    //���ͼ������������
float VerDistRes = 0.01;  ///���ͼ������������
float DistZ_Res = 0.008; ///���ͼ���ֵ������ 

//boost::mutex updateModelMutex;
bool Online = true;
bool Dist_Syn = TRUE;  //  ��Ŀ��㺯�����õľ���ֵ�Ƿ���������ޣ�������ʱ�����õ�ֵͬ��
static bool AutoSetTrigDelayFlag = false;
int AutoSpinTime = 500;
float DistShowRatio = 1100.0 / (TrueTargetDist - 2000);

DWORD imshow1DThread(LPDWORD lpdwParam);


//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// CPCLDemoDlg �Ի���



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


// CPCLDemoDlg ��Ϣ�������

BOOL CPCLDemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ  ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������
	//==========================��viewer���ڽ�ϵ�MFCͼ��ؼ���===========================
	CRect rect;
	m_iren = vtkRenderWindowInteractor::New();//���������ַ
	m_win = viewer->getRenderWindow();//��ȡviewer�ľ��
	GetDlgItem(IDC_FRAME)->GetClientRect(&rect);//��ȡͼ��ؼ���ʵ�ʴ�С
	m_win->SetSize(rect.right - rect.left, rect.bottom - rect.top);//��viewer��С����Ϊͼ��ؼ���С
	m_win->SetParentId(GetDlgItem(IDC_FRAME)->m_hWnd);//��vtk���ڽ�ϵ�MFC������
	viewer->resetCamera();//ʹ������ʾ����Ļ�м䣬�������Ĳ���
	m_iren->SetRenderWindow(m_win);
	viewer->createInteractor();//���ڳ�ʼ������Ϊfalse���ô����´���PCL����Interactor
	m_win->Render();
	viewer->addCoordinateSystem();
	//viewer->setBackgroundColor(255, 255, 255);
	viewer->setBackgroundColor(0, 0, 0);
	//========================================================================================

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ  ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CPCLDemoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
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
	GetClientRect(&rect);//ʵʱ��ȡMFC���ڴ�С
	m_win = viewer->getRenderWindow();
	m_win->SetSize(rect.right - rect.left, rect.bottom - rect.top);
	m_win->SetParentId(this->m_hWnd);//��vtk���ڽ�ϵ�MFC������
	m_iren->SetRenderWindow(m_win);
	m_win->Render();
	return 0;
}
void CPCLDemoDlg::DrawMat(cv::Mat & img, UINT nControlID, bool bRoomToControlSize)
{
	cv::Mat imgTmp;
	CRect rect;
	GetDlgItem(nControlID)->GetClientRect(&rect);  // ��ȡ�ؼ���С
	if (bRoomToControlSize)
	{
		cv::resize(img, imgTmp, cv::Size(rect.Width(), rect.Height()));// ��С��Ŵ�Mat������
	}
	else
	{
		img.copyTo(imgTmp);
	}

	switch (imgTmp.channels())
	{
	case 1:
		cv::cvtColor(imgTmp, imgTmp, CV_GRAY2BGRA); // GRAY��ͨ��
		break;
	case 3:
		cv::cvtColor(imgTmp, imgTmp, CV_BGR2BGRA);  // BGR��ͨ��
		break;
	default:
		break;
	}

	int pixelBytes = imgTmp.channels()*(imgTmp.depth() + 1); // ����һ�����ض��ٸ��ֽ�
															 // ����bitmapinfo(����ͷ)
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
	// Mat.data + bitmap����ͷ -> MFC
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
		AfxMessageBox(_T("��ȡָ��ʧ��!"));
		return 0;
	}
	clock_t startTime, endTime;
	srand((int)time(0));  //�������ʼ��
	int frameNum = 0;
	while (pView->isThreadbegin) {
		startTime = clock();
		if (Online)
			pView->server.receiveData(pView->depth_img, pView->force_img);		//����
		else
			pView->server.readRAWData(pView->depth_img, pView->force_img);	//����
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
			//ǿ��ͼ��һ����ɫ��ʾ
			double vmin, vmax, alpha;
			cv::minMaxLoc(resForce, &vmin, &vmax);
			alpha = (255.0 / (vmax - vmin))*0.85;
			resForce.convertTo(resForce, CV_8U, alpha, -vmin * alpha);
			cv::Mat imgShowForce;
			applyColorMap(resForce, imgShowForce, cv::COLORMAP_JET);
			pView->SetDlgItemInt(IDC_EDIT1, (int)vmin);
			pView->SetDlgItemInt(IDC_EDIT2, (int)vmax);

			///////////////////
			//���ͼ������ɫ
			Mat resDepth(64, 64, CV_8UC1, Scalar(0));// = track.resImage.clone();
			pView->Mat16to8(TempDepth, resDepth);

			applyColorMap(resDepth, imgShowDepth, COLORMAP_JET);



			//��ʾͼ����ʱ����ת90��
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
			//ǿ��ͼ��һ����ɫ��ʾ
			double vmin, vmax, alpha;
			cv::minMaxLoc(resForce, &vmin, &vmax);
			alpha = (255.0 / (vmax - vmin))*0.85;
			resForce.convertTo(resForce, CV_8U, alpha, -vmin * alpha);
			cv::Mat imgShowForce;
			applyColorMap(resForce, imgShowForce, cv::COLORMAP_JET);
			pView->SetDlgItemInt(IDC_EDIT1, (int)vmin);
			pView->SetDlgItemInt(IDC_EDIT2, (int)vmax);

			//���ͼ������ɫ
			Mat resDepth(64, 64, CV_8UC1, Scalar(0));// = track.resImage.clone();
			pView->Mat16to8(TempDepth, resDepth);

			applyColorMap(resDepth, imgShowDepth, COLORMAP_JET);

			//��ʾͼ����ʱ����ת90��
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

		str.Format(_T("%.4fs"), process_time / FPSRatio);  //��ʾ�Ĵ���ʱ�����FPSRatio
		pView->SetDlgItemText(IDC_EDIT4, str);
		str.Format(_T("%.2fFPS"), FPS*FPSRatio);  //��ʾ����ʾ��֡�ʳ���FPSRatio
		pView->SetDlgItemText(IDC_EDIT6, str);
		frameNum++;

	}
	return 0;
}




void CPCLDemoDlg::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	/*m_iren->Delete();
	m_win->Finalize();
	m_win->Delete();
	viewer->removeAllPointClouds();*/
	CDialogEx::OnCancel();
}



void CPCLDemoDlg::OnUDPPortOpen()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (!isPortOpen) {
		isPortOpen = true;
		bool port = server.startServer();
		if (port) {
			SetDlgItemText(IDC_EDIT3, _T("���ڿ���"));
		}
		else {
			SetDlgItemText(IDC_EDIT3, _T("���ڿ�������"));
		}
	}
}


void CPCLDemoDlg::OnUDPPortClose()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (isPortOpen) {
		//�ڹر�����֮ǰ�Ƚ����߳�
		GetDataStartMutex = false;
		isThreadbegin = false;
		WaitForSingleObject(h1, 1000);
		GetDlgItem(IDC_Pic_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_force_STATIC)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic1)->ShowWindow(FALSE);
		GetDlgItem(IDC_ColorMapPic2)->ShowWindow(FALSE);

		isPortOpen = false;
		server.closeServer();
		SetDlgItemText(IDC_EDIT3, _T("���ڹر�"));
	}


}


void CPCLDemoDlg::OnStartReceive()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (!GetDataStartMutex && (isPortOpen || !Online))
	{
		GetDataStartMutex = true;  //����
		isThreadbegin = true;
		GetDlgItem(IDC_Pic_STATIC)->ShowWindow(TRUE);
		GetDlgItem(IDC_force_STATIC)->ShowWindow(TRUE);
		GetDlgItem(IDC_ColorMapPic1)->ShowWindow(TRUE);
		GetDlgItem(IDC_ColorMapPic2)->ShowWindow(TRUE);
		Mat ColorMapImage = imread("colorscale_jet.jpg");
		DrawMat(ColorMapImage, IDC_ColorMapPic1, true);
		DrawMat(ColorMapImage, IDC_ColorMapPic2, true);

		// TODO: �ڴ���ӿؼ�֪ͨ����������
		DWORD dw1;     //�����µ��̣߳���UI���̷ֿ߳������һֱ�����д���������UI����һֱû���˳�����������
		h1 = CreateThread(NULL, 0, ImageProcessThread, this, 0, &dw1);
		CloseHandle(h1);
	}

}
void CPCLDemoDlg::OnStopReceive()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (GetDataStartMutex == true) {
		GetDataStartMutex = false;
		// TODO: �ڴ���ӿؼ�֪ͨ����������
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	 //?���ù�����???
	TCHAR szFilter[] = _T("�ı��ļ�(*.raw)|*.raw|�����ļ�(*.*)|*.*||");
	//?������ļ��Ի���???
	CFileDialog fileDlg(TRUE,_T("raw"), NULL, 0, szFilter, this);
	CString strFilePath;

	//?��ʾ���ļ��Ի���???
	if (IDOK == fileDlg.DoModal())
	{
		//?���������ļ��Ի����ϵġ��򿪡���ť����ѡ����ļ�·����ʾ���༭����???
		strFilePath = fileDlg.GetPathName();
		SetDlgItemText(IDC_EDIT7, strFilePath);
	}
	server.image_file = CT2A(strFilePath.GetString());

}

void CPCLDemoDlg::OnStartTracking()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	startTracking = true;
	//track.InitParam();

}


void CPCLDemoDlg::OnStopTracking()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	startTracking = false;
	track.FirstFrame = true;  ///���ٵ���֡������λtrue
}

void CPCLDemoDlg::OnBnClickedCheck2()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	server.can_save = true;
	server.is_new_file = true;
}

void CPCLDemoDlg::OnStopSave()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	server.can_save = false;
	server.out_RAW.close();
}
