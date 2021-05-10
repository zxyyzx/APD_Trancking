#ifndef _UDPSERVER_H_
#define _UDPSERVER_H_
#include<stdio.h>
#include <winsock2.h>
#include <deque>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <fstream>
#include <time.h>
#include<string>
#pragma comment(lib,"ws2_32.lib")

#define	BUF_SIZE	400
#define PORT_		8080
using namespace std;

//const char image_file[100] = ".\\RAW\\2hz2.raw";
//const char image_file[100] = "E:\\803出差\\SinglePD_v120201210_v2\\SerialTest\\RAW\\10hz.raw";
class UDPServer{

private:
	bool file_read;					//是否已读取文件
	bool haveOneFrame;				//是否已读取一帧图像
	bool can_process;  //缓冲区中数据大于200字节才能开始处理

	FILE *fp;						//读取RAW文件指针
	int FrameCount;
	int img_pos = 0;				//图像中的点的位置
	int frame_flag = 0;				//是否一帧开始标志
	int iRet = 0;					//网络编程函数返回值
	unsigned short image[8192];		//图像存储数组
	char strRecv[BUF_SIZE];			//recvfrom函数接收缓冲区
	deque<unsigned char> buffer;	//UDP数据合帧缓冲区
	WSADATA wsd;
	SOCKET socketSrv;
	SOCKADDR_IN addrSrv;
	SOCKADDR_IN addrClient;
	int SOCKADDR_len;
	clock_t start = 0, end = 0;

	struct tm *local;
	time_t t;

public:
	UDPServer(){
		file_read = false;
		haveOneFrame = false;
		FrameCount = 0;
		can_process = false;
		can_save = false;
		is_new_file = false;
		UDPPackCount = 0;
	}
	bool can_save;
	bool is_new_file;
	int UDPPackCount;
	std::ofstream out_RAW;

	bool startServer();				//开启server
	int closeServer();				//关闭server
	int receiveData(cv::Mat &depth, cv::Mat &force);				//开始接收数据
	int dealData(cv::Mat &depth, cv::Mat &force);					//处理接收到的一帧数据
	int mergeDataDeque(char *buf, int recvSize, cv::Mat &depth, cv::Mat &force); //将接收到的UPD包解析并存为一副图像
	int saveDataStream(char *buf, int recvSize); //将接收到的UPD包存为RAW
	int readRAWData(cv::Mat &depth, cv::Mat &force);
	int readOnePack(char *buf, int recvSize);				//读一个UDP包

	string image_file = ".\\RAW\\2hz2.raw";
};

#endif
