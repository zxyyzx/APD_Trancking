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
//const char image_file[100] = "E:\\803����\\SinglePD_v120201210_v2\\SerialTest\\RAW\\10hz.raw";
class UDPServer{

private:
	bool file_read;					//�Ƿ��Ѷ�ȡ�ļ�
	bool haveOneFrame;				//�Ƿ��Ѷ�ȡһ֡ͼ��
	bool can_process;  //�����������ݴ���200�ֽڲ��ܿ�ʼ����

	FILE *fp;						//��ȡRAW�ļ�ָ��
	int FrameCount;
	int img_pos = 0;				//ͼ���еĵ��λ��
	int frame_flag = 0;				//�Ƿ�һ֡��ʼ��־
	int iRet = 0;					//�����̺�������ֵ
	unsigned short image[8192];		//ͼ��洢����
	char strRecv[BUF_SIZE];			//recvfrom�������ջ�����
	deque<unsigned char> buffer;	//UDP���ݺ�֡������
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

	bool startServer();				//����server
	int closeServer();				//�ر�server
	int receiveData(cv::Mat &depth, cv::Mat &force);				//��ʼ��������
	int dealData(cv::Mat &depth, cv::Mat &force);					//������յ���һ֡����
	int mergeDataDeque(char *buf, int recvSize, cv::Mat &depth, cv::Mat &force); //�����յ���UPD����������Ϊһ��ͼ��
	int saveDataStream(char *buf, int recvSize); //�����յ���UPD����ΪRAW
	int readRAWData(cv::Mat &depth, cv::Mat &force);
	int readOnePack(char *buf, int recvSize);				//��һ��UDP��

	string image_file = ".\\RAW\\2hz2.raw";
};

#endif
