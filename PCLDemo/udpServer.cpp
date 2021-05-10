#include"udpServer.h"
//#include"stdafx.h"
//UDPServer::UDPServer()
//{
//	image_file= ".\\RAW\\2hz2.raw";
//}
int UDPServer::dealData(cv::Mat &depth, cv::Mat &force){
	static int k = 0;
	unsigned short _recv_depth[4096];
	int n = 0;
	unsigned short _recv_force[4096];
	int m = 0;
	std::stringstream ss;
	//cv::namedWindow("depth img", CV_WINDOW_NORMAL);
	//cv::resizeWindow("depth img", 640, 640);
	//cv::namedWindow("force img", CV_WINDOW_NORMAL);
	//cv::resizeWindow("force img", 640, 640);
	for (int i = 0; i < 8192; i++){
		if (i % 2 == 0) _recv_force[m++] = image[i];
		else _recv_depth[n++] = image[i];
	}
	cv::Mat recv_depth(64, 64, CV_16UC1, _recv_depth);
	cv::Mat recv_force(64, 64, CV_16UC1, _recv_force);

	//cv::imshow("depth img", recv_depth * 70);
	//cv::imshow("force img", recv_force * 70);
	//cv::waitKey(1);
	//ss << ".\\Received\\sequence" << k << ".png";
	//cv::imwrite(ss.str(), recv_img);
	depth = recv_depth.clone();
	force = recv_force.clone();
	return 0;
}
int UDPServer::readOnePack(char *buf, int recvSize) {
	if (!can_process) {
		for (int i = 0; i < recvSize; i++) {
			buffer.push_back(buf[i]);
		}
	}
	if (buffer.size() > 200)
		can_process = true;
	return 0;
}

int UDPServer::mergeDataDeque(char *buf, int recvSize, cv::Mat &depth, cv::Mat &force){
	static bool found_head = false;
	if (can_process){
		if (!found_head){
			while (buffer.size() > 200){
				if ((unsigned char)buffer.front() == 0x55){
					buffer.pop_front();
					if ((unsigned char)buffer.front() == 0x55){
						buffer.pop_front();
						found_head = true;
						start = clock();
						break;
					}
				}
				else
					buffer.pop_front();
			}
		}
		while (buffer.size() > 200){
			unsigned char high;
			unsigned char low;
			high = (unsigned char)buffer.front();
			buffer.pop_front();
			low = (unsigned char)buffer.front();
			buffer.pop_front();
			unsigned short img_point = ((unsigned char)high << 8) | (unsigned char)low;
			if (img_point != 0x5555 && img_point != 0xaaaa){
				image[img_pos] = img_point;
				img_pos++;
			}
			else if (img_point == 0xaaaa){
				if (img_pos == 8192){
					printf("Frame end!Receice successful!\n\n");
					img_pos = 0;
					frame_flag = 0;
					dealData(depth, force);  //正常接收一帧数据，进入处理
					end = clock();
					cout << "帧率 :  " << CLOCKS_PER_SEC / (double)(end - start) << "FPS" << endl;
					found_head = false;
					haveOneFrame = true;
					can_process = false;
					readOnePack(buf, recvSize);
					return 0;
				}
				else if (img_pos != 8192){
					printf("Transmit Error,start another frame!current img_pos = %d\n", img_pos);
					img_pos = 0;
					frame_flag = 0;
					found_head = false;
					break;
				}
			}
			else if (img_point == 0x5555){
				printf("Transmit Error,start another frame before end!current img_pos = %d\n", img_pos);
				img_pos = 0;
			}
			if (img_pos > 8192){
				img_pos = 0;
				printf("img_pos error!start another frame.");
				found_head = false;
				break;
			}
		}
		can_process = false;
	}
	readOnePack(buf, recvSize);
	return 0;
}

int UDPServer::saveDataStream(char *buf, int recvSize) {
	if (is_new_file || UDPPackCount == 1000000) {
		stringstream filename;
		t = time(NULL);
		local = localtime(&t);
		filename << ".\\RAW\\" << local->tm_year << local->tm_mon << local->tm_mday << local->tm_hour << local->tm_min << local->tm_sec << ".raw";
		is_new_file = false;
		out_RAW.open(filename.str(), std::ios::out | std::ios::app | std::ios::binary);
	}
	if (!out_RAW.is_open())
	{
		printf("read file error!");
		return 0;
	}
	out_RAW.write((char*)buf, recvSize);
	out_RAW.flush();
	out_RAW.clear();
	UDPPackCount++;
}

bool UDPServer::startServer()
{
	// 初始化套接字动态库
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0){
		printf("WSAStartup failed:%d!\n", WSAGetLastError());
		return -1;
	}

	//socketSrv = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	socketSrv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	SOCKADDR_len = sizeof(SOCKADDR);

	// 设置服务器地址
	ZeroMemory(strRecv, BUF_SIZE);
	//addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrSrv.sin_addr.S_un.S_addr = inet_addr("192.168.0.3");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(8080);

	// 绑定套接字
	iRet =::bind(socketSrv, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
	if (SOCKET_ERROR == iRet)
	{
		printf("bind failed%d!\n", WSAGetLastError());
		closesocket(socketSrv);
		WSACleanup();
		return false;
	}
	else {
		printf("udp server start...\n");
		return true;
	}
}

int UDPServer::receiveData(cv::Mat &depth, cv::Mat &force){
	haveOneFrame = false;
	while (!haveOneFrame){
		iRet = recvfrom(socketSrv, strRecv, BUF_SIZE, 0, (SOCKADDR*)&addrClient, &SOCKADDR_len);
		if (SOCKET_ERROR == iRet){
			printf("recvfrom failed %d!\n", WSAGetLastError());
			closesocket(socketSrv);
			WSACleanup();
			return -1;
		}
		if (iRet > 0){
			if (can_save) {
				saveDataStream(strRecv, iRet);//存储udp数据流
			}
			mergeDataDeque(strRecv, BUF_SIZE, depth, force);
		}
	}
}

int UDPServer::closeServer(){
	closesocket(socketSrv);
	WSACleanup();
	return 0;
}

int UDPServer::readRAWData(cv::Mat &depth, cv::Mat &force){
	haveOneFrame = false;
	if (!file_read){
		fp = fopen(image_file.c_str(), "rb");
		if (fp == NULL)
		{
			printf("can't open file\n");
			return 1;
		}
		file_read = true;
	}
	while (!haveOneFrame) {
		fread(strRecv, sizeof(char), BUF_SIZE, fp);
		mergeDataDeque(strRecv, BUF_SIZE, depth, force);
		if (can_save) {
			saveDataStream(strRecv, BUF_SIZE);//存储udp数据流
		}
	}

	if (FrameCount++ == 562) {
		fseek(fp, 0, SEEK_SET);
		FrameCount = 0;
	}
	//Sleep(500);
	//printf("%d ", i);
}