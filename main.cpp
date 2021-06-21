/*
 *  Copyright (c) 2003-2012, Shoichi Hasegawa and Springhead development team
 *  All rights reserved.
 *  This software is free software. You can freely use, distribute and modify this
 *  software. Please deal with this software under one of the following licenses:
 *  This license itself, Boost Software License, The MIT License, The BSD License.
 */

 /**
	 SPIDARG6の接続確認のためのプログラム
	 グリップ位置の表示と柔らかい床が提示される
 */
 //#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include <Springhead.h>
#include <HumanInterface/SprHIDRUsb.h>
#include <HumanInterface/SprHIKeyMouse.h>
#include <Foundation/SprUTQPTimer.h>
#include <iomanip>
#include <math.h>
#include <WinSock2.h> //WinSocketを使用するためのinclude
#include <WS2tcpip.h> //WinSocketを使用するためのinclude
#include <sstream> //string型を数字型に変換
#include <vector> //長さを変えることのできる配列
#include <direct.h> //mkdir用
#include <fstream> //ファイル操作
#include  <time.h> //日時記録用


//WinSocketを使用するためのlibファイル
#pragma comment(lib, "ws2_32.lib")

using namespace Spr;

#ifdef _WIN32
#include <windows.h>
#endif

std::string object = "confirm the error between Spidar and Optitrack";

#define PI 3.14159265359
#define STR(var) #var   //引数にした変数を変数名を示す文字列リテラルとして返すマクロ関数
Vec6d End2RodCalc(Vec3d goalEndPos);
Vec3d PDCalc(Vec3d goalPoint, Vec3d currentPos);  //the Proportional Differential Controller function
Vec6d PDCalc6d(Vec3d goalPos, Vec3d currentPos, Vec3d goalRot, Vec3d currentRot);
Vec6d PIDCalc6d(Vec3d goalPos, Vec3d currentPos, Vec3d goalRot, Vec3d currentRot);
void StartWinSock();
void make_csv(std::string file_name);
std::vector<std::string> split(std::string str, char del);
double firstImpulse = 0.02;  //inital force for PD controller 

Vec3d goalPos = Vec3d(0.0, 0.0, 0.0); //goal position of the PD controller
Vec3d goalRot = Vec3d(0.0, 0.0, 0.0); //goal position of the PD controller
Vec3d max = Vec3d(5.0, 5.0, 5.0); //PD maximum force (Newton)
Vec3d maxPos = Vec3d(0.1, 0.4, 0.7);
Vec3d minPos = -maxPos;
Vec3d maxRot = Vec3d(PI / 2, PI / 2, PI / 2);
Vec3d minRot = -minRot;

WSADATA wsaData;
SOCKET sockListen;
SOCKET sock;
bool runWinSock = true;
bool runThread = false;

char buf[256];
bool switchJ = false;
bool enter = false;

//マルチスレッド用変数
DWORD WINAPI tcp_comm_thread(LPVOID lpParam);

//Motion Trackerからの位置情報を参照する場合（UnityのRodコンポーネント Rod Controllerの変数sendSpidarRodPosをtrueにしておく）
bool receiveRodPos = true;// true;

Vec3d trackerRodPos = Vec3d(0.0, 0.0, 0.0);
Quaterniond trackerRodRotQuat = Quaterniond();//Vec3d(0.0, 0.0, 0.0);

Vec6f pose;

double ddt = 0.001f;
//double KP = 0.0;//411;//21.0 * 5;
//double KD = 0.0;//1.08;//0.7;
//double KI = 0.0;//39200;//0.5;
//double rotKP = 15;//3.0 * 5;
//double rotKD = 0.0067;//0.07;
//double rotKI = 8379;//0.3;


//Optiの位置情報を使う場合
//double KP = 0.0;//24.0;//35.4;
//double KD = 0.0;//0.52;//0.775;
//double KI = 0.0;//273.8;//403.9;
//double rotKP = 0.0;//8.4;
//double rotKD = 0.0;//0.23;
//double rotKI = 0.0;//75.3;
Vec3d integral = Vec3d(0.0, 0.0, 0.0);
Vec3d rotIntegral = Vec3d(0.0, 0.0, 0.0);

Vec3d KP = Vec3d(24.0, 24.0, 24.0          ); //Vec3d(24.0, 31.8, 24.0); // For Y Axis Tuning
Vec3d KD = Vec3d(0.52, 0.52, 0.52); //Vec3d(0.52, 0.521, 0.52);
Vec3d KI = Vec3d(273.8, 273.0, 273.8);//Vec3d(273.8, 485.1, 273.8);
Vec3d rotKP = Vec3d(8.4, 8.4, 8.4);
Vec3d rotKD = Vec3d(0.23, 0.23, 0.23);
Vec3d rotKI = Vec3d(75.3, 75.3, 75.3);


double globalClock = 0.000;
float minF = 0.3f;

std::ofstream csvFile;

UTRef<HISpidarGIf> spg;

//TCP通信用スレッド
class CTcpCommThread {
public:
	HANDLE hThread;
	DWORD ThreadId;
	SOCKET sockListen;

	CTcpCommThread() {
		hThread = NULL;
	}

	void Start() {
		hThread = CreateThread(NULL, 0, tcp_comm_thread, (void*)this, 0, &ThreadId);
	}

	~CTcpCommThread() {
		if (hThread) {
			if (WAIT_TIMEOUT == WaitForSingleObject(hThread, 1000)) {
				TerminateThread(hThread, 0);
			}

			CloseHandle(hThread);
			hThread = NULL;
			printf("CloseHandle().\r\n");
		}
	}
};

int __cdecl main() {


	// 力覚インタフェースとの接続設定
	UTRef<HISdkIf> hiSdk = HISdkIf::CreateSdk();

	// win32
	DRUsb20SimpleDesc usbSimpleDesc;
	hiSdk->AddRealDevice(DRUsb20SimpleIf::GetIfInfoStatic(), &usbSimpleDesc);
	DRUsb20Sh4Desc usb20Sh4Desc;
	for (int i = 0; i < 10; ++i) {
		usb20Sh4Desc.channel = i;
		hiSdk->AddRealDevice(DRUsb20Sh4If::GetIfInfoStatic(), &usb20Sh4Desc);
	}
	// win64
	DRCyUsb20Sh4Desc cyDesc;
	for (int i = 0; i < 10; ++i) {
		cyDesc.channel = i;
		hiSdk->AddRealDevice(DRCyUsb20Sh4If::GetIfInfoStatic(), &cyDesc);
	}
	//	UART Motor Driver
	DRUARTMotorDriverDesc umDesc;
	hiSdk->AddRealDevice(DRUARTMotorDriverIf::GetIfInfoStatic(), &umDesc);
	hiSdk->AddRealDevice(DRKeyMouseWin32If::GetIfInfoStatic());
	hiSdk->Print(DSTR);
	hiSdk->Print(std::cout);

	//UTRef<HISpidar4If> spg = hiSdk->CreateHumanInterface(HISpidar4If::GetIfInfoStatic())->Cast();
	//spg->Init(&HISpidar4DDesc());

	spg = hiSdk->CreateHumanInterface(HISpidarGIf::GetIfInfoStatic())->Cast();
	//spg->Init(&HISpidarGDesc("SpidarG6T1"));
	spg->Init(&HISpidarGDesc("SlowBall"));
	//spg->Init(&HISpidarGDesc("SlowBallR"));
	//spg->Init(&HISpidarGDesc("SpidarG6X3L"));
	spg->Calibration();

	int t = 0;
	int lastTime;
	int count = 0;
	int countMax = 10000;
	int loopLastTime = timeGetTime();


	make_csv("SpidarG6");
	csvFile << object << std::endl;

#ifdef _WIN32
	lastTime = timeGetTime();
#endif
	std::cout << "The Port Number is :" << umDesc.port << std::endl;


	float length[8];
	for (int i = 0; i < 8; ++i) {
		length[i] = spg->GetMotor(i)->GetLength();
	}
	enum Mode {
		STRING,
		POSE,
		FORCE,
		PDCONTROL,
		JUGGLING,
	} mode = POSE;
	while (1) {
		int loopTime = timeGetTime();
		int loopDiff = loopTime - loopLastTime;
		loopLastTime = loopTime;
		if (_kbhit()) {
			switch (_getch()) {
			case '0x1b':
			case 'q':
			case 'Q':
				std::cout << "Quit." << std::endl;
				goto next;
			case 'f':
			case 'F':
				mode = FORCE;
				std::cout << "force feedback mode." << std::endl;
				enter = false;
				break;
			case 's':
			case 'S':
				mode = STRING;
				std::cout << "string length mode." << std::endl;
				enter = false;
				break;
			case 'p':
			case 'P':
				mode = POSE;
				std::cout << "pose mode." << std::endl;
				enter = false;
				break;
			case 'C':
			case 'c':
				spg->Calibration();
				std::cout << "Calibrate." << std::endl;
				enter = false;
				break;
			case 'd':
			case 'D':
				mode = PDCONTROL;
				std::cout << "PD control mode." << std::endl;
				enter = false;
				break;
			case 'j':
			case 'J':
				mode = JUGGLING;
				std::cout << "Juggling mode." << std::endl;
				enter = false;
				break;
			case 0xe0:
				switch (_getch())
				{
				case 0x48:
					printf("↑\n");
					KP[1] += 1.0;
					std::cout << "KP is " << KP << std::endl;

					//rotKP += 1.0;
					//std::cout << "rotKP is " << rotKP << std::endl;

					//countMax += 100.0;
					//std::cout << "countMax is " << countMax << std::endl;

					csvFile << (globalClock) << ","
						<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ","
						<< trackerRodPos[0] << "," << trackerRodPos[1] << "," << trackerRodPos[2] << "," << trackerRodRotQuat.Rotation0()[0] << "," << trackerRodRotQuat.Rotation0()[1] << "," << trackerRodRotQuat.Rotation0()[2] << ","
						<< "KP is " << KP//<< "countMax is" << countMax//<< "rotKP is " << rotKP
						<< std::endl;

					break;
				case 0x50:
					printf("↓\n");
					KP[1] -= 1.0;
					std::cout << "KP is " << KP << std::endl;

					//rotKP -= 1.0;
					//std::cout << "rotKP is " << rotKP << std::endl;

					//countMax -= 100.0;
					//std::cout << "countMax is " << countMax << std::endl;


					csvFile << (globalClock) << ","
						<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ","
						<< trackerRodPos[0] << "," << trackerRodPos[1] << "," << trackerRodPos[2] << "," << trackerRodRotQuat.Rotation0()[0] << "," << trackerRodRotQuat.Rotation0()[1] << "," << trackerRodRotQuat.Rotation0()[2] << ","
						<< "KP is " << KP //<< "countMax is" << countMax//<< "rotKP is " << rotKP
						<< std::endl;
					break;
				case 0x4b:
					printf("←\n");
					rotKP[1] -= 0.1;
					std::cout << "rotKP is " << rotKP << std::endl;
					csvFile << (globalClock) << ","
						<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ","
						<< trackerRodPos[0] << "," << trackerRodPos[1] << "," << trackerRodPos[2] << "," << trackerRodRotQuat.Rotation0()[0] << "," << trackerRodRotQuat.Rotation0()[1] << "," << trackerRodRotQuat.Rotation0()[2] << ","
						<< "rotKP is " << rotKP
						<< std::endl;
					break;
				case 0x4d:
					printf("→\n");
					rotKP[1] += 0.1;
					std::cout << "rotKP is " << rotKP << std::endl;
					csvFile << (globalClock) << ","
						<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ","
						<< trackerRodPos[0] << "," << trackerRodPos[1] << "," << trackerRodPos[2] << "," << trackerRodRotQuat.Rotation0()[0] << "," << trackerRodRotQuat.Rotation0()[1] << "," << trackerRodRotQuat.Rotation0()[2] << ","
						<< "rotKP is " << rotKP
						<< std::endl;
					break;
				}
				break;


			}
		}

		t += 1;
		if (t >= 1000) {
			t = 0;
#ifdef _WIN32
			int time = timeGetTime();
#endif
			int diff = time - lastTime;
			DPF("\r\nDuration: %d,  Freq: %f\n", diff, 1000.0 / diff * 1000);
			lastTime = time;
		}
		//DPF("t=%d", t);
		spg->Update(0.001f);
		if (mode == FORCE) {
			//	Virtual floor
			Vec3f spgpos = spg->GetPosition();
			//		std::cout << std::setprecision(2) << spgpos << std::endl;
			Vec3f f(0.0, 0.0, 0.0);
			if (spgpos.y < -0.015) {
				f.y = (float)(-(spgpos.y - -0.015) * 1000);
			}
			spg->SetForce(f, Vec3f());
		}
		else if (mode == STRING) {
			//	print string length
			for (size_t i = 0; i < spg->NMotor(); ++i) {
				printf(" %7.4f", spg->GetMotor(i)->GetLength() - length[i]);
			}
			std::cout << std::endl;
		}
		else if (mode == POSE) {
			if (!enter) {
				//csvFile << STR(POSE) << std::endl;
				//csvFile << "time" << "," << "RodPosition" << std::endl;
				enter = true;
			}





			Vec3f pos = spg->GetPosition();
			Quaternionf ori = spg->GetOrientation();
			spg->SetPose(pos, ori);
			pose.sub_vector(0, Vec3f()) = spg->GetPosition();
			pose.sub_vector(3, Vec3f()) = spg->GetOrientation().Rotation0();

			for (size_t i = 0; i < pose.size(); ++i) {
				printf(" %7.4f", pose[i]);
			}
			std::cout << std::endl;

			//csvFileに経過時間とposeを記録
			//csvFile << (globalClock += loopDiff / 1000.0) << ","
			//	<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5]
			//	<< std::endl;

		}
		else if (mode == PDCONTROL) {
			csvFile << STR(PDCONTROL) << std::endl;

			Vec3d currentPos = spg->GetPosition();
			Vec3d currentRot = spg->GetOrientation().Rotation0();

			//Vec3d nextForce = PDCalc(goalPos, currentPos);
			//spg->SetForce(nextForce, Vec3f());

			//Vec6d nextForce6d = PDCalc6d(goalPos, currentPos, goalRot, currentRot);
			Vec6d nextForce6d = PIDCalc6d(goalPos, currentPos, goalRot, currentRot);
			//for (int i = 0; i < 6; i++) {
			//	if (nextForce6d[i] > 18.0f) nextForce6d[i] = 18.0f;
			//}
			//printf("NextForce is% f, % f, % f, % f, % f, % f\n", nextForce6d[0], nextForce6d[1], nextForce6d[2], nextForce6d[3], nextForce6d[4], nextForce6d[5]);
			spg->SetForce(nextForce6d.sub_vector(0, Vec3d()), nextForce6d.sub_vector(3, Vec3d()));


			//goalPos = Vec3d(0.0, 0.0, 0.2 * sin(2.0 * PI * ((double)count / (double)countMax)));
			//goalPos = Vec3d(0.0, 0.2 * sin(2.0 * PI * ((double)count / (double)countMax)), 0.0);
			//goalPos = Vec3d(0.2 * sin(2.0 * PI * ((double)count / (double)countMax)), 0.0, 0.0);
			//goalPos = Vec3d(0.0, 0.3f, 0.0);
			//goalRot = Vec3d(0.0, 0.0, 0.0);
			//goalRot = Vec3d(PI/8 + PI/8*(cos(2.0 * PI * ((double)count / (double)countMax))), 0.0, 0.0);
			//goalRot = Vec3d(0.0, 0.0, PI / 8 * sin(2.0/2 * PI * ((double)count / (double)countMax)));

			//goalPos = Vec3d(0.1 * cos(2.0 * PI * ((double)count / (double)countMax)), 0.2q * sin(2.0 * PI * ((double)count / (double)countMax)), 0); // EndEffectorなので70cm下で円運動

			float PHI = PI / 16;
			goalPos = Vec3d(sin(PHI) / 2 * cos(2.0 * PI * ((double)count / (double)countMax)),
				0.1 * sin(2.0 * PI * ((double)count / (double)countMax)),
				sin(PHI) / 2 * -sin(2.0 * PI * ((double)count / (double)countMax)));
			goalRot = Vec3d(PHI * sin(2.0 * PI * ((double)count / (double)countMax)), 0.0, PHI * cos(2.0 * PI * ((double)count / (double)countMax)));
			//goalPos = Vec3d(sin(PHI) / 2 * cos(2.0 * PI * ((double)count / (double)countMax)),
			//	0.3 * sin(2.0 * PI * ((double)count / (double)countMax)),
			//	sin(PHI) / 2 * -sin(2.0 * PI * ((double)count / (double)countMax)));
			//goalRot = Vec3d(PHI * sin(2.0 * PI * ((double)count / (double)countMax)), 0.0, PHI * cos(2.0 * PI * ((double)count / (double)countMax)));

			//Vec6d RodPose = End2RodCalc(goalEndPos);
			//goalPos = RodPose.sub_vector(0, Vec3d());
			//goalRot = RodPose.sub_vector(3, Vec3d());

			/*
			if (currentPos.y > 0.20) goalPos = Vec3d(0.0, -0.40, 0.0);
			if (currentPos.y < -0.20) {
				if (goalPos.y	 == -0.40)
				{
					int time = timeGetTime();
					int diff = time - lastTime;
					printf("\rDuration: %d,  Freq: %f\n", diff, 1000.0 / diff * 1000);
					lastTime = time;
				}
				goalPos = Vec3d(0.0, 0.40, 0.0);

			}
			*/




			//goalPos = Vec3d(0.0, 0.35 * sin(2.0 * PI * ((double)count / (double)countMax)), 0.0);
			//goalRot = Vec3d(0.0, 0.0, 0.0); //goal position of the PD controller
			pose.sub_vector(0, Vec3f()) = spg->GetPosition();
			pose.sub_vector(3, Vec3f()) = spg->GetOrientation().Rotation0();


			//姿勢の表示
			for (size_t i = 0; i < pose.size(); ++i) {
				printf(" %7.4f", pose[i]);
			}
			std::cout << std::endl;



			//ループ変数
			count++;
			if (count >= countMax) {
				count = 0;
				int time = timeGetTime();
				int diff = time - lastTime;
				//printf("\r\nDuration: %d,  Freq: %f\n", diff, 1000.0 / diff * 1000);
				lastTime = time;
			}

		}
		else if (mode == JUGGLING) {
		//UnityのVR Jugglingと通信モード
			if (!enter) {
				csvFile << STR(JUGGLING) << std::endl;
				csvFile << "time" << ","
					<< "RodPosition"
					<< "," << "," << "," << "," << "," << ","
					<< "trackerRodPos"
					<< "," << "," << "," << "," << ","
					<< "goalPos and Rot"
					<< std::endl;
				enter = true;
			}
			if (!switchJ) {
				// ソケット通信の開始
				StartWinSock();
				switchJ = true;
			}

			Vec3d currentPos = spg->GetPosition();
			Vec3d currentRot = spg->GetOrientation().Rotation0();
			Quaternionf currentQua = spg->GetOrientation();
			pose.sub_vector(0, Vec3f()) = currentPos;
			pose.sub_vector(3, Vec3f()) = currentRot;

			//for (size_t i = 0; i < pose.size(); ++i) {
			//	printf(" %7.4f", pose[i]);
			//}
			//std::cout << std::endl;

			//csvFileに経過時間とposeを記録
			//csvFile << (globalClock += loopDiff / 1000.0) << ","
			//	<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5]
			//	<< std::endl;

			//csvFileに経過時間とmotion captchar データを記録, goalPosition記録
			csvFile << (globalClock) << ","
				<< pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ","
				<< trackerRodPos[0] << "," << trackerRodPos[1] << "," << trackerRodPos[2] << "," << trackerRodRotQuat.Rotation0()[0] << "," << trackerRodRotQuat.Rotation0()[1] << "," << trackerRodRotQuat.Rotation0()[2] << ","
				<< goalPos[0] << "," << goalPos[1] << "," << goalPos[2] << ","
				<< goalRot[0] << "," << goalRot[1] << "," << goalRot[2] << ","
				<< std::endl;

			//WinSocket通信が始まったら別スレッド処理を開始
			if (runWinSock) {

				if (!runThread) {
					CTcpCommThread* ptcthread = new CTcpCommThread();
					ptcthread->sockListen = sockListen;
					ptcthread->Start();
					runThread = true;
				}

				float PHI = PI / 16;


				count++;
				if (count >= countMax) {
					count = 0;
					int time = timeGetTime();
					int diff = time - lastTime;
					//printf("\r\nDuration: %d,  Freq: %f\n", diff, 1000.0 / diff * 1000);
					lastTime = time;
				}




				//goalRot = currentRot;
				//Vec3d nextForce = PDCalc(goalPos, currentPos);
				//spg->SetForce(nextForce, Vec3f());

				//Spidarの位置計算を用いて計算
				//Vec6d nextForce6d = PIDCalc6d(goalPos, currentPos, goalRot, currentRot);

				//motion trackerの情報のみを参照
				Vec6d nextForce6d = PIDCalc6d(goalPos, trackerRodPos, goalRot, trackerRodRotQuat.Rotation0());
				//Vec6d nextForce6d = PIDCalc6d(trackerRodPos, trackerRodPos, goalRot, trackerRodRotQuat.Rotation0());
				//printf("NextForce is% f, % f, % f, % f, % f, % f\n", nextForce6d[0], nextForce6d[1], nextForce6d[2], nextForce6d[3], nextForce6d[4], nextForce6d[5]);
				spg->SetPose(trackerRodPos, trackerRodRotQuat);
				spg->SetForce(nextForce6d.sub_vector(0, Vec3d()), nextForce6d.sub_vector(3, Vec3d()));


			}
			else {
				std::cout << "Can't Open WinSock" << std::endl;
				return 0;
			}
		}
	}
next:
	//spg->SetForce(Vec3d(0,0,0), Vec3d(0, 0, 0));
	if (runWinSock) {
		// ソケット通信を終了
		closesocket(sock);
		// Winsockを終了
		WSACleanup();
	};
#if 0	//	test for KeyMouseWin32
	DRKeyMouseWin32If* wif = hiSdk->FindRealDevice("KeyMouseWin32")->Cast();
	wif->Update();
	DVKeyMouseIf* keyMouse = wif->Rent(DVKeyMouseIf::GetIfInfoStatic(), NULL, 0)->Cast();
	while (1) {
		if (keyMouse->GetKeyState('Q') & DVKeySt::PRESSED) return 0;
		for (int i = 0; i < 200; ++i) {
			if (keyMouse->GetKeyState(i) & DVKeySt::PRESSED) {
				std::cout << i << " '" << (char)i << "' " << std::endl;
			}
		}
	}
#endif
}

double RodLength = 1.0;
Vec6d End2RodCalc(Vec3d goalEndPos) {
	Vec3d rodPos;
	Vec3d rodRot;

	rodPos = Vec3d(goalEndPos.x / 2, sqrt(pow(RodLength, 2) - pow(goalEndPos.x, 2) - pow(goalEndPos.z, 2)) / 2 + goalEndPos.y, goalEndPos.z / 2);
	rodRot = Vec3d(0, acos(goalEndPos.x / sqrt(pow(goalEndPos.x, 2) + pow(goalEndPos.z, 2))), asin(sqrt(pow(goalEndPos.x, 2) + pow(goalEndPos.z, 2)) / RodLength));
	Vec6d out;
	out.sub_vector(0, Vec3d()) = rodPos;
	out.sub_vector(3, Vec3d()) = rodRot;
	return out;
}

Vec3d posLastError = Vec3d(0.0, 0.0, 0.0);
Vec3d PDCalc(Vec3d goalPos, Vec3d currentPos) {
	//calculate error
	Vec3d posError = goalPos - currentPos;
	//std::cout << "Distance Error: " << error << std::endl;  
	Vec3d Pout;
	Vec3d derivative;
	Vec3d Dout;


	for (int i = 0; i < 3; i++) {
		//calculate the proportional term 
		Pout[i] = KP[i] * posError[i];

		// Derivative term
		derivative[i] = (posError[i] - posLastError[i]) / ddt;
		Dout[i] = KD[i] * derivative[i];
	}


	/*
	//calculate the proportional term
	Vec3d Pout = KP * posError;

	// Derivative term
	Vec3d derivative = (posError - posLastError) / ddt;
	Vec3d Dout = KD * derivative;
	*/


	//Join the proportion and derivative terms
	Vec3d output = Pout + Dout;

	// Restrict to maximum force
	//output.element_max(max);

	posLastError = posError;
	return output;
}



Vec3d rotLastError = Vec3d(0.0, 0.0, 0.0);


Vec6d PIDCalc6d(Vec3d goalPos, Vec3d currentPos, Vec3d goalRot, Vec3d currentRot) {
	//calculate error
	Vec3d posError = goalPos - currentPos;
	Vec3d rotError = goalRot - currentRot;

	//std::cout << "Distance Error: " << posError << std::endl;
	//std::cout << "Rotation Error: " << rotError << std::endl;

	Vec3d Pout;
	Vec3d rotPout;
	Vec3d derivative;
	Vec3d rotDerivative;
	Vec3d Dout;
	Vec3d rotDout;
	Vec3d Iout;
	Vec3d rotIout;

	for (int i = 0; i < 3; i++) {
		//calculate the proportional term 
		Pout[i] = KP[i] * posError[i];
		rotPout[i] = rotKP[i] * rotError[i];

		// Derivative term
		derivative[i] = (posError[i] - posLastError[i]) / ddt;
		rotDerivative[i] = (rotError[i] - rotLastError[i]) / ddt;
		Dout[i] = KD[i] * derivative[i];
		rotDout[i] = rotKD[i] * rotDerivative[i];

		// Integral term
		integral[i] = posError[i] * ddt;
		rotIntegral[i] = rotError[i] * ddt;
		Iout[i] = KI[i] * integral[i];
		rotIout[i] = rotKI[i] * rotIntegral[i];
	}

	//Join the proportion and derivative terms
	Vec3d output = Pout + Dout + Iout;
	Vec3d rotOutput = rotPout + rotDout + rotIout;

	// Restrict to maximum force
	//output.element_max(max);

	posLastError = posError;
	rotLastError = rotError;
	Vec6d out;
	out.sub_vector(0, Vec3d()) = output;
	out.sub_vector(3, Vec3d()) = rotOutput;
	globalClock = globalClock + ddt;

	//y軸回りは回転してほしくない
	out[4] = 0.0;

	//debugFile << globalClock << "," << posError.y << "," << output.y << std::endl;
	//debugFile << globalClock << "," << rotError.z << "," << rotOutput.z << std::endl;
	return out;
}

//WinSocketを開始する
void StartWinSock() {
	//WinSockの初期化
	WSAStartup(WINSOCK_VERSION, &wsaData);

	struct sockaddr_in addr;
	// AF_INETを渡すことによりIPv4で通信する事を指定
	addr.sin_family = AF_INET;
	// ポート番号を指定好きなポートを指定してください。
	addr.sin_port = htons(12345);
	// INADDR_ANYを指定する事によりどのIPからでも通信を受け取る状態にする。
	addr.sin_addr.S_un.S_addr = INADDR_ANY;

	// ソケットを作成
	sockListen = socket(addr.sin_family, SOCK_STREAM, 0);

	// ソケットの作成に失敗していないかどうか
	if (sockListen == INVALID_SOCKET)
	{
		printf("socket failed\n");
		runWinSock = false;
		return;
	}

	// ソケットにアドレスを割り当てる
	if (bind(sockListen, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
	{
		printf("bind: error");
		runWinSock = false;
		return;
	}

	// ソケットを接続待ち状態にする
	if (listen(sockListen, 5) == SOCKET_ERROR)
	{
		printf("listen: error");
		runWinSock = false;
		return;
	}



	struct sockaddr_in client;
	int clientlen = sizeof(client);
	memset(buf, 0, sizeof(buf));

	// クライアントからの接続を待つ
	sock = accept(sockListen, (struct sockaddr*)&client, &clientlen);
	printf("connected\n");

	runWinSock = true;
	return;
}

//split関数
std::vector<std::string> split(std::string str, char del) {
	int first = 0;
	int last = str.find_first_of(del);

	std::vector<std::string> result;

	while (first < str.size()) {
		std::string subStr(str, first, last - first); //strの{first}から{last - first}文字分を抽出

		result.push_back(subStr);

		first = last + 1;
		last = str.find_first_of(del, first);

		if (last == std::string::npos) {
			last = str.size();
		}

	}

	return result;
}



DWORD WINAPI tcp_comm_thread(LPVOID lpParam) {

	while (1) {
		// クライアントからデータを受け取る
		recv(sock, buf, sizeof(buf), 0);
		// 受け取った文字を表示
		//printf("%s\n", buf);

		//受け取った文字を分割、数字に変換
		std::vector<std::string> spl = split(buf, ',');
		//目標姿勢を取得
		std::vector<float> spd = {
			strtof(spl[0].c_str(), NULL) ,
			strtof(spl[1].c_str(), NULL) ,
			strtof(spl[2].c_str(), NULL) ,
			strtof(spl[3].c_str(), NULL) ,
			strtof(spl[4].c_str(), NULL) ,
			strtof(spl[5].c_str(), NULL) ,
			strtof(spl[6].c_str(), NULL) };
		Quaternionf spdRotQ = Quaternionf(spd[3], spd[4], spd[5], spd[6]);
		//printf("%f,%f,%f,%f,%f,%f\n", spd[0], spd[1], spd[2], 
		//	spdRotQ.Rotation0()[0], spdRotQ.Rotation0()[1], spdRotQ.Rotation0()[2]);
		goalPos = Vec3f(spd[0], spd[1], spd[2]);
		goalRot = spdRotQ.Rotation0();

		//UnityのRodController.csからモーショントラッカーのロッド位置情報を取得する
		if (receiveRodPos) {
			std::vector<float> spd2 = {
				strtof(spl[7].c_str(), NULL) ,
				strtof(spl[8].c_str(), NULL) ,
				strtof(spl[9].c_str(), NULL) ,
				strtof(spl[10].c_str(), NULL) ,
				strtof(spl[11].c_str(), NULL) ,
				strtof(spl[12].c_str(), NULL) ,
				strtof(spl[13].c_str(), NULL) };
			trackerRodPos = Vec3f(spd2[0], spd2[1], spd2[2]);
			//goalPos = Vec3f(0.0f, spd[1], 0.0f);juggli
			trackerRodRotQuat = Quaternionf(spd2[3], spd2[4], spd2[5], spd2[6]);
			spg->SetPose(trackerRodPos, trackerRodRotQuat);

			//printf("Spidar is : \n%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f\n", spd2[0], spd2[1], spd2[2],
			//	trackerRodRotQuat.Rotation0()[0], trackerRodRotQuat.Rotation0()[1], trackerRodRotQuat.Rotation0()[2]);


			// print error between current pose and motin tracker
			pose.sub_vector(0, Vec3f()) = spg->GetPosition();
			pose.sub_vector(3, Vec3f()) = spg->GetOrientation().Rotation0();
			//for (size_t i = 0; i < 3; ++i) {
			//	printf(" %7.4f", (pose[i] - spd2[i]));
			//}for (size_t i = 0; i < 3; ++i) {
			//	printf(" %7.4f", (pose[i+3] - trackerRodRotQuat.Rotation0()[i]));
			//}
			//std::cout << std::endl;



			//for (size_t i = 0; i < pose.size(); ++i) {
			//	printf(" %7.4f", (pose[i]));
			//}


		}

	}


}

void tcp_server()
{
	unsigned short port = 10000;
	SOCKET sockListen;
	struct sockaddr_in sockAddrInListen;

	memset(&sockAddrInListen, 0, sizeof(sockAddrInListen));
	sockAddrInListen.sin_port = htons(port);
	sockAddrInListen.sin_family = AF_INET;
	sockAddrInListen.sin_addr.s_addr = htonl(INADDR_ANY);

	sockListen = socket(AF_INET, SOCK_STREAM, 0);
	if (INVALID_SOCKET == sockListen)
		return;

	if (SOCKET_ERROR == bind(sockListen,
		(struct sockaddr*)&sockAddrInListen,
		sizeof(sockAddrInListen))) {
		closesocket(sockListen);
		return;
	}

	for (int n = 0; n < 100; n++) {
		if (SOCKET_ERROR == listen(sockListen, SOMAXCONN)) {
			closesocket(sockListen);
			return;
		}

		fd_set mask;
		FD_ZERO(&mask);
		FD_SET(sockListen, &mask);
		struct timeval tv = { 1, 0 };

		int ret = select((int)sockListen + 1, &mask, NULL, NULL, &tv);
		if (ret == SOCKET_ERROR) {
			break;
		}
		else if (ret == 0) {
			printf("wait 1 second ... \r\n");
			continue;
		}
		else if (FD_ISSET(sockListen, &mask)) {
			CTcpCommThread* ptcthread = new CTcpCommThread();
			ptcthread->sockListen = sockListen;
			ptcthread->Start();
		}

	}
	closesocket(sockListen);
}

void make_csv(std::string file_name) {
	const char* dir = "./csv";
	if (_mkdir(dir) != 0) { std::cout << "./csv already exists" << std::endl; }
	time_t     current = time(NULL);
	struct tm local;
	//time(&current);                     /* 現在の時刻を取得 */
	localtime_s(&local, &current);
	//local = localtime(&current);        /* 地方時の構造体に変換 */
	std::ostringstream oss;
	oss << "./csv/" << file_name << local.tm_mon + 1 << local.tm_mday << "_" << local.tm_hour << local.tm_min << "_" << local.tm_sec << ".csv";
	//string file_name = oss.str();
	file_name = oss.str();


	csvFile.open(file_name);
}