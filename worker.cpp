#include <iostream>
#include "DxLib.h"
#include <math.h>
#include <windows.h>
#define N 1000 //
#define M 181 //sensorの角度の分解能
HANDLE serial;
bool Ret;

struct Point {
	double x;
	double y;
	int team;
	int Class;
	int life;
	int group;
	int num;
	int attack;
	double sin;
	double cos;
};

struct Buf {
	unsigned char xhigh;
	unsigned char xlow;
	unsigned char yhigh;
	unsigned char ylow;
};


// プログラムは WinMain から始まります
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	static const int SCREEN_SIZE_W = 1000;  // 幅px
	static const int SCREEN_SIZE_H = 700;  // 高px
	
	
	ChangeWindowMode(TRUE);
	SetGraphMode(SCREEN_SIZE_W, SCREEN_SIZE_H, 32);   // 描画スクリーンのサイズを指定します

	//-------------------------------------------------------------------------------------------------------------------------
	///*
	//1.ポートをオープン
	//serial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	serial = CreateFile("\\\\.\\COM21", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (serial == INVALID_HANDLE_VALUE) {
		printf("Port could not open.\n");
		exit(0);
	}
	//2.送受信バッファ初期化
	Ret = SetupComm(serial, 1024, 1024);
	if (!Ret) {
		printf("SET UP FAILED\n");
		CloseHandle(serial);
		system("PAUSE");
		exit(0);
	}
	Ret = PurgeComm(serial, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!Ret) {
		printf("CLEAR FAILED\n");
		CloseHandle(serial);
		exit(0);
	}
	//3.基本通信条件の設定
	DCB dcb;
	GetCommState(serial, &dcb);
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = 115200;
	dcb.fBinary = TRUE;
	dcb.ByteSize = 8;
	dcb.fParity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	Ret = SetCommState(serial, &dcb);
	if (!Ret) {
		printf("SetCommState FAILED\n");
		CloseHandle(serial);
		system("PAUSE");
		exit(0);
	}

	//4.送信
	DWORD dwSendSize;
	DWORD dwreadSize;
	DWORD dwErrorMask;
	//*/
	//------------------------------------------------------------------------------------------------------------------


	if (DxLib_Init() == -1)		// ＤＸライブラリ初期化処理
	{
		return -1;			// エラーが起きたら直ちに終了
	}
	
	struct Point Destination[N];
	struct Point Machine;
	struct Point Wall[M];
	struct Buf buf[N];
	unsigned char pBuf[50];
	double sensor[M];
	int MouseX, MouseY;
	int old_MOUSE_INPUT_LEFT = 0;
	int old_MOUSE_INPUT_RIGHT = 0;
	int t = 0;
	int theta_m = 0;
	int range;
	int rtheta;
	int Dx;
	int Dy;
	char xpm;
	char ypm;
	int start = 0xAA;
	int end = 0xFF;


	while (ProcessMessage() == 0) {
		ClearDrawScreen();//スクリーンを初期化


		GetMousePoint(&MouseX, &MouseY);
		if (old_MOUSE_INPUT_LEFT == 0 && (GetMouseInput() & MOUSE_INPUT_LEFT) != 0) {
			Destination[t].x = MouseX;
			Destination[t].y = MouseY;
			old_MOUSE_INPUT_LEFT = 1;
			t++;
		}
		if ((GetMouseInput() & MOUSE_INPUT_LEFT) == 0) old_MOUSE_INPUT_LEFT = 0;
		if (CheckHitKey(KEY_INPUT_R) > 0) {
			for (int i = 0; i < t; i++)Destination[i] = { 0,0,0,0 };
			t = 0;
		}

		//Sを押したら目的地を送信
		if (CheckHitKey(KEY_INPUT_S) > 0) {
			Ret = WriteFile(serial, &start, sizeof(start), &dwSendSize, NULL);
			Sleep(50);
			for (int i = 0; i < t; i++) {
				Dx = ((int)Destination[i].x - SCREEN_SIZE_W / 2)*2;
				Dy = (SCREEN_SIZE_H / 2 - (int)Destination[i].y)*2;
				if (Dx< 0) {Dx *= -1; xpm = 0b10000000;}
				else xpm = 0b00000000;
				if (Dy< 0) {Dy *= -1; ypm = 0b10000000;}
				else ypm = 0b00000000;
				buf[i].xhigh = ((Dx >> 6) & 0b00111111) + xpm;
				buf[i].xlow	 =  Dx		 & 0b00111111 + 0b01000000;
				buf[i].yhigh = ((Dy >> 6) & 0b00111111) + ypm;
				buf[i].ylow	 =  Dy		 & 0b00111111 + 0b01000000;
				DrawFormatString(200, 20 * i, GetColor(155, 255, 255), "i:%d Dx;%d Dy:%d", i, buf[i].xhigh, buf[i].yhigh);
				Ret = WriteFile(serial, &buf[i].xhigh, sizeof(buf[i].xhigh), &dwSendSize, NULL);
				Sleep(50);
				Ret = WriteFile(serial, &buf[i].xlow, sizeof(buf[i].xlow), &dwSendSize, NULL);
				Sleep(50);
				Ret = WriteFile(serial, &buf[i].yhigh, sizeof(buf[i].yhigh), &dwSendSize, NULL);
				Sleep(50);
				Ret = WriteFile(serial, &buf[i].ylow, sizeof(buf[i].ylow), &dwSendSize, NULL);
				Sleep(50);
				DrawFormatString(0, 20*i, GetColor(155, 255, 255), "i:%d Dx;%d Dy:%d",i, Dx, Dy);
			}
			Ret = WriteFile(serial, &end, sizeof(end), &dwSendSize, NULL);
			Sleep(50);
		}
		//Ret = ReadFile(serial, pBuf, toReadBytes, &readBytes, NULL);

		//マシンからの情報を受信
		Ret = ReadFile(serial, pBuf, 21, &dwreadSize, NULL);
		for (int i = 0; i < 10; i++) {
			if (pBuf[i] == 0xAA) {
				Machine.x = (int)((pBuf[i + 1] << 6 | pBuf[i + 2]) & 0b111111111111);
				Machine.y = (int)((pBuf[i + 3] << 6 | pBuf[i + 4]) & 0b111111111111);
				if (pBuf[i+1] >= 0b10000000)Machine.x *= -1;
				if (pBuf[i+3] >= 0b10000000)Machine.y *= -1;
				theta_m = (int)((pBuf[i + 5] << 6 | pBuf[i + 6]) & 0b111111111111);
				//int range_ = (int)(((pBuf[i + 8] << 6 | pBuf[i + 9]) & 0b111111111111) / 10.0);
				double range_ = 10 * (((pBuf[i + 8] << 6 | pBuf[i + 9]) & 0b111111111111) - 50) / 11;
				if (range_ > 4000)range_ = 4000;
				rtheta = (int)(pBuf[i + 7]/90.0 * 180);
				//rtheta = 60;
				Wall[rtheta].x = (Machine.x + range_ * cos((rtheta + theta_m +90)*3.141592 / 180.0));
				Wall[rtheta].y = (Machine.y - range_ * sin((rtheta + theta_m +90)*3.141592 / 180.0));
				//(int)(Machine.x  + 30.0 * sin((theta_m + 90)*3.141592 / 180.0))
				DrawFormatString(0, 0, GetColor(255, 155, 155), "(%d,%d,%d)", (int)Wall[rtheta].x, (int)Wall[rtheta].y, (int)range_);
				DrawFormatString(0, 20, GetColor(255, 155, 155), "(%d,%d,%d)", theta_m,rtheta , rtheta + theta_m);
			}
		}
		
		
	
		
		Sleep(50);
		//mapの描画
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, 120);		//ブレンドモードをα(70/255)に設定
		DrawLine(0.0, SCREEN_SIZE_H / 2, SCREEN_SIZE_W, SCREEN_SIZE_H / 2, GetColor(255, 255, 155));
		DrawLine(SCREEN_SIZE_W / 2, 0.0, SCREEN_SIZE_W / 2, SCREEN_SIZE_H, GetColor(255, 255, 155));
		for (int i = 0; i * 10 < SCREEN_SIZE_H; i++) {
			if(i%10==5)SetDrawBlendMode(DX_BLENDMODE_ALPHA, 100);
			else SetDrawBlendMode(DX_BLENDMODE_ALPHA, 50);
			DrawLine(0.0, 10 * i, SCREEN_SIZE_W, 10 * i, GetColor(255, 255, 155));
		}
		for (int i = 0; i * 10 < SCREEN_SIZE_W; i++) {
			if (i % 10 == 0)SetDrawBlendMode(DX_BLENDMODE_ALPHA, 100);
			else SetDrawBlendMode(DX_BLENDMODE_ALPHA, 50);
			DrawLine(10 * i,0.0, 10 * i, SCREEN_SIZE_H, GetColor(255, 255, 155));
		}
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255);		//ブレンドモードをα(255/255)に設定


		for (int i = 1; i <= 180; i++) {
			DrawCircle((int)(Wall[i].x / 2.0 + SCREEN_SIZE_W / 2.0), (int)(SCREEN_SIZE_H / 2.0 - Wall[i].y / 2.0), 2.0, GetColor(155, 255, 155), FALSE);
			SetDrawBlendMode(DX_BLENDMODE_ALPHA, 55);		//ブレンドモードをα(255/255)に設定
			if((int)(Wall[i].y)>-10000)DrawLine((int)(Machine.x / 2 + SCREEN_SIZE_W / 2), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2), (int)(Wall[i].x / 2.0 + SCREEN_SIZE_W / 2.0), (int)(SCREEN_SIZE_H / 2.0 - Wall[i].y / 2.0), GetColor(155, 255, 155));
			SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255);		//ブレンドモードをα(255/255)に設定
		}

		for (int i = 0; i < t; i++) {
			DrawCircle(Destination[i].x, Destination[i].y, 2.0, GetColor(155, 255, 255), FALSE);
			DrawFormatString(Destination[i].x, Destination[i].y, GetColor(155, 255, 255), "(%d,%d)", ((int)Destination[i].x - SCREEN_SIZE_W/2)*2, (SCREEN_SIZE_H / 2 - (int)Destination[i].y)*2);
			if (i > 0)DrawLine(Destination[i - 1].x, Destination[i - 1].y, Destination[i].x, Destination[i].y, GetColor(62, 133, 255));    // 線を描画
		}
		DrawCircle((int)(Machine.x / 2 + SCREEN_SIZE_W / 2), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2), 20.0, GetColor(255, 155, 155), FALSE);
		DrawCircle((int)(Machine.x / 2 + SCREEN_SIZE_W / 2), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2), 1.0, GetColor(255, 155, 155), FALSE);
		DrawFormatString((int)(Machine.x / 2 + SCREEN_SIZE_W / 2)+20, (int)(SCREEN_SIZE_H / 2 - Machine.y / 2)-20, GetColor(255, 155, 155), "(%d,%d)", (int)Machine.x, (int)Machine.y);
		DrawLine((int)(Machine.x / 2 + SCREEN_SIZE_W / 2 + 30.0 * sin((theta_m + 90)*3.141592 / 180.0)), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2 + 30.0 * cos((theta_m + 90)*3.141592 / 180.0)), (int)(Machine.x / 2 + SCREEN_SIZE_W / 2 + 20.0 * sin((theta_m + 60)*3.141592 / 180.0)), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2 + 20.0 * cos((theta_m + 60)*3.141592 / 180.0)), GetColor(255, 155, 155));
		DrawLine((int)(Machine.x / 2 + SCREEN_SIZE_W / 2 + 30.0 * sin((theta_m + 90)*3.141592 / 180.0)), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2 + 30.0 * cos((theta_m + 90)*3.141592 / 180.0)), (int)(Machine.x / 2 + SCREEN_SIZE_W / 2 + 20.0 * sin((theta_m + 120)*3.141592 / 180.0)), (int)(SCREEN_SIZE_H / 2 - Machine.y / 2 + 20.0 * cos((theta_m + 120)*3.141592 / 180.0)), GetColor(255, 155, 155));
		
		//Sleep(0.1);
		// 裏画面の内容を表画面に反映
		ScreenFlip();
		if (CheckHitKey(KEY_INPUT_ESCAPE) > 0) return 0;

	}

	DxLib_End();				// ＤＸライブラリ使用の終了処理

	if (!Ret) {
		printf("SEND FAILED\n");
		CloseHandle(serial);
		system("PAUSE");
		exit(0);
	}
	printf("FINISH\n");
	CloseHandle(serial);
	system("PAUSE");
	return 0;				// ソフトの終了 
}

