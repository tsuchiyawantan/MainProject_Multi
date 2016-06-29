#pragma once

#include <iostream>
#include <sstream>
#include <time.h>
#include <Windows.h>

using namespace std;

class Log{
private:
public:
	string fileName = "";
	FILE *file = NULL;

	void Initialize(string fileName){
		// ファイルを開き、ポインタを保存
		fopen_s(&file, fileName.data(), "w");

		// ファイルが開けなかった場合、
		if (file == NULL){
			// エラーメッセージを出し、プログラムを終了します。
			exit(0);
		}
	}
	void Write(string log){
		tm newTime;
		__time64_t longTime;

		// 現在の時間を獲得し、ローカル時間（日本の時間）に変換する
		_time64(&longTime);
		_localtime64_s(&newTime, &longTime);

		// 書き込むログの内容を保存する
		fprintf(file, "[ %02d月%02d日 - %02d:%02d:%02d ]: %s\n",
			newTime.tm_mon + 1, newTime.tm_mday,
			newTime.tm_hour, newTime.tm_min, newTime.tm_sec,
			log.c_str());
	}
};