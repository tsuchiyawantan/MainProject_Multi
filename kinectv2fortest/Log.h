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
		// �t�@�C�����J���A�|�C���^��ۑ�
		fopen_s(&file, fileName.data(), "w");

		// �t�@�C�����J���Ȃ������ꍇ�A
		if (file == NULL){
			// �G���[���b�Z�[�W���o���A�v���O�������I�����܂��B
			exit(0);
		}
	}
	void Write(string log){
		tm newTime;
		__time64_t longTime;

		// ���݂̎��Ԃ��l�����A���[�J�����ԁi���{�̎��ԁj�ɕϊ�����
		_time64(&longTime);
		_localtime64_s(&newTime, &longTime);

		// �������ރ��O�̓��e��ۑ�����
		fprintf(file, "[ %02d��%02d�� - %02d:%02d:%02d ]: %s\n",
			newTime.tm_mon + 1, newTime.tm_mday,
			newTime.tm_hour, newTime.tm_min, newTime.tm_sec,
			log.c_str());
	}
};