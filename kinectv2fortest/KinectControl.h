#pragma once
#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <atlbase.h>

#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define ERROR_CHECK(ret) \
  if ((ret) != S_OK) { \
	stringstream ss; \
	ss << "failed " #ret " " << hex << ret << endl; \
	throw runtime_error(ss.str().c_str()); \
    }

#define BODY_MAX	6

class KinectControl
{
	// ******* kinect ********
private:
	CComPtr<IKinectSensor> kinect = nullptr;
public:
	CComPtr<ICoordinateMapper> coordinateMapper = nullptr;

private:
	void initialize() {
		ERROR_CHECK(GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());
		BOOLEAN isOpen;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		if (!isOpen)throw runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");

		kinect->get_CoordinateMapper(&coordinateMapper);
	}

#ifdef USE_AUDIO
	// ******** audio *******
private:
	CComPtr<IAudioBeamFrameReader> audioBeamFrameReader = nullptr;
	BOOLEAN audio_initialized = false;
public:
	float beamAngle;
	float beamAngleConfidence;
	vector<BYTE> audioBuffer;
	UINT64 audioTrackingId = (UINT64)-1;
	int audioTrackingIndex = -1;
#if 0
	WaveFile audioFile;
#endif
	void initializeAudio() {
		CComPtr<IAudioSource > audioSource = nullptr;
		ERROR_CHECK(kinect->get_AudioSource(&audioSource));
		ERROR_CHECK(audioSource->OpenReader(&audioBeamFrameReader));
		UINT subFrameLength = 0;
		ERROR_CHECK(audioSource->get_SubFrameLengthInBytes(&subFrameLength));
		audioBuffer.resize(subFrameLength);
		audio_initialized = true;
#if 0
		audioFile.Open("sample.wav");
#endif
	}

	void updateAudioFrame(int flag = 0, WaveFile *pAudioFile = nullptr) {
		if (!audio_initialized) initializeAudio();
		CComPtr<IAudioBeamFrameList> audioBeamFrameList;
		auto ret = audioBeamFrameReader->AcquireLatestBeamFrames(&audioBeamFrameList);
		if (ret != S_OK) return;
		UINT beamCount = 0;
		ERROR_CHECK(audioBeamFrameList->get_BeamCount(&beamCount));
		for (int i = 0; i < beamCount; i++) {
			CComPtr<IAudioBeamFrame> audioBeamFrame;
			ERROR_CHECK(audioBeamFrameList->OpenAudioBeamFrame(i, &audioBeamFrame));
			UINT subFrameCount = 0;
			ERROR_CHECK(audioBeamFrame->get_SubFrameCount(&subFrameCount));
			for (int j = 0; j < subFrameCount; j++) {
				CComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
				ERROR_CHECK(audioBeamFrame->GetSubFrame(j, &audioBeamSubFrame));
#if 1
				ERROR_CHECK(audioBeamSubFrame->get_BeamAngle(&beamAngle));
				ERROR_CHECK(audioBeamSubFrame->get_BeamAngleConfidence(&beamAngleConfidence));

				UINT32 count = 0;
				ERROR_CHECK(audioBeamSubFrame->get_AudioBodyCorrelationCount(&count));
				if (count == 0) { audioTrackingId = (UINT64)-1; return; }
				CComPtr<IAudioBodyCorrelation> audioBodyCorrelation;
				ERROR_CHECK(audioBeamSubFrame->GetAudioBodyCorrelation(0, &audioBodyCorrelation));
				ERROR_CHECK(audioBodyCorrelation->get_BodyTrackingId(&audioTrackingId));
#else
				if (pAudioFile != nullpt) {
					audioBeamSubFrame->CopyFrameDataToArray(audioBuffer.size(), &audioBuffer[0]);
					audioFile.Write(&audioBuffer[0], audioBuffer.size());
				}
#endif
			}
		}
	}
	void updateAudioTracking() {
		audioTrackingIndex = -1;
		if (audioTrackingId == (UINT64)-1) return;
		for (int i = 0; i < BODY_MAX; i++) {
			UINT64 trackingId = 0;
			bodies[i]->get_TrackingId(&trackingId);
			if (trackingId == audioTrackingId) {
				audioTrackingIndex = i;
				break;
			}
		}
	}

	void drawAudioDirection() {
		const int w = 640, h = 480, len = h * 2 / 3;
		cv::Mat image = cv::Mat::zeros(h, w, CV_8UC4);
		float theta = -beamAngle;
		auto dx = 0 * cos(theta) - len * sin(theta);
		auto dy = 0 * sin(theta) + len * cos(theta);
		if (beamAngleConfidence > 0.5)
			cv::line(image, cv::Point(w / 2, 0), cv::Point(dx + w / 2, dy), cv::Scalar(255, 255, 255), 10);
		cv::imshow("AudioBeamAngle", image);
		//cerr << "beamAngle = " << beamAngle << "  beamAngleConfidense = " << beamAngleConfidence << endl;
	}
#endif /* USE_AUDIO */

	// ******** color ********
protected:
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	BOOLEAN color_initialized = false;
	vector<BYTE> colorBuffer;
public:
	int colorWidth;
	int colorHeight;
protected:
	unsigned int colorBytesPerPixel;
	void initializeColorFrame() {
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
		colorFrameDescription->get_Width(&colorWidth);
		colorFrameDescription->get_Height(&colorHeight);
		colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
		color_initialized = true;
	}
	void updateColorFrame()
	{
		if (!color_initialized) initializeColorFrame();
		CComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret))return;
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray((UINT)colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
	}
public:
	cv::Mat rgbImage;
	void setRGB() { setRGB(rgbImage); }
	void setRGB(cv::Mat& image) {
		updateColorFrame();
		image = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		// cv::cvtColor(image,image,CV_BGRA2BGR); // when you save with VideoWriter
	}

	// ******** depth *******
protected:
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	BOOLEAN depth_initialized = false;
	vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	UINT16 maxDepth;
	UINT16 minDepth;
	void initializeDepthFrame()
	{
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(
			&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepth));
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepth));
		depthBuffer.resize(depthWidth * depthHeight);
		depth_initialized = true;
	}
	void updateDepthFrame()
	{
		if (!depth_initialized) initializeDepthFrame();
		CComPtr<IDepthFrame> depthFrame;
		auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (FAILED(ret))return;
		ERROR_CHECK(depthFrame->CopyFrameDataToArray((UINT)depthBuffer.size(), &depthBuffer[0]));
	}
public:
	cv::Mat depthImage;
	void setDepth(bool raw = true) { setDepth(depthImage, raw); }
	void setDepth(cv::Mat& depthImage, bool raw = true) {
		updateDepthFrame();
		depthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		for (int i = 0; i < depthImage.total(); i++) {
			double rate = depthBuffer[i] / (double)maxDepth;
			depthImage.at<UINT16>(i) = (raw) ? depthBuffer[i] : (UINT16)(255 * 255 * rate); // [0, 65535]
		}
	}

	// ******* infrared ********
private:
	CComPtr<IInfraredFrameReader> infraredFrameReader = nullptr;
	BOOLEAN infrared_initialized = false;
	int infraredWidth;
	int infraredHeight;
	vector<UINT16> infraredBuffer;
	void initializeInfraredFrame() {
		CComPtr<IInfraredFrameSource> infraredFrameSource;
		ERROR_CHECK(kinect->get_InfraredFrameSource(&infraredFrameSource));
		ERROR_CHECK(infraredFrameSource->OpenReader(&infraredFrameReader));
		CComPtr<IFrameDescription> infraredFrameDescription;
		ERROR_CHECK(infraredFrameSource->get_FrameDescription(&infraredFrameDescription));
		ERROR_CHECK(infraredFrameDescription->get_Width(&infraredWidth));
		ERROR_CHECK(infraredFrameDescription->get_Height(&infraredHeight));
		//cout << "infrared: " << infraredWidth << " " << infraredHeight << endl;
		infraredBuffer.resize(infraredWidth * infraredHeight);
		infrared_initialized = true;
	}
	void updateInfraredFrame() {
		if (!infrared_initialized) initializeInfraredFrame();
		CComPtr<IInfraredFrame> infraredFrame;
		auto ret = infraredFrameReader->AcquireLatestFrame(&infraredFrame);
		if (FAILED(ret)) return;
		ERROR_CHECK(infraredFrame->CopyFrameDataToArray((UINT)infraredBuffer.size(), &infraredBuffer[0]));
	}
public:
	cv::Mat infraredImage;
	void setInfrared() { setInfrared(infraredImage); }
	void setInfrared(cv::Mat& infraredImage) {
		updateInfraredFrame();
		infraredImage = cv::Mat(infraredHeight, infraredWidth, CV_16UC1, &infraredBuffer[0]);
	}

	// ******** bodyIndex *******
protected:
	CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
	BOOLEAN bodyIndex_initialized = false;
	vector<BYTE> bodyIndexBuffer;
	int bodyIndexWidth;
	int bodyIndexHeight;
	cv::Vec3b   colors[8];
	void initializeBodyIndexFrame()
	{
		CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
		ERROR_CHECK(kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource));
		ERROR_CHECK(bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader));
		CComPtr<IFrameDescription> bodyIndexFrameDescription;
		ERROR_CHECK(bodyIndexFrameSource->get_FrameDescription(&bodyIndexFrameDescription));
		bodyIndexFrameDescription->get_Width(&bodyIndexWidth);
		bodyIndexFrameDescription->get_Height(&bodyIndexHeight);
		bodyIndexBuffer.resize(depthWidth * depthHeight);

		colors[0] = cv::Vec3b(0, 0, 0);
		colors[1] = cv::Vec3b(255, 0, 0);
		colors[2] = cv::Vec3b(0, 255, 0);
		colors[3] = cv::Vec3b(0, 0, 255);
		colors[4] = cv::Vec3b(255, 255, 0);
		colors[5] = cv::Vec3b(255, 0, 255);
		colors[6] = cv::Vec3b(0, 255, 255);
		colors[7] = cv::Vec3b(255, 255, 255);

		bodyIndex_initialized = true;
	}
	void updateBodyIndexFrame()
	{
		if (!bodyIndex_initialized) initializeBodyIndexFrame();
		CComPtr<IBodyIndexFrame> bodyIndexFrame;
		auto ret = bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame);
		if (FAILED(ret)) return;
		ERROR_CHECK(bodyIndexFrame->CopyFrameDataToArray((UINT)bodyIndexBuffer.size(), &bodyIndexBuffer[0]));
	}
public:
	cv::Mat bodyIndexImage;
	void setBodyIndex() { setBodyIndex(bodyIndexImage); }
	void setBodyIndex(cv::Mat& bodyIndexImage) {
		updateBodyIndexFrame();
		bodyIndexImage = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC3);
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			int c = (bodyIndexBuffer[i] != 255) ? bodyIndexBuffer[i] + 1 : 0;
			bodyIndexImage.at<cv::Vec3b>(y, x) = colors[c];
		}
	}

	// ******** body ********
	CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
	BOOLEAN body_initialized = false;
	IBody* bodies[BODY_MAX];
	void initializeBodyFrame() {
		CComPtr<IBodyFrameSource> bodyFrameSource;
		ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
		ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));
		CComPtr<IFrameDescription> bodyFrameDescription;
		for (auto& body : bodies) {
			body = nullptr;
		}
		body_initialized = true;
	}
	void updateBodyFrame() {
		if (!body_initialized) initializeBodyFrame();
		CComPtr<IBodyFrame> bodyFrame;
		auto ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		if (FAILED(ret)) return;
		for (auto& body : bodies) {
			if (body != nullptr) {
				body->Release();
				body = nullptr;
			}
		}
		ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(BODY_MAX, &bodies[0]));
	}
public:
	// Joint.Position.{X,Y,Z}  // CameraSpacePoint
	//    DepthSpacePoint dp;
	//    coordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dp);
	//    ColorSpacePoint cp;
	//    coordinateMapper->MapCameraPointToColorSpace(joint.Position, &cp);
	// Joint.TrackingState  == TrackingState::TrackingState_{Tracked,Inferred}
	vector<vector<Joint> > skeleton;
	void setSkeleton() { setSkeleton(skeleton); }
	void setSkeleton(vector<vector<Joint> >& skeleton) {
		updateBodyFrame();
		skeleton.clear();
		for (auto body : bodies) {
			if (body == nullptr) continue;
			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) continue;
			vector<Joint> skel;
			Joint joints[JointType::JointType_Count];
			body->GetJoints(JointType::JointType_Count, joints);
			for (auto joint : joints) skel.push_back(joint);
			skeleton.push_back(skel);
		}
	}
	// HandState::HandState_{Unknown,NotTracked,Open,Closed,Lasso}
	// TrackingConfidence::TrackingConfidence_{Low,Hight}
	pair<int, int> handState(int id = 0, bool isLeft = true) {
		if (!body_initialized) throw runtime_error("body not initialized");
		if (id < 0 || id >= BODY_MAX) throw runtime_error("handstate: bad id " + id);

		pair<int, int> ans(HandState::HandState_Unknown, TrackingConfidence::TrackingConfidence_Low);
		HRESULT hResult;
		int count = 0;
		for (auto body : bodies) {
			if (body == nullptr) continue;
			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) continue;
			count++;
			if (id != count - 1) continue;
			HandState handState;
			TrackingConfidence handConfidence;
			if (isLeft) {
				hResult = body->get_HandLeftState(&handState);
				if (!SUCCEEDED(hResult)) return ans;
				hResult = body->get_HandLeftConfidence(&handConfidence);
				if (!SUCCEEDED(hResult)) return ans;
			}
			else {
				hResult = body->get_HandRightState(&handState);
				if (!SUCCEEDED(hResult)) return ans;
				hResult = body->get_HandRightConfidence(&handConfidence);
				if (!SUCCEEDED(hResult)) return ans;
			}
			ans.first = handState; ans.second = handConfidence;
			break;
		}
		return ans;
	}

public:
	KinectControl() { initialize(); }
	~KinectControl() {
		if (kinect != nullptr) kinect->Close();
	}
};