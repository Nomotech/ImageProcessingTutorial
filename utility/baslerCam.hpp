#pragma once
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>


class BASLER {
public:
	int CAM_WIDTH = 640;
	int CAM_HEIGHT = 480;
	int CAM_FPS = 300;
	int timeout = 5000;

	int cameraNum = 0;
	Pylon::CTlFactory* tlFactory;
	Pylon::CImageFormatConverter formatConverter;
	Pylon::DeviceInfoList_t devices;
	Pylon::CBaslerUsbInstantCamera *camera;
	Pylon::EGrabStrategy grab_strategy;

	BASLER();
	~BASLER();
	int init(int id);
	int setParam(double exposure, int width, int height, bool trigger = false);
	int start();
	int getData(void* data);
	int stop();
};
