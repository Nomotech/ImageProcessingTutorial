#include "baslerCam.hpp"

BASLER::BASLER() {
	Pylon::PylonInitialize();
	Pylon::CTlFactory& tlFactoryTmp = Pylon::CTlFactory::GetInstance();

	tlFactory = &tlFactoryTmp;
	if (cameraNum == 0) {
		cameraNum = tlFactory->EnumerateDevices(devices);
		if (cameraNum == 0) std::runtime_error("No camera present.");
	}
	cameraNum++;
	formatConverter.OutputPixelFormat = Pylon::PixelType_RGB8packed;
}



BASLER::~BASLER(){
	camera->Close();
	Pylon::PylonTerminate();
}
			

int BASLER::init (int id) {

	if (id < cameraNum) {
		camera = new Pylon::CBaslerUsbInstantCamera(tlFactory->CreateDevice(devices[id]));
	}
	else
	{
		std::runtime_error("number of camera is over flow.");
		return false;
	}
	grab_strategy = Pylon::GrabStrategy_OneByOne;
	camera->Open();
	camera->AcquisitionFrameRateEnable.SetValue(true);
	camera->AcquisitionFrameRate.SetValue(CAM_FPS);
	camera->Width.SetValue(CAM_WIDTH);
	camera->Height.SetValue(CAM_HEIGHT);
	return 1;
}


int BASLER::setParam(double exposure, int width, int height, bool trigger) {
	// exposure
	camera->ExposureTime.SetValue(exposure);

	// size
	CAM_WIDTH = width;
	CAM_HEIGHT = height;
	camera->Width.SetValue(CAM_WIDTH);
	camera->Height.SetValue(CAM_HEIGHT);

	// data type
	camera->PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_Mono8);
	formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;

	// trigger
	if (trigger) {
		Basler_UsbCameraParams::LineSelectorEnums line = Basler_UsbCameraParams::LineSelector_Line3;
	}
	return 1;
}


int BASLER::start() {
	camera->StartGrabbing(grab_strategy);
	return 1;
}

int BASLER::getData(void* data) {
	std::size_t frameNumber;
	Pylon::CGrabResultPtr ptrGrabResult;
	if (camera->IsGrabbing()) {
		camera->RetrieveResult(timeout, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
		if (ptrGrabResult->GrabSucceeded()){
			memcpy(data, ptrGrabResult->GetBuffer(), CAM_WIDTH * CAM_HEIGHT);
			frameNumber = ptrGrabResult->GetBlockID();
			return frameNumber;
		}
	}
	return -1;
}

int BASLER::stop() {
	camera->StopGrabbing();
	return 1;
}