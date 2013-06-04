#include "KinectSensor.h"

KinectSensor::KinectSensor(INuiSensor * sens, HANDLE depth, HANDLE stream) : nuiSensor(sens), depthEvent(depth), streamHandle(stream){};

KinectSensor::~KinectSensor(void){
	nuiSensor->Release();
};

int KinectSensor::SensorCount(){
	int iSensorCount = 0;
	if(NuiGetSensorCount(&iSensorCount) < 0)
		return -1;
	else
		return iSensorCount;
};

KinectSensor::Ptr KinectSensor::fromDeviceIndex(int id){
	INuiSensor * pNuiSensor;
	bool fail = NuiCreateSensorByIndex(id, &pNuiSensor) < 0;
	if(!fail)
		fail |= pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX) < 0; 

	if(!fail)
		fail |= !(pNuiSensor->NuiStatus() == 0);

	HANDLE depthEvent2, streamHandle2;
	if(!fail){
		// Create an event that will be signaled when depth data is available
		depthEvent2 = CreateEvent(NULL, TRUE, FALSE, NULL);

		// Open a depth image stream to receive depth frames
		fail |= pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
			NUI_IMAGE_RESOLUTION_640x480,
			0,
			2,
			depthEvent2,
			&streamHandle2);
	}

	if(fail){
		pNuiSensor->Release();
		return KinectSensor::Ptr((KinectSensor*)NULL);
	}

	return KinectSensor::Ptr(new KinectSensor(pNuiSensor, depthEvent2, streamHandle2));
};

std::vector<KinectSensor::Ptr> KinectSensor::getKinects(int numb){ 
	std::vector<KinectSensor::Ptr> r(numb);
	for(int i=0, c=0; c < numb || i < SensorCount(); i++){
		KinectSensor::Ptr a = fromDeviceIndex(i);
		if(a != NULL)
		{
			r[c++] = a;
		}
	}

	return r;
};

bool KinectSensor::setNearMode(bool near2){
	int mode = 0;
	if(near2 == true) mode = NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;

	HRESULT hr = nuiSensor->NuiImageStreamSetImageFrameFlags(streamHandle, mode);
	return SUCCEEDED(hr);
};

