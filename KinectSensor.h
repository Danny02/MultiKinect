#pragma once
#include "stdafx.h"
#include <iostream>
#include "NuiApi.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <limits>

class KinectSensor
{
private:
	static const NUI_IMAGE_RESOLUTION RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	static const int WIDTH = 640;
	static const int HEIGHT = 480;
	static const int AREA = WIDTH*HEIGHT;

	static const float DegreesToRadians, m_xyScale, NAN;  

	BYTE* colorFrame;
	USHORT* depthFrame;
	LONG* colorCoordinates;

	INuiSensor * nuiSensor;
	HANDLE depthEvent, depthStreamHandle, colorEvent, colorStreamHandle;

	KinectSensor(INuiSensor*, HANDLE, HANDLE, HANDLE, HANDLE);

	template<class A>
    HRESULT getFrameData(A* data, HANDLE stream);

	template<class PointT> 
	PointT calcPos(int x, int y, USHORT raw);

	float calcDepth(USHORT raw);

public:
	typedef boost::shared_ptr<KinectSensor> Ptr;

	~KinectSensor(void);

	static int SensorCount();

	static Ptr fromDeviceIndex(int);

	static std::vector<Ptr> getKinects(int);

	bool setNearMode(bool);

	HRESULT setLaser(bool);

	pcl::PointCloud<pcl::PointXYZ>::Ptr getNextDepthPointCloud();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNextColorPointCloud();
};

