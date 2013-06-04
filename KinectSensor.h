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
	INuiSensor * nuiSensor;
	HANDLE depthEvent, streamHandle;

	KinectSensor(INuiSensor*, HANDLE, HANDLE);

public:
	typedef boost::shared_ptr<KinectSensor> Ptr;
	typedef pcl::PointXYZ Data;
	typedef pcl::PointCloud<Data> Cloud;

	~KinectSensor(void);

	static int SensorCount();

	static KinectSensor::Ptr fromDeviceIndex(int);

	static std::vector<KinectSensor::Ptr> getKinects(int);

	bool setNearMode(bool);

	//template<typename T> 
	Cloud::Ptr getNextPointCloud(){		
		Cloud::Ptr cloud (new Cloud);

		if ( WAIT_OBJECT_0 == WaitForSingleObject(depthEvent, 0) )
		{
			DWORD width = 0;
			DWORD height = 0;

			NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);
			LONG m_depthWidth  = static_cast<LONG>(width);
			// Calculate correct XY scaling factor so that our vertices are correctly placed in the world
			// This helps us to unproject from the Kinect's depth camera back to a 3d world
			// Since the Horizontal and Vertical FOVs are proportional with the sensor's resolution along those axes
			// We only need to do this for horizontal
			// I.e. tan(horizontalFOV)/depthWidth == tan(verticalFOV)/depthHeight
			// Essentially we're computing the vector that light comes in on for a given pixel on the depth camera
			// We can then scale our x&y depth position by this and the depth to get how far along that vector we are
			const float DegreesToRadians = 3.14159265359f / 180.0f;
			float m_xyScale = tanf(NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * DegreesToRadians * 0.5f) / (m_depthWidth * 0.5f);  

			cloud->header.frame_id = "some_tf_frame";
			cloud->height = height;
			cloud->width = width;
			cloud->is_dense = false;
			cloud->points.resize(width*height);

			NUI_IMAGE_FRAME imageFrame;

			// Attempt to get the depth frame
			HRESULT hr = nuiSensor->NuiImageStreamGetNextFrame(streamHandle, 0, &imageFrame);
			if (FAILED(hr))
			{
				goto ReleaseFrame;
			}

			BOOL nearMode;
			INuiFrameTexture* pTexture;

			// Get the depth image pixel texture
			hr = nuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
				streamHandle, &imageFrame, &nearMode, &pTexture);
			if (FAILED(hr))
			{
				goto ReleaseFrame;
			}

			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			NUI_LOCKED_RECT LockedRect;
			pTexture->LockRect(0, &LockedRect, NULL, 0);

			// Make sure we've received valid data
			if (LockedRect.Pitch != 0)
			{
				// Get the min and max reliable depth for the current frame
				int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
				int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

				const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

				for(int x = 0; x < width; ++x)
					for(int y = 0; y < height; ++y){
						int id = x+y*width;
						float realDepth = pBufferRun[id].depth / 1000.0;
						//(x - width/2)*m_xyScale * realDepth
						pcl::PointXYZ result;
						if(pBufferRun[id].depth >= 0)
						{
							result.x = (x - width/2)*m_xyScale * realDepth;
							result.y = (y - height/2)*(-m_xyScale) * realDepth;
							result.z = realDepth;
						}else
						{
							result.x = std::numeric_limits<float>::quiet_NaN();
							result.y = std::numeric_limits<float>::quiet_NaN();
							result.z = std::numeric_limits<float>::quiet_NaN();
						}
						cloud->points[id] = result;
					}

			}
			// We're done with the texture so unlock it
			pTexture->UnlockRect(0);

			pTexture->Release();

ReleaseFrame:
			// Release the frame
			nuiSensor->NuiImageStreamReleaseFrame(streamHandle, &imageFrame);
		}
		return cloud;
	};
};

