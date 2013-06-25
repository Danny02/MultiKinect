#include "KinectSensor.h"

const float KinectSensor::NAN = std::numeric_limits<float>::quiet_NaN();
const float KinectSensor::DegreesToRadians = 3.14159265359f / 180.0f;
const float KinectSensor::m_xyScale = tanf(NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * DegreesToRadians * 0.5f) / (WIDTH * 0.5f);  

KinectSensor::KinectSensor(INuiSensor * sens, HANDLE depth, HANDLE dstream, HANDLE color, HANDLE cstream) 
	: nuiSensor(sens), depthEvent(depth), depthStreamHandle(dstream),
	colorEvent(color), colorStreamHandle(cstream), colorFrame(new BYTE[AREA*4]), depthFrame(new USHORT[AREA]), colorCoordinates(new LONG[AREA*2])
{};

KinectSensor::~KinectSensor(void){
	nuiSensor->Release();

	delete[] colorFrame;
	delete[] depthFrame;
	delete[] colorCoordinates;
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
		fail |= pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH) < 0; 

	if(!fail)
		fail |= !(pNuiSensor->NuiStatus() == 0);

	HANDLE depthEvent2, depthStreamHandle2, colorEvent2, colorStreamHandle2;
	if(!fail){
		// Create an event that will be signaled when depth data is available
		depthEvent2 = CreateEvent(NULL, TRUE, FALSE, NULL);

		// Open a depth image stream to receive depth frames
		fail |= pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH,
			RESOLUTION,
			0,
			2,
			depthEvent2,
			&depthStreamHandle2);
		
		colorEvent2 = CreateEvent(NULL, TRUE, FALSE, NULL);
		fail |= pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			RESOLUTION,
			0,
			2,
			colorEvent2,
			&colorStreamHandle2);

		if(fail){
			pNuiSensor->Release();
		}else{
			return KinectSensor::Ptr(new KinectSensor(pNuiSensor, depthEvent2, depthStreamHandle2, colorEvent2, colorStreamHandle2));
		}
	}

	return KinectSensor::Ptr((KinectSensor*)NULL);
};

std::vector<KinectSensor::Ptr> KinectSensor::getKinects(int numb){ 
	std::vector<KinectSensor::Ptr> r(numb);
	int max = SensorCount();
	for(int i=0, c=0; c < numb && i < max; i++){
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

	HRESULT hr = nuiSensor->NuiImageStreamSetImageFrameFlags(depthStreamHandle, mode);
	return SUCCEEDED(hr);
};

HRESULT KinectSensor::setLaser(bool on){
	return nuiSensor->NuiSetForceInfraredEmitterOff(!on);
}

template<class PointT> PointT KinectSensor::calcPos(int x, int y, USHORT raw){
	// Calculate correct XY scaling factor so that our vertices are correctly placed in the world
	// This helps us to unproject from the Kinect's depth camera back to a 3d world
	// Since the Horizontal and Vertical FOVs are proportional with the sensor's resolution along those axes
	// We only need to do this for horizontal
	// I.e. tan(horizontalFOV)/depthWidth == tan(verticalFOV)/depthHeight
	// Essentially we're computing the vector that light comes in on for a given pixel on the depth camera
	// We can then scale our x&y depth position by this and the depth to get how far along that vector we are

	float realDepth = raw > 0 ? raw / 1000.0f : NAN;

	PointT result;
	result.x = (x - WIDTH/2)*m_xyScale * realDepth;
	result.y = (y - HEIGHT/2)*(-m_xyScale) * realDepth;
	result.z = realDepth;

	return result;
}

template<class A>
HRESULT KinectSensor::getFrameData(A* data, HANDLE stream)
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = nuiSensor->NuiImageStreamGetNextFrame(stream, 0, &imageFrame);
	if ( FAILED(hr) ) { return hr; }

	NUI_LOCKED_RECT LockedRect;
	hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( FAILED(hr) ) { return hr; }

	memcpy(data, LockedRect.pBits, LockedRect.size);

	hr = imageFrame.pFrameTexture->UnlockRect(0);
	if ( FAILED(hr) ) { return hr; };

	hr = nuiSensor->NuiImageStreamReleaseFrame(stream, &imageFrame);
	return hr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KinectSensor::getNextDepthPointCloud(){		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if ( WAIT_OBJECT_0 == WaitForSingleObject(depthEvent, 0) )
	{		
		HRESULT hr = getFrameData(depthFrame, depthStreamHandle);

		if( SUCCEEDED(hr) ) 
		{
			cloud->header.frame_id = "some_tf_frame";
			cloud->height = HEIGHT;
			cloud->width = WIDTH;
			cloud->is_dense = true;
			cloud->points.resize(AREA);

			for(int x = 0; x < WIDTH; ++x)
				for(int y = 0; y < HEIGHT; ++y){
					int id = x+y*WIDTH;
					cloud->points[id] = calcPos<pcl::PointXYZ>(x, y, depthFrame[id]);
				}
		}
	}
	return cloud;
};

#include <strsafe.h>
HRESULT GetScreenshotFileName(wchar_t *screenshotName, UINT screenshotNameSize)
{
    wchar_t *knownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &knownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        wchar_t timeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", timeString, _countof(timeString));

        // File name will be KinectSnapshot-HH-MM-SS.wav
        StringCchPrintfW(screenshotName, screenshotNameSize, L"%s\\KinectSnapshot-%s.bmp", knownPath, timeString);
    }

    CoTaskMemFree(knownPath);
    return hr;
}

#include <d2d1.h>

HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if ( !WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if ( !WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if ( !WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectSensor::getNextColorPointCloud(){	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	if ( WAIT_OBJECT_0 == WaitForSingleObject(colorEvent, 0)){
		HRESULT hr1 = getFrameData(colorFrame, colorStreamHandle);
		
        /*WCHAR screenshotPath[MAX_PATH];
        GetScreenshotFileName(screenshotPath, _countof(screenshotPath));
		SaveBitmapToFile(colorFrame, 640, 480, 32, screenshotPath);*/

		if(WAIT_OBJECT_0 == WaitForSingleObject(depthEvent, 0) ){
			HRESULT hr2 = getFrameData(depthFrame, depthStreamHandle);

			if( SUCCEEDED(hr1) && SUCCEEDED(hr2) ) {
				// Get of x, y coordinates for color in depth space
				// This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
				nuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
					RESOLUTION,
					RESOLUTION,
					AREA,
					depthFrame,
					AREA*2,
					colorCoordinates
					);

				cloud->header.frame_id = "some_tf_frame";
				cloud->height = HEIGHT;
				cloud->width = WIDTH;
				cloud->is_dense = true;
				cloud->points.resize(AREA);

				for(int x = 0; x < WIDTH; ++x)
					for(int y = 0; y < HEIGHT; ++y){
						int id = x+y*WIDTH;					
						pcl::PointXYZRGB point = calcPos<pcl::PointXYZRGB>(x, y, depthFrame[id]);

						if(point.z != point.z) continue;

						LONG colorInDepthX = colorCoordinates[id * 2];
						LONG colorInDepthY = colorCoordinates[id * 2 + 1];
						// make sure the depth pixel maps to a valid point in color space
						if ( colorInDepthX >= 0 && colorInDepthX < WIDTH && colorInDepthY >= 0 && colorInDepthY < HEIGHT )
						{
							//RGBx format, RGB + padding byte
							LONG colorIndex = (colorInDepthX + colorInDepthY * WIDTH) * 4;
							point.r = colorFrame[colorIndex];
							point.g = colorFrame[colorIndex + 1];
							point.b = colorFrame[colorIndex + 2];
						}

						cloud->points[id] = point;
					}
			}
		}
	}

	return cloud;
};

