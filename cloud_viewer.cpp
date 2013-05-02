
#define WIN32_LEAN_AND_MEAN
#define INC_OLE2

#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <boost/thread.hpp> 
#include <boost/array.hpp> 

#include "NuiApi.h"

#ifdef WIN32 
#include <windows.h>
#define sleep(x) Sleep((x) * 1000) 
#endif 

typedef pcl::PointXYZ DataType;
typedef pcl::PointCloud<DataType> DataPoint;

HRESULT ProcessDepth(DataPoint::Ptr msg, int i);
HRESULT ToggleNearMode(int i);
HRESULT CreateFirstConnected();

//*/ comment-toggle
const int DEVICE_NUMBER = 2;
boost::array<bool, DEVICE_NUMBER> init = {false, false};
boost::array<int, DEVICE_NUMBER> viewports;
boost::array<INuiSensor* , DEVICE_NUMBER> sensors;
boost::array<HANDLE , DEVICE_NUMBER> depthframeevents;
boost::array<HANDLE , DEVICE_NUMBER> streamhandles;
//boost::array<DataPointPtr, DEVICE_NUMBER> clouds;
//
//class Caller {
//    int i;
//
//    void cloud_cb_(const DataPointPtr &cloud) {
//        if (i < clouds.size())
//            clouds[i] = cloud;
//    }
//public:
//
//    Caller(int a) : i(a) {
//    }
//
//    void registerCallBackTo(pcl::Grabber * interface) {
//        boost::function<void (const DataPointPtr&) > f = boost::bind(&Caller::cloud_cb_, this, _1);
//        interface->registerCallback(f);
//    }
//};

class SimpleOpenNIViewer {
public:

    SimpleOpenNIViewer() {
    }

   /* const char* getDeviceName(int i) {
        char* buff = (char *) malloc(20 * sizeof (char));
#ifdef _WIN32
        sprintf(buff, "#%d", i + 1);
#else
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        sprintf(buff, "%d@%d", (int) driver.getBus(i), (int) driver.getAddress(i));
#endif
        return buff;
    }

    pcl::Grabber* getDevice(int i) {
        cout << getDeviceName(i);
        return new pcl::OpenNIGrabber(getDeviceName(i));
    }*/

    static void renderData(pcl::visualization::PCLVisualizer& viewer) {
		for(int i=0; i<DEVICE_NUMBER; ++i)
		if ( WAIT_OBJECT_0 == WaitForSingleObject(depthframeevents[i], 0) )
		{
			DataPoint::Ptr cloud (new DataPoint);
			ProcessDepth(cloud, i);

        //for (int i = 0; i < viewports.size(); ++i) {
                //pcl::visualization::PointCloudColorHandlerRGBField<DataPoint> rgb(cloud);
                if (!init[i]) {
                    init[i] = true;
                    viewer.addPointCloud<DataType> (cloud, "sample cloud"+i, viewports[i]);
                } else {
                    viewer.updatePointCloud<DataType> (cloud, "sample cloud"+i);
                }
        //}
		}
    }

    static void ini(pcl::visualization::PCLVisualizer& viewer) {
        double split = 1.0 / viewports.size();
        for (int i = 0; i < DEVICE_NUMBER; ++i) {
            int v(0);
            viewer.createViewPort(split*i, 0.0, split * (i + 1), 1.0, v);
            viewer.setBackgroundColor(0, 0.4, 20, v);
            viewer.addCoordinateSystem(1, v);
            viewports[i] = v;
        }
        viewer.initCameraParameters();
    }

    void run() {
        /*openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        unsigned int nmb = driver.getNumberDevices();
        if (nmb >= DEVICE_NUMBER) {*/
		/*pcl::visualization::PCLVisualizer& viewer (pcl::visualization::PCLVisualizer ("3D Viewer"));
		ini(viewer);*/
            pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer1");
            viewer.runOnVisualizationThreadOnce(ini);
            viewer.runOnVisualizationThread(renderData);

            //boost::array<pcl::Grabber*, DEVICE_NUMBER> devices;

            //for (int i = 0; i < DEVICE_NUMBER; i++) {
            //    devices[i] = getDevice(i);
            //    devices[i]->start();
            //    Caller(i).registerCallBackTo(devices[i]);
            //    //Caller(i).registerCallBackTo(devices[i]);
            //    cout << "initialized device " << i << "\n";
            //}
            
            //workaround: the last callback which gets registered to any device is never called(pcl bug))
            //Caller(0).registerCallBackTo(devices[0]);

            while (!viewer.wasStopped()) {
                sleep(1);
            }

            /*for (int i = 0; i < DEVICE_NUMBER; i++) {
                devices[i]->stop();
            }*/
        /*} else {
            cout << "device count to small";
        }*/
    }
};

int main() {
	
	if ( FAILED( CreateFirstConnected() ) )
    {
        MessageBox(NULL, "No ready Kinect found!", "Error", MB_ICONHAND | MB_OK);
        return 0;
    }

	SimpleOpenNIViewer v;
    v.run();

    return 0;
}

HRESULT CreateFirstConnected()
{
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr) ) { return hr; }

	int c = 0;
    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            sensors[c++] = pNuiSensor;
            continue;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }
	
    for (int i = 0; i < DEVICE_NUMBER; ++i)
    if (NULL == sensors[i])
    {
        return E_FAIL;
    }
	
    for (int i = 0; i < DEVICE_NUMBER; ++i){
    // Initialize the Kinect and specify that we'll be using depth
    hr =  sensors[i]->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX); 
    if (FAILED(hr) ) { return hr; }

    // Create an event that will be signaled when depth data is available
	depthframeevents[i] = CreateEvent(NULL, TRUE, FALSE, NULL);

    // Open a depth image stream to receive depth frames
    hr =  sensors[i]->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
        NUI_IMAGE_RESOLUTION_640x480,
        0,
        2,
        depthframeevents[i],
		&streamhandles[i]);
    if (FAILED(hr) ) { return hr; }

    // Start with near mode on
	//ToggleNearMode();
	}

    return hr;
}

    bool                                m_bNearMode;
HRESULT ToggleNearMode(int i)
{
    HRESULT hr = E_FAIL;

    if ( sensors[i] )
    {
        hr = sensors[i] ->NuiImageStreamSetImageFrameFlags(streamhandles[i], m_bNearMode ? 0 : NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);

        if ( SUCCEEDED(hr) )
        {
            m_bNearMode = !m_bNearMode;
        }
    }

    return hr;
}

static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;

HRESULT ProcessDepth(DataPoint::Ptr msg, int i)
{
	DWORD width = 0;
    DWORD height = 0;

    NuiImageResolutionToSize(cDepthResolution, width, height);
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
	
	 //msg (new DataPoint);
	 msg->header.frame_id = "some_tf_frame";
	 msg->height = height;
	 msg->width = width;
	 msg->is_dense = true;
	 msg->points.resize(width*height);

	 HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
    hr = sensors[i] ->NuiImageStreamGetNextFrame(streamhandles[i], 0, &imageFrame);
    if (FAILED(hr))
    {
        return hr;
    }

    BOOL nearMode;
    INuiFrameTexture* pTexture;

    // Get the depth image pixel texture
    hr = sensors[i]->NuiImageFrameGetDepthImagePixelFrameTexture(
        streamhandles[i], &imageFrame, &nearMode, &pTexture);
    if (FAILED(hr))
    {
        goto ReleaseFrame;
    }

    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
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
				float realDepth = pBufferRun[x+y*width].depth / 1000.0;
				//(x - width/2)*m_xyScale * realDepth

				pcl::PointXYZ result;
				result.x = (x - width/2)*m_xyScale * realDepth;
				result.y = (y - height/2)*(-m_xyScale) * realDepth;
				result.z = realDepth;
				msg->points.push_back(result);
			}

	}
    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    pTexture->Release();

ReleaseFrame:
    // Release the frame
    sensors[i]->NuiImageStreamReleaseFrame(streamhandles[i], &imageFrame);

    return hr;
}