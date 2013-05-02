
#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <boost/thread.hpp> 
#include <boost/array.hpp> 

typedef pcl::PointXYZRGBA DataPoint;
typedef pcl::PointCloud<DataPoint>::ConstPtr DataPointPtr;

const int DEVICE_NUMBER = 2;
boost::array<int, DEVICE_NUMBER> viewports;
boost::array<DataPointPtr, DEVICE_NUMBER> clouds;
boost::array<bool, DEVICE_NUMBER> iniD = {false, false};

class Caller {
    int i;

    void cloud_cb_(const DataPointPtr &cloud) {
        if (i < clouds.size())
            clouds[i] = cloud;
    }
public:

    Caller(int a) : i(a) {
    }

    void registerCallBackTo(pcl::Grabber * interface) {
        boost::function<void (const DataPointPtr&) > f = boost::bind(&Caller::cloud_cb_, this, _1);
        interface->registerCallback(f);
    }
};

class SimpleOpenNIViewer {
public:

    SimpleOpenNIViewer() {
    }

    const char* getDeviceName(int i) {
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
    }

    static void renderData(pcl::visualization::PCLVisualizer& viewer) {
        for (int i = 0; i < viewports.size(); ++i) {
            const DataPointPtr cloud = clouds[i];
            if (cloud.get() != NULL) {
                pcl::visualization::PointCloudColorHandlerRGBField<DataPoint> rgb(cloud);
                if (!iniD[i]) {
                    iniD[i] = true;
                    viewer.addPointCloud<DataPoint> (cloud,  rgb,"sample cloud" + i, viewports[i]);
                } else {
                    viewer.updatePointCloud<DataPoint> (cloud,rgb, "sample cloud" + i);
                }
            }
        }
    }

    static void ini(pcl::visualization::PCLVisualizer& viewer) {
        double split = 1.0 / viewports.size();
        for (int i = 0; i < viewports.size(); ++i) {
            int v(0);
            viewer.createViewPort(split*i, 0.0, split * (i + 1), 1.0, v);
            viewer.setBackgroundColor(0, 0.4, 20, v);
            viewer.addCoordinateSystem(1, v);
            viewports[i] = v;
        }

        viewer.initCameraParameters();
    }

    void run() {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        unsigned int nmb = driver.getNumberDevices();
        if (nmb >= DEVICE_NUMBER) {
            pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer1");
            viewer.runOnVisualizationThreadOnce(ini);
            viewer.runOnVisualizationThread(renderData);

            boost::array<pcl::Grabber*, DEVICE_NUMBER> devices;

            for (int i = 0; i < DEVICE_NUMBER; i++) {
                devices[i] = getDevice(i);
                devices[i]->start();
                Caller(i).registerCallBackTo(devices[i]);
                Caller(i).registerCallBackTo(devices[i]);
                cout << "initialized device " << i << "\n";
            }
            
            //workaround: the last callback which gets registered to any device is never called(pcl bug))
            Caller(0).registerCallBackTo(devices[0]);

            while (!viewer.wasStopped()) {
                sleep(1);
            }

            for (int i = 0; i < DEVICE_NUMBER; i++) {
                devices[i]->stop();
            }
        } else {
            cout << "device count to small";
        }
    }
};

int main() {
    SimpleOpenNIViewer v;
    v.run();
    return 0;
}