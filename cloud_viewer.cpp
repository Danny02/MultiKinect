
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "stdafx.h"
#include <iostream>
#include <vector>
#include <string>

#include <boost/thread.hpp> 
#include <boost/array.hpp> 
#include <boost/format.hpp> 

#include "KinectSensor.h"

#define sleep(x) Sleep((x) * 1000) 

//*/ comment-toggle
const int DEVICE_NUMBER = 2;
std::vector<KinectSensor::Ptr> sensors;
boost::array<bool, DEVICE_NUMBER> init = {false, false};
boost::array<int, 1> viewports;

bool onlyOne = true;
bool was[] = {false,false};

class SimpleKinectViewer {
public:

	SimpleKinectViewer() {
	}

	static void renderData(pcl::visualization::PCLVisualizer& viewer) {
		if(!was[0] || !was[1]){
			for(int i=0; i<DEVICE_NUMBER; ++i){
				KinectSensor::Cloud::Ptr cloud = sensors[i]->getNextPointCloud();
				cout << cloud->points.size() << "  " << i << '\n';

				pcl::visualization::PointCloudColorHandlerCustom<KinectSensor::Data> single_color(cloud, i==0?255:0, i==1?255:0, 0);
				if (!init[i]) {
					init[i] = true;
					std::string name = (boost::format("sample cloud %1%") % i).str();
					viewer.addPointCloud<pcl::PointXYZ> (cloud, single_color, name, viewports[0]);
				} else {
					//viewer.updatePointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+i);
				}
				if(onlyOne && !was[i] && cloud->points.size() > 0){
					was[i]=true;
					std::string name = (boost::format("kinect%1%.pcd") % i).str();
					pcl::io::savePCDFileBinary(name , *cloud.get());
						viewer.updatePointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+i);
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
			viewer.initCameraParameters();
			viewer.setCameraClipDistances(-1000000.001,1000000,v);
		}
		viewer.initCameraParameters();
	}

	void run() {
		pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer1");
		viewer.runOnVisualizationThreadOnce(ini);
		viewer.runOnVisualizationThread(renderData);

		while (!viewer.wasStopped()) {
			sleep(1);
		}
	}
};

int main() {
	sensors =  KinectSensor::getKinects(DEVICE_NUMBER);
	
	SimpleKinectViewer v;
	v.run();

	return 0;
}
