/* 
 * File:   DoorHandle.cpp
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>

#include "Detect.h"

class DoorHandle {
public:
	/*DoorHandle() :
			viewer("PCL OpenNI Viewer") {
	}*/
	DoorHandle() {
	}

	/*pcl::visualization::CloudViewer viewer;*/
	Detect detect;
	//callback function
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		/*if (!viewer.wasStopped()) {*/
			detect.startDetection(cloud);
			//viewer.showCloud(cloud);
		/*}*/
	}

	void run() {
		//read file
		//pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured_drawers_all.oni",
		//pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured_drawer.oni",
		//pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured_door3.oni",
		pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured.oni",
				false, false);
		//register callback
		boost::function<
				void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
				boost::bind(&DoorHandle::cloud_cb_, this, _1);

		interface->registerCallback(f);
		interface->start();

		while(1){interface->start();}
		/*//start viewer
		while (!viewer.wasStopped()) {
			interface->start();
			//boost::this_thread::sleep (boost::posix_time::seconds (1));
		}
		*/
		PCL_INFO("Completed reading file.\n");
		interface->stop();
	}
};

int main() {
	DoorHandle dh;
	dh.run();
	return 0;
}
