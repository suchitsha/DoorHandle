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
#include <pcl/io/pcd_grabber.h>

#include "Detect.h"

class DoorHandle {
public:
	DoorHandle() :
			viewer("PCL OpenNI Viewer") {
	}

	pcl::visualization::CloudViewer viewer;
	Detect detect;
	//callback function
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		if (!viewer.wasStopped()) {
			pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr outCloud;
			outCloud = detect.startDetection(cloud);
				viewer.showCloud(outCloud);
		}
	}

	void run() {
		//read file
		// for oni file uncomment code below
		pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured.oni",
		//pcl::Grabber* interface = new pcl::ONIGrabber("../data/Captured_drawer.oni",
				false, false);
		
		/*
		//for pcd file uncomment code below
		boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > interface;
		interface.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> ("../data/other/kitchen/Rf3.pcd", 1, true)); 			
		*/
		
		
		
		//register callback
		boost::function<
				void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
				boost::bind(&DoorHandle::cloud_cb_, this, _1);

		interface->registerCallback(f);
		interface->start();

		//start viewer
		while (!viewer.wasStopped()) {
			interface->start();
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}
		PCL_INFO("Completed reading file.\n");
		interface->stop();
	}
};

int main() {
	DoorHandle dh;
	dh.run();
	return 0;
}
