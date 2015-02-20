/* 
 * File:   Detect.h
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include <log4cxx/logger.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//todo
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

class Detect {
public:
	Detect() {
	}
	virtual ~Detect() {
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr startDetection(
			const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		
	log4cxx::LoggerPtr logger;

};
