/* 
 * File:   GNG.h
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include <log4cxx/logger.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//TODO
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

class GNG {
public:
    GNG() {
        //GNG constants

        lambda = 100;
	}
    virtual ~GNG() {
	}

    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr startGNG(
			const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	
    int eb;
    int en;
    int lambda;
	log4cxx::LoggerPtr logger;

};
