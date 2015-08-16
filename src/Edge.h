/* 
 * File:   Edge.h
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include <log4cxx/logger.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class Edge {
public:
    Edge() {
	}
    virtual ~Edge() {
	}

    int from;
    int to;
    long age;

	log4cxx::LoggerPtr logger;

};
