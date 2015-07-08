/* 
 * File:   Node.h
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include <log4cxx/logger.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Edge.h"

using namespace pcl;

class Node {
public:
    Node() {
	}
    virtual ~Node() {
	}

    long id;
    std::vector<Edge> edges;
    std::vector<int> neighbours; // is it needed ?????
    pcl::PointXYZRGBA point;
    double error;
	log4cxx::LoggerPtr logger;

};
