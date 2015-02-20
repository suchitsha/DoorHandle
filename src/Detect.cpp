/* 
 * File:   Detect.cpp
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include "Detect.h"
//PCL headers
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//todo
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Detect::startDetection(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	    
	    //TODO remove this if not used
    	//create and fill input from XYZRGBA to xyz cloud 
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    *cloudXYZRGBA = *cloud;
	    
	    //-----------compute normals------------------//
	     
	    // Create the normal estimation class, and pass the input dataset to it
	    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	    ne.setInputCloud (cloud);
		// Create an empty kdtree representation, and pass it to the normal estimation object. Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	    ne.setSearchMethod (tree);
		// Output datasets
	    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	    // Use all neighbors in a sphere of radius 3cm
	    ne.setRadiusSearch (0.03);
		// Compute the features
	    ne.compute (*cloud_normals);
	    
	    
	    //-----filter out the normals from the ground, ceilling,etc-----//
	    
	    for(pcl::PointCloud<pcl::Normal>::iterator it = cloud_normals->begin(); it!= cloud_normals->end(); it++){
        cout << "a" << it << endl;
    	}
	    
	    // uncomment this for normal visuallization, and remove visualizer from doorHandle.cpp or use doorHandle.cpp_normalVisualisation
		// --------------------------------------------------------
	    // -----Open 3D viewer and add point cloud and normals-----
	    // --------------------------------------------------------
	    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	    viewer->setBackgroundColor (0, 0, 0);
	    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
	    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	    viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
	    viewer->addCoordinateSystem (0.2);
	    viewer->initCameraParameters ();
		
		while (!viewer->wasStopped ())
		{
		    viewer->spinOnce (1000);
		    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  		}
		//*/		
	    return cloud;
}
