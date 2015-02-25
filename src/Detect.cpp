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


#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>


//TODO remove this when not needed
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Detect::startDetection(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		
	    //for filtering normal
	    pcl::PointCloud<pcl::Normal>::Ptr cloudNormalFiltered (new pcl::PointCloud<pcl::Normal>);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    //for segmentation
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::ExtractIndices<pcl::PointXYZRGBA> extract; 
	    
	    //filter out points with NaN (invalid) values
	    std::vector<int> indices;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr noNANCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::removeNaNFromPointCloud(*cloud, *noNANCloud, indices);
		
	    
	    //-----------compute normals------------------//
	     
	    // Create the normal estimation class, and pass the input dataset to it
	    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	    ne.setInputCloud (noNANCloud);
		// Create an empty kdtree representation, and pass it to the normal estimation object. Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	    ne.setSearchMethod (tree);
		// Output datasets
	    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
	    // Use all neighbors in a sphere of radius 3cm
	    ne.setRadiusSearch (0.01); //TODO put into constants
		// Compute the features
	    ne.compute (*cloudNormals);
	    

		//----nearest neighbours----//
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud (noNANCloud);
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		float radius = 0.03 ;//TODO to constants
		// temporary cloud
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::PointCloud<pcl::Normal>::iterator itNorm = cloudNormals->begin();
		for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it = noNANCloud->begin(); it!= noNANCloud->end(); it++){
			pcl::PointXYZRGBA point ;
			point.x = it->x;
			point.y = it->y;
			point.z = it->z;
			point.rgba = it->rgba;
			
			int r = 255;
			int g = 0;
			int b = 0;			
			int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
			
			//find nearest neighbours
			if ( kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
			{	
				//count of neighbouring normals that don't point in same direction as point 
				int numNormalDiff = 0;
				//iterate over all neighbours of point
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
					
					//get the dot product of normals of point with its neighbour
					float dotProduct = ( cloudNormals->points[pointIdxRadiusSearch[i]].normal_x * itNorm->normal_x ) + ( cloudNormals->points[pointIdxRadiusSearch[i]].normal_y * itNorm->normal_y ) + ( cloudNormals->points[pointIdxRadiusSearch[i]].normal_z * itNorm->normal_z ) ;
					float dotThresh = 0.96; //TODO to constants
					//if angle between normals is more than threshhold
					if(dotProduct < dotThresh ){
						numNormalDiff++;
					}
					
				}
				//sufficiant number of normals indicating change in direction
				float percentSupport = 0.2; //TODO to constants
				if( numNormalDiff > (percentSupport * pointIdxRadiusSearch.size()) ){
					point.rgba = rgb;		
				}	
			}	    	
							
		tempCloud->push_back(point);
		itNorm++;
		}
		noNANCloud->clear();
		noNANCloud->swap(*tempCloud);
		tempCloud->clear();
		
	    //-----filter out the normals from the ground, ceilling,etc-----//
	    pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator itCloud = noNANCloud->begin();
	    for(pcl::PointCloud<pcl::Normal>::iterator it = cloudNormals->begin(); it!= cloudNormals->end(); it++){
			
			//check if normal is nearly parallel to z axis ie dot product is > cos(theta)
			float dotProductZ = it->normal_x*0 + it->normal_y*0 + it->normal_z *1;
			float angle = 0.98; //TODO put to constants
			
			//change sign as we only need the magnitude here
			if (dotProductZ < 0)
			{
				dotProductZ = -dotProductZ;
			}
			if(dotProductZ > angle){	 
				pcl::Normal norm (it->normal_x,it->normal_y,it->normal_z);
				cloudNormalFiltered->push_back(norm);

				pcl::PointXYZRGBA point ;
				point.x = itCloud->x;
				point.y = itCloud->y;
				point.z = itCloud->z;
				point.rgba = itCloud->rgba;
				cloudFiltered->push_back(point);				 	
				
			}
			itCloud++;
			
		}
		
		//--------------do segmentation------------//
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.03);

		seg.setInputCloud (cloudFiltered);
		seg.segment (*inliers, *coefficients);

		//int min_points_of_plane = 100;
		//if (inliers->indices.size () < min_points_of_plane)  {
		
		// Extract the inliers
		extract.setInputCloud (cloudFiltered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		// Get the points associated with the planar surface
		extract.filter (*cloudPlane);
		
		// Extract points above plane, remove the planar inliers, extract the rest
		//extract.setNegative (true);
		//extract.filter (*cloudOnplane);
		



	    
	    // uncomment this for normal visuallization, and remove visualizer from doorHandle.cpp or use doorHandle.cpp_normalVisualisation
		// --------------------------------------------------------
	    // -----Open 3D viewer and add point cloud and normals-----
	    // --------------------------------------------------------
	    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	    viewer->setBackgroundColor (0, 0, 0);
	    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(noNANCloud);
	    viewer->addPointCloud<pcl::PointXYZRGBA> (noNANCloud, rgb, "sample cloud");
	    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	    viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (noNANCloud, cloudNormals, 10, 0.05, "normals");
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
