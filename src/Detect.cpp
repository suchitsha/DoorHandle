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

//region growing
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
//outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
//smoothing
#include <pcl/filters/bilateral.h>
//mls
//#include <pcl/surface/mls.h>


//TODO remove this when not needed
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Detect::startDetection(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		
	    //for filtering normal
	    //pcl::PointCloud<pcl::Normal>::Ptr cloudNormalFiltered (new pcl::PointCloud<pcl::Normal>);
	    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    //for segmentation
	    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    //pcl::ExtractIndices<pcl::PointXYZRGBA> extract; 
	    
	    	    
	    // Outlier removal for filtering noise 
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloudTemp);
	    
	    /*//moving least square
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr mlsTree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA> mlsPoints;
		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointNormal> mls;
		mls.setComputeNormals(false);
		mls.setInputCloud (cloudTemp);
		mls.setPolynomialFit (true);
		mls.setSearchMethod (mlsTree);
		mls.setSearchRadius (0.03);
		mls.process (mlsPoints);
		*/
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudTemp2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::BilateralFilter<pcl::PointXYZRGBA> bFilter;
		bFilter.setInputCloud(cloudTemp);
		bFilter.setHalfSize(5.0f);
		bFilter.setStdDev(0.2f);
		bFilter.applyFilter(*cloudTemp2); 
		
	    //filter out points with NaN (invalid) values
	    std::vector<int> indices;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr noNANCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::removeNaNFromPointCloud(*cloudTemp2, *noNANCloud, indices);
		cloudTemp->clear();
		cloudTemp2->clear();
		
	    
	    /*//-----------compute normals------------------//	     
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
	    */
	    
	    //----------region growing for normals------------//
	    pcl::search::Search<pcl::PointXYZRGBA>::Ptr searchTree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
  		normal_estimator.setSearchMethod (searchTree);
 		normal_estimator.setInputCloud (noNANCloud);
		normal_estimator.setKSearch (50);
		normal_estimator.compute (*normals);
	    
	    /*pcl::IndicesPtr indicesNormal (new std::vector <int>);
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		pass.setInputCloud (noNANCloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 1.0);
		pass.filter (*indicesNormal);
		*/
		
		pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
		reg.setMinClusterSize (50);
		reg.setMaxClusterSize (1000000);
		reg.setSearchMethod (searchTree);
		reg.setNumberOfNeighbours (30);
		reg.setInputCloud (noNANCloud);
		//reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold (1.0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

	    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr coloredCloud = reg.getColoredCloudRGBA();
	    
		std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
		std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
		std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
		int counter = 0;
		while (counter < clusters[0].indices.size ())
		{
			std::cout << clusters[0].indices[counter] << ", ";
			counter++;
			if (counter % 10 == 0)
			std::cout << std::endl;
		}
		std::cout << std::endl;

		/*//----nearest neighbours----//
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud (noNANCloud);
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		float radius = 0.03 ;//TODO to constants
		// temporary cloud
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		//traverse over all points
		pcl::PointCloud<pcl::Normal>::iterator itNorm = cloudNormals->begin();
		for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it = noNANCloud->begin(); it!= noNANCloud->end(); it++){
			pcl::PointXYZRGBA point ;
			point.x = it->x;
			point.y = it->y;
			point.z = it->z;
			point.rgba = it->rgba;
			
			int r1 = 255;
			int r2 = 0;
			int g1 = 0;
			int g2 = 127;
			int b = 0;			
			int rgb1 = ((int)r1) << 16 | ((int)g1) << 8 | ((int)b);
			int rgb2 = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b);			

			//find nearest neighbours in a radius
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
					point.rgba = rgb1;		
				}	
			}	    	
		
		// find K nearest neighbor
		int K = 50;//TODO to constants
		float distThresh = 0.06; //TODO to constants

		std::vector<int> pointIdxNKNSearch(K);
  		std::vector<float> pointNKNSquaredDistance(K);
	
		if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
		    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
				if(pointNKNSquaredDistance[i] > distThresh){
					//point.rgba = rgb2;
				}
			}
  		}

					
		tempCloud->push_back(point);
		itNorm++;
		}
		//clear unused
		noNANCloud->clear();
		noNANCloud->swap(*tempCloud);
		tempCloud->clear();
		
		*/
		
	    /*//-----filter out the normals from the ground, ceilling,etc-----//
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
		*/
		
		/*
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
		*/



	    
	    /*// uncomment this for normal visuallization, and remove visualizer from doorHandle.cpp or use doorHandle.cpp_normalVisualisation
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
		*/		
	    return coloredCloud;
}
