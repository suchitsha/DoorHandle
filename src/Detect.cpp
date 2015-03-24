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
//mls
#include <pcl/surface/mls.h>
// filter
#include <pcl/filters/voxel_grid.h>


//#include <pcl/features/moment_of_inertia_estimation.h>
//#include <boost/thread/thread.hpp>

//TODO remove this when not needed
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Detect::startDetection(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		// coordinates of y axis
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
		 
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
	    
	    /*//mls
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        // Output has the same type as the input one, it will be only smoothed
        pcl::PointCloud<pcl::PointXYZRGBA> mls_points;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointNormal> mls;
               
         // Optionally, a pointer to a cloud can be provided, to be set by MLS
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals (new pcl::PointCloud<pcl::PointNormal> ());
        mls.setOutputNormals (mls_normals);
        cout<<"SUPERMUESTRO INICIADA..."<<endl;
        // Set parameters
		       
        mls.setInputCloud (cloudTemp);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGBA,pcl::PointNormal>::VOXEL_GRID_DILATION);
        mls.setDilationVoxelSize(0.002);
        mls.setPolynomialOrder(4);
        // Reconstruct
        mls.reconstruct (mls_points); 
        */
        
        
		// Create the filtering object
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudTemp2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
		vox.setInputCloud (cloudTemp);
		vox.setLeafSize (0.02f, 0.02f, 0.02f);//TODO check whats better
		vox.filter (*cloudTemp2);

		
		
	    //-------------filter out points with NaN (invalid) values--------------//
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

		std::vector<pcl::PointIndices> clusters;
		reg.extract (clusters);
		
		//uncomment to see coloured segments
		//pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr coloredCloud = reg.getColoredCloudRGBA();
   		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr coloredCloud;
   		
		//std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
		//std::cout << "Third cluster has " << clusters[2].indices.size () << " points." << endl;
		//std::cout << "width and height are:" << coloredCloud->width << "@" << coloredCloud->height << "@" << coloredCloud->isOrganized() << "@" << std::endl;
		
		//iterate over clusters
		std::vector<std::vector<float> > boundaries;
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments ;//(new std::vector<pcl::PointCloud<pcl::PointXYZRGBA> >);
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentsOutliers ;
		for(int i=0; i<clusters.size(); i++)
		{	
			//remove clusters with few points
			if(clusters[i].indices.size() > 100) //TODO choose correct value
			{
				std::cout << "size of:" << i <<":" << clusters[i].indices.size();
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segCloud (new pcl::PointCloud <pcl::PointXYZRGBA>);
				//get points in one segment into a cloud
				for(int j=0; j<clusters[i].indices.size(); j++)
				{
					pcl::PointXYZRGBA p = noNANCloud->at(clusters[i].indices[j]);
					segCloud->push_back(p);
				}
				
				//fit plane
				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (SACMODEL_PARALLEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				//TODO try out these parameters
				seg.setDistanceThreshold (0.01);
				seg.setEpsAngle( 30.0f * (M_PI/180.0f) );
				seg.setAxis(axis);
				seg.setInputCloud (segCloud);
				seg.segment (*inliers, *coefficients);

				if (inliers->indices.size() == 0)
				{
					//TODO remove the cloud that does not fit
					std::cout << "Could not estimate a planar model for the given dataset." << std::endl ;
				}else 
				{
					//ignore too small clouds
					if(inliers->indices.size() > 100) //TODO choose correct value
					{
						std::cout << "Estimated a planar model." << std::endl;
					
						//max and min x and y for each cluster
						float maxX;
						float maxY;
						float minX;
						float minY;
						float avgZ;
						for(int j=0; j < inliers->indices.size(); j++)
						{
							pcl::PointXYZRGBA p = noNANCloud->at(inliers->indices[j]);
							if(j == 0)
							{
								maxX = p.x;
								maxY = p.y;
								minX = p.x;
								minY = p.y;
							}
							if(p.x > maxX)
							{
								maxX = p.x;
							}
							if(p.x < minX)
							{
								minX = p.x;
							}
							if(p.y > maxY)
							{
								maxY = p.y;
							}
							if(p.y < minY)
							{
								minY = p.y;
							}
							avgZ = avgZ + p.z;
						}
						if(clusters[i].indices.size() != 0)
						{
							avgZ = avgZ/clusters[i].indices.size();
						}
						std::cout << "for cluster " << i << " max min values are " << maxX << " "<< minX << " "<< maxY << " "<< minY << " " << avgZ << std::endl;
						std::vector<float> tempBound;
						tempBound.push_back(maxX);
						tempBound.push_back(maxY);
						tempBound.push_back(minX);
						tempBound.push_back(minY);
						boundaries.push_back(tempBound);
					
						// Extract the inliers
						pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempPlane (new pcl::PointCloud<pcl::PointXYZRGBA>);
						pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempOnPlane (new pcl::PointCloud<pcl::PointXYZRGBA>);
						pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
						extract.setInputCloud (segCloud);
						extract.setIndices (inliers);
						extract.setNegative (false);
						extract.filter (*tempPlane);
						
						//get outliers 
						extract.setNegative (true);
					    extract.filter (*tempOnPlane);
    
						// as plane satisfies boundary conditions add plane
						segments.push_back(tempPlane);
						//outliers
						segmentsOutliers.push_back(tempOnPlane);
					}
				}
			}
		}

		//check for background planes
		float percentOfSize = 0.10; //TODO to constants
		std::vector<std::vector<float> > inboundaries;
		std::vector<int> ignoreList;
		for(int l=0; l < boundaries.size(); l++)
		{	
			for(int m=0; m < boundaries.size(); m++)
			{	
				if(l != m)
				{	
					if(boundaries[l].at(0) >= boundaries[m].at(0) )
					{
						if(boundaries[l].at(1) >= boundaries[m].at(1) )
						{
							if(boundaries[l].at(2) <= boundaries[m].at(2) )
							{
								if(boundaries[l].at(3) <= boundaries[m].at(3) )
								{	
									//if plane inside is large enough, this is to eliminate wrong values due to noise or small structures in a plane
									if( percentOfSize*segments[l]->size() < segments[m]->size() )
									{	
										std::cout << "adding to ignoreList: " << l  << " of size: " << segments[l]->size() << " compared: " << m << " " << segments[m]->size() << std::endl;
										ignoreList.push_back(l);
									}
								}							
							}	
						}							
					}
				}
			}
		}
		//TODO
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for(int i=0; i < segments.size(); i++)
		{
			if( std::find(ignoreList.begin(), ignoreList.end(), i) != ignoreList.end() ) 
			{
				// ignoreList contains index so ignore 
			} else 
			{
				// ignoreList does not contain index
				std::cout << "showing cloud: " << i << " of size: " << segments[i]->size() << std::endl;	
				*outliers = *segmentsOutliers[i];
				if (this->o.size() < 15)
				{
					this->o.push_back(segmentsOutliers[i]);
				} else 
				{
					this->o.push_back(segmentsOutliers[i]);
					//o.pop_front(); //TODO
				}
				
				
				*planes += *segments[i];
			
		/*
				//bounding box
				pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
				feature_extractor.setInputCloud (segments[i]);
				feature_extractor.compute();

				std::vector <float> moment_of_inertia;
				std::vector <float> eccentricity;
				pcl::PointXYZRGBA min_point_AABB;
				pcl::PointXYZRGBA max_point_AABB;
				pcl::PointXYZRGBA min_point_OBB;
				pcl::PointXYZRGBA max_point_OBB;
				pcl::PointXYZRGBA position_OBB;
				Eigen::Matrix3f rotational_matrix_OBB;
				float major_value, middle_value, minor_value;
				Eigen::Vector3f major_vector, middle_vector, minor_vector;
				Eigen::Vector3f mass_center;

				feature_extractor.getMomentOfInertia (moment_of_inertia);
				feature_extractor.getEccentricity (eccentricity);
				feature_extractor.getAABB (min_point_AABB, max_point_AABB);
				feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB)
				feature_extractor.getEigenValues (major_value, middle_value, minor_value);
				feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
				feature_extractor.getMassCenter (mass_center);
				
				//viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

				Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
				Eigen::Quaternionf quat (rotational_matrix_OBB);
				//viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

				std::cout << min_point_AABB.x << max_point_AABB.x << min_point_AABB.y << max_point_AABB.y << min_point_AABB.z << max_point_AABB.z << std::endl;    				
			
			*/
			
			
			
			}
		}
		
		for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = outliers->points.begin(); it < outliers->points.end(); it++)
    	{	
    		int red = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
    		it->rgb= red;
    	}
    	
    	//TODO
    	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out;
    	for(int i=0; i < this->o.size(); i++)
    	{
    		*out += *(this->o[i]);
    	}
    	for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = out->points.begin(); it < out->points.end(); it++)
    	{	
    		int red = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
    		it->rgb= red;
    	}
    	//TODO
    	*planes += *out;
    	
    	
    	*planes += *outliers;
    	
    	
    	
    	

    	
    	
		coloredCloud = planes;
		//a->clear();
		//TODO clear segments and all other clouds after use
		//TODO 
		//TODO  check if plane_parallel_ransac is working as expected
		//TODO 
		//TODO  
		
		
		
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
