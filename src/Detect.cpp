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

//hull
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

//TODO remove this when not needed
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Detect::startDetection(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud , float (&a)[3],float (&b)[3] ) {
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
		cloudTemp->clear(); //TODO uncomment this if this cloud is not used later in code (in bounding box)
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
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);//3
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
		std::vector<std::vector<float> > planeCoeff;
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments ;//(new std::vector<pcl::PointCloud<pcl::PointXYZRGBA> >);
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentsOutliers ;
		for(int i=0; i<clusters.size(); i++)
		{	
			//remove clusters with few points
            if(clusters[i].indices.size() > 100) //TODO choose correct value
			{
                //std::cout << "size of cluster indices:" << i <<":" << clusters[i].indices.size() << std::endl ;
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
                seg.setDistanceThreshold (0.02);//2
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
                        //std::cout << "Estimated a planar model." << std::endl;
					
						//max and min x and y for each cluster
						float maxX;
						float maxY;
						//float maxZ;
						float minX;
						float minY;
						//float minZ;
						float avgZ;
						for(int j=0; j < inliers->indices.size(); j++)
						{
                            //TODO replaced the line below with this //  pcl::PointXYZRGBA p = noNANCloud->at(inliers->indices[j]);
                            pcl::PointXYZRGBA p = segCloud->at(inliers->indices[j]);
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
                        //std::cout << "for cluster " << i << " max min values are " << maxX << " "<< minX << " "<< maxY << " "<< minY << " " << avgZ << std::endl;
                        //std::cout << "for segment " << segments.size() << " max min values are " << maxX << " "<< minX << " "<< maxY << " "<< minY << " " << avgZ << std::endl;
						//save boundries or size of plane
						std::vector<float> tempBound;
						tempBound.push_back(maxX);
						tempBound.push_back(maxY);
						tempBound.push_back(minX);
						tempBound.push_back(minY);
						boundaries.push_back(tempBound);
						//std::cout << "for cluster " << i << " max min values are " << boundaries[i].at(0) << " "<< boundaries[i].at(2) << " "<< boundaries[i].at(1) << " "<< boundaries[i].at(3) << " " << avgZ << std::endl;
						
						//save plane coefficients for the segment
						std::vector<float> tempcoefficients;
						tempcoefficients.push_back(coefficients->values[0]);
						tempcoefficients.push_back(coefficients->values[1]);
						tempcoefficients.push_back(coefficients->values[2]);
						tempcoefficients.push_back(coefficients->values[3]);
						planeCoeff.push_back(tempcoefficients);
						
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
        //std::cout << "sizes " << boundaries.size() << " " <<  segments.size() << " " << segmentsOutliers.size() << std::endl;
		//check for background planes
        float percentOfSize = 0.05; //TODO to constants
        float sizeThresh = -0.10;//TODO to constants TODO the value should be smaller when kinect is closer
		std::vector<std::vector<float> > inboundaries;
		std::vector<int> ignoreList;
		for(int l=0; l < boundaries.size(); l++)
		{	
			for(int m=0; m < boundaries.size(); m++)
			{	
				if(l != m)
				{	
					if( (boundaries[l].at(0) + sizeThresh) >= boundaries[m].at(0) )
					{
						if( (boundaries[l].at(1) + sizeThresh) >= boundaries[m].at(1) )
						{
							if( (boundaries[l].at(2) - sizeThresh) <= boundaries[m].at(2) )
							{
								if( (boundaries[l].at(3) - sizeThresh) <= boundaries[m].at(3) )
								{	
									//if plane inside is large enough, this is to eliminate wrong values due to noise or small structures in a plane
									if( percentOfSize*segments[l]->size() < segments[m]->size() )
									{	
                                        //std::cout << "adding to ignoreList: " << l  << " of size: " << segments[l]->size() << " compared: " << m << " " << segments[m]->size() << std::endl;
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
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr handle (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for(int i=0; i < boundaries.size(); i++)
		{
			if( std::find(ignoreList.begin(), ignoreList.end(), i) != ignoreList.end() ) 
			{
				// ignoreList contains index so ignore 
			} else 
			{
				// ignoreList does not contain index
                //std::cout << "showing cloud: " << i << " of size: " << segments[i]->size() << std::endl;
                //std::cout << "showing cloud: " << i << " of boundary: " << boundaries[i].at(0) << " " << boundaries[i].at(1) << " "<< boundaries[i].at(2) << " " << boundaries[i].at(3)  << std::endl;
				
				*outliers += *segmentsOutliers[i];
                // add all other points with their original color for visualization
                *planes += *segments[i];
				
				//check if point lies within boundries of the plane
                float minDistThresh = -0.015; //TODO to constants
                float maxDistThresh = -0.11; //TODO to constants
                float minInThresh = 0.15; //TODO to constants
                float maxInThresh = 0.15; //TODO to constants

				int green = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
                //int grey = ((int)1) << 16 | ((int)1) << 8 | ((int)1);
                //int redl = ((int)100) << 16 | ((int)0) << 8 | ((int)0);
                //int red = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
                for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it = noNANCloud->begin(); it!= noNANCloud->end(); it++)
				{
					//put the point in plane equation of segment[i] ax + by + cz +d = val and minThresh < val < maxThresh
					float val = planeCoeff[i].at(0)*it->x + planeCoeff[i].at(1)*it->y + planeCoeff[i].at(2)*it->z + planeCoeff[i].at(3) ;
					
					pcl::PointXYZRGBA p ;
					p.x = it->x;
					p.y = it->y;
					p.z = it->z;
                    //p.rgb = it->rgb;
					
					//std::cout << "boundaries for seg: " << i << ":" << boundaries[i].at(0) << ":"  << boundaries[i].at(1) << ": " << boundaries[i].at(2) << " " << boundaries[i].at(3) << std::endl;
					//check if x y of point lies in the required range
                    if( (boundaries[i].at(0) - maxInThresh) > p.x )
					{ 
                        if( (boundaries[i].at(1) - maxInThresh) > p.y )
						{ 
                            if( (boundaries[i].at(2) + minInThresh) < p.x )
							{ 
                                if( (boundaries[i].at(3) + minInThresh) < p.y )
								{	
                                    //if z of point is between the required range
									if(val < minDistThresh)
									{
										if(val > maxDistThresh)
										{	
											//pcl::PointXYZRGBA p ;
											//p.x = it->x;
											//p.y = it->y;
											//p.z = it->z;
                                            p.rgb = green;//it->rgb;
                                            //planes->push_back(p);
                                            handle->push_back(p);
											//cout << "value and plane coeff: " << val << " " << planeCoeff[i].at(0) << " " << planeCoeff[i].at(1) << planeCoeff[i].at(2) << planeCoeff[i].at(3) << endl;
										}
									}
                                }
							}
						}
					}
                }

                /*// uncommment this to see convex hull points
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
				temp = segments[i];
				pcl::ConvexHull<pcl::PointXYZRGBA> hull;
				hull.setInputCloud(temp);
				hull.setDimension(3);
				std::vector<pcl::Vertices> polygons;
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr surfaceHull (new pcl::PointCloud<pcl::PointXYZRGBA>);
				hull.reconstruct(*surfaceHull, polygons);
				for(int i = 0; i < polygons.size(); i++)
				{
					std::cout << " vertices of hull: " << polygons.size() << " : "<< i << " : " << polygons[i] << std::endl;
				}	
				
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects (new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::CropHull<pcl::PointXYZRGBA> bbFilter;

				bbFilter.setDim(3);
				bbFilter.setInputCloud(noNANCloud);
				bbFilter.setHullIndices(polygons);
				bbFilter.setHullCloud(temp);
				bbFilter.setCropOutside(true);
				bbFilter.filter(*objects);
				std::cout << "size of objects is: " << objects->size() << std::endl;

				*planes += *objects;
				//*planes += *surfaceHull;
				*/
			}
		}
		
        /* uncomment to see segments
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        for(int i =0;i<segments.size();i++)
    	{
            for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it1 = segments[i]->begin(); it1 != segments[i]->end(); it1++)
    		{
    			pcl::PointXYZRGBA p ;
				p.x = it1->x;
				p.y = it1->y;
				p.z = it1->z;
				p.rgb = it1->rgb;
    			newCloud->push_back(p);
    		}

        }
        coloredCloud = newCloud;*/





        /* Uncomment to see k nearest neighbours colored
        // color neighbours of detected area
        //----nearest neighbours----//
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
        kdtree.setInputCloud (planes);
        //std::vector<int> pointIdxRadiusSearch;
        //std::vector<float> pointRadiusSquaredDistance;
        //float radius = 0.03 ;//TODO to constants
        // find K nearest neighbor
        int K = 10;//TODO to constants
        float distThresh = 0.05; //TODO to constants
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        int green1 = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
        for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it1 = planes->begin(); it1 != planes->end(); it1++)
        {
            if (it1->rgb == green1)
            {
                pcl::PointXYZRGBA point ;
                point.x = it1->x;
                point.y = it1->y;
                point.z = it1->z;
                point.rgba = it1->rgba;
                if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
                    {
                        if(pointNKNSquaredDistance[i] < distThresh)
                        {
                            planes->points[pointIdxNKNSearch[i]].rgba = green1;
                        }
                    }
                }
            }
        }
        */

        //make clusters of handle candidate
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(handle);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        //TODO try these parameters  and set them as constants
        ec.setClusterTolerance (0.10);
        ec.setMinClusterSize (2);
        ec.setMaxClusterSize (handle->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(handle);
        ec.extract(cluster_indices);


        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          {
            cloud_cluster->points.push_back(handle->points[*pit]);
          }
          cloud_cluster->width = cloud_cluster->points.size();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          //std::cout << "PointCloud representing the Cluster" << j << " : " << cloud_cluster->points.size() << " data points." << std::endl;
          j++;


          //line
          pcl::SACSegmentation<pcl::PointXYZRGBA> segm;
          pcl::PointIndices::Ptr inliersLine(new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
          segm.setModelType(pcl::SACMODEL_LINE);
          segm.setMethodType(pcl::SAC_RANSAC);
          //TODO try these parameters  and set them as constants
          segm.setDistanceThreshold(0.15);//.006
          segm.setInputCloud(cloud_cluster);
          segm.segment(*inliersLine, *coefficients);
          //std::cout << "inliers: " << *inliersLine << std::endl;
          //std::cout << "coefficients: " << *coefficients << std::endl;

          Eigen::Vector4f tmax, tmin;
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpObject (new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::ExtractIndices<pcl::PointXYZRGBA> ext;
          ext.setInputCloud(cloud_cluster);
          ext.setNegative(false);
          ext.setIndices(boost::make_shared<pcl::PointIndices>(*inliersLine));
          ext.filter(*tmpObject);

          /*
          // Remove the planar inliers, extract the rest
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpOutliers (new pcl::PointCloud<pcl::PointXYZRGBA>);
          ext.setNegative(true);
          ext.filter (*tmpOutliers);

          // if there are lots of outliers, probably its not a line but rounded object so ignore it
          if( tmpOutliers->size() > (0.1 * tmpObject->size()) )
          {
              //skip this cluster
              continue;
          }
          */

          pcl::getMinMax3D(*tmpObject, tmin, tmax);

          //std::cout << "tmax: " << tmax << std::endl;
          //std::cout << "tmin: " << tmin << std::endl;

          a[0] = tmin[0];//coefficients->values[0];//tmin[0];//x
          a[1] = tmin[1];//coefficients->values[1];//tmin[1];//y
          a[2] = tmin[2];//coefficients->values[2];//tmin[2];//z
          b[0] = tmax[0] - tmin[0]; //coefficients->values[3];//tmax[0];//direction x
          b[1] = tmax[1] - tmin[1]; //coefficients->values[4];//tmax[1];// dir y
          b[2] = tmax[2] - tmin[2]; //coefficients->values[5];//tmax[2];//dir z
          //std::cout << "a b: " << a[0] << " " << a[1] << a[2] << " " << b[0] <<  b[1] << " " << b[2] << std::endl;

          for(int i=0; i < inliersLine->indices.size(); i++)
          {
              pcl::PointXYZRGBA p = cloud_cluster->at(inliersLine->indices[i]);
              //TODO put this into a variable
              p.rgb = ((int)1) << 16 | ((int)(j*10)) << 8 | ((int)255);
              planes->push_back(p);
          }
          // display only largest cluster
          //break;
        }

        //fit line/sphere/cylender
        //sphere
        std::vector<int> inliers;
        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (handle));
        //TODO get size through command line
        model_s->setRadiusLimits(0.012, 0.0675);
        pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransc (model_s);
        ransc.setDistanceThreshold(.02);//.01
        ransc.computeModel();
        ransc.getInliers(inliers);

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZRGBA>(*handle, inliers, *final);
        for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it1 = final->begin(); it1 != final->end(); it1++)
        {
            pcl::PointXYZRGBA p;
            p.x = it1->x;
            p.y = it1->y;
            p.z = it1->z;
            //TODO put this into a variable
            p.rgb = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
            planes->push_back(p);
        }

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
                if(pointNKNSquaredDistance[i] < distThresh){
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
