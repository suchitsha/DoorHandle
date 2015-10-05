/*
 * File:   GNG.cpp
 * Author: ssharma
 * Description: Door Handle Detection
 * 
 */
#include "GNG.h"
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

#include <math.h>

#include "Node.h"
//#include "Edge.h"

using namespace pcl;

double findDistance(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it,std::vector<Node>::iterator ni);

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr GNG::startGNG(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

        // coordinates of y axis
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr coloredCloud;
	    	    
	    // Outlier removal for filtering noise 
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
        sor.filter (*cloudTemp);
	    
        //------------Start---------------//
        long nodeCounter = 0;
        int green = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
        int white = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
        long inputCounter = 0;
        std::vector<pcl::PointXYZRGBA> input;
        std::vector<Node> nodes;

        // TODO to gng.h
        //constants
        float eb = 0.2;
        float en = 0.006;
        int aMax = 50;
        float d = 0.995;
        float alpha = 0.5;

        //step 0: start with two random units a and b
        Node n;
        n.id = nodeCounter;
        pcl::PointXYZRGBA p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        p.rgb = green;
        n.point = p;
        std::vector<Edge> edg;
        n.edges = edg;
        nodes.push_back(n);
        nodeCounter++;

        n.id = nodeCounter;
        p.x = 1.0;
        p.y = 1.0;
        p.z = 1.0;
        n.point = p;
        n.edges = edg;
        nodes.push_back(n);
        nodeCounter++;

        //step 1: generate a input signal
        for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloudTemp->points.begin(); it < cloudTemp->points.end(); it++)
        {
            inputCounter++;

            //step 2: find the nearest and second nearest units s1 and s2
            double minDis = 100;
            long minDisId;
            double minDis2 = 101;
            long minDisId2;
            long indexS1;
            long indexS2;
            long counter = 0;
            for(std::vector<Node>::iterator i = nodes.begin(); i != nodes.end(); i++)
            {
                double dis = findDistance(it,i);
                if(dis < minDis2)
                {
                    if(dis < minDis)
                    {
                        //TODO these assignments may through exeption of null assignment at first run
                        minDis2 = minDis;
                        minDisId2 = minDisId;
                        indexS2 = indexS1;

                        minDis = dis;
                        minDisId = i->id;
                        indexS1 = counter;
                    }else
                    {
                        minDis2 = dis;
                        minDisId2 = i->id;
                        indexS2 = counter;
                    }
                }
                counter++;
            }

            //step 3: increment age of all edges from s1
            for(std::vector<Edge>::iterator i = nodes[indexS1].edges.begin(); i != nodes[indexS1].edges.end(); i++)
            {
                i->age++;
            }

            //step 4: add squared distance between input signal and closest node to error
            nodes[indexS1].error = pow(minDis,2);

            //step 5: move s1 and its direct topological neighbours towards input by a factor of eb and en of the total distance
            //update s1
            nodes[indexS1].point.x = nodes[indexS1].point.x + eb*(it->x - nodes[indexS1].point.x);
            nodes[indexS1].point.y = nodes[indexS1].point.y + eb*(it->y - nodes[indexS1].point.y);
            nodes[indexS1].point.z = nodes[indexS1].point.z + eb*(it->z - nodes[indexS1].point.z);
            //update neighbours
            for(std::vector<Edge>::iterator i = nodes[indexS1].edges.begin(); i != nodes[indexS1].edges.end(); i++)
            {
                int id1 = i->from;
                int id2 = i->to;
                if(id1 == nodes[indexS1].id)
                {
                    //use id2
                    // TODO check if node.id and nodes[index] are same
                    if(nodes[id2].id != id2)
                    {
                        std::cout << "problem" << std::endl;
                    }
                    nodes[id2].point.x = nodes[id2].point.x + en*(it->x - nodes[id2].point.x);
                    nodes[id2].point.y = nodes[id2].point.y + en*(it->y - nodes[id2].point.y);
                    nodes[id2].point.z = nodes[id2].point.z + en*(it->z - nodes[id2].point.z);
                }else
                {
                    //use id1
                    // TODO check if node.id and nodes[index] are same
                    if(nodes[id1].id != id1)
                    {
                        std::cout << "problem" << std::endl;
                    }
                    nodes[id1].point.x = nodes[id1].point.x + en*(it->x - nodes[id1].point.x);
                    nodes[id1].point.y = nodes[id1].point.y + en*(it->y - nodes[id1].point.y);
                    nodes[id1].point.z = nodes[id1].point.z + en*(it->z - nodes[id1].point.z);
                }
            }

            //step 6: if s1 and s2 are connected by an edge set its age as zero, if there is no edge then create it
            bool hasEdge = false;
            for(std::vector<Edge>::iterator i = nodes[indexS1].edges.begin(); i != nodes[indexS1].edges.end(); i++)
            {
                if(i->from == indexS2)
                {
                    hasEdge = true;
                    i->age = 0;
                } else if(i->to == indexS2)
                {
                    hasEdge = true;
                    i->age = 0;
                }
            }
            if(!hasEdge)
            {
                //create edge between s1 and s2
                Edge e;
                e.age = 0;
                e.from = indexS1;
                e.to = indexS2;
                nodes[indexS1].edges.push_back(e);
                e.from = indexS2;
                e.to = indexS1;
                nodes[indexS2].edges.push_back(e);
            }
            //step 7: remove edge with age > aMax , remove nodes without edges
            for(std::vector<Node>::iterator ni = nodes.begin(); ni != nodes.end(); ni++)
            {
                for(std::vector<Edge>::iterator i = ni->edges.begin(); i != ni->edges.end(); i++)
                {
                    if(i->age > aMax)
                    {
                        if(!ni->edges.empty())
                        {
                            ni->edges.erase(i);
                        }
                    }
                }
            }

            for(std::vector<Node>::iterator ni = nodes.begin(); ni != nodes.end(); ni++)
            {
                //if(ni->edges.size() == 0)
                if(ni->edges.empty())
                {
                    if(!nodes.empty())
                    {
                       nodes.erase(ni);
                    }
                }
            }

            /*step 8: at every lambda input:
              determine q with max error
              insert a new node r halfway between q and its neighbour f with largest error
              insert edges between q-r and r-f, and remove edge between q-f
              decrease error of q and f by alpha, initialize error of r with new error of q
            */
            if(inputCounter % lambda == 0)
            {
                // max error
                double maxError = 0;
                long maxIndex;
                for(std::vector<Node>::iterator ni = nodes.begin(); ni != nodes.end(); ni++)
                {
                    if(ni->error > maxError)
                    {
                        maxError = ni->error;
                        maxIndex = ni->id;
                    }
                }
                //find neighbour with max error
                double maxNbrError = 0;
                long maxNbrIndex;
                for(std::vector<Edge>::iterator i = nodes[maxIndex].edges.begin(); i != nodes[maxIndex].edges.end(); i++)
                {
                    double nbrIndex = i->to;
                    if(nodes[nbrIndex].error > maxNbrError)
                    {
                        maxNbrError = nodes[nbrIndex].error;
                        maxNbrIndex = nbrIndex;
                    }
                }

                //insert new node
                Node n1;
                n1.id = nodeCounter;
                pcl::PointXYZRGBA p1;
                p1.x = 0.5*(nodes[maxIndex].point.x + nodes[maxNbrIndex].point.x);
                p1.y = 0.5*(nodes[maxIndex].point.y + nodes[maxNbrIndex].point.y);
                p1.z = 0.5*(nodes[maxIndex].point.z + nodes[maxNbrIndex].point.z);
                p1.rgb = green;
                n1.point = p1;
                n1.edges = edg;
                nodes.push_back(n1);
                nodeCounter++;

                //update edges
                //q-r
                Edge e1;
                e1.age = 0;
                e1.from = maxIndex;
                e1.to = n1.id;
                nodes[maxIndex].edges.push_back(e1);
                e1.from = n1.id;
                e1.to = maxIndex;
                nodes[n1.id].edges.push_back(e1);

                //r-f
                e1.from = maxNbrIndex;
                e1.to = n1.id;
                nodes[maxNbrIndex].edges.push_back(e1);
                e1.from = n1.id;
                e1.to = maxNbrIndex;
                nodes[n1.id].edges.push_back(e1);

                //remove q-f
                for(std::vector<Edge>::iterator i = nodes[maxIndex].edges.begin(); i != nodes[maxIndex].edges.end(); i++)
                {
                    if(i->to == maxNbrIndex)
                    {
                        nodes[maxIndex].edges.erase(i);
                    } else if(i->from == maxNbrIndex)
                    {
                        nodes[maxIndex].edges.erase(i);
                    }
                }
                for(std::vector<Edge>::iterator i = nodes[maxNbrIndex].edges.begin(); i != nodes[maxNbrIndex].edges.end(); i++)
                {
                    if(i->to == maxIndex)
                    {
                        nodes[maxNbrIndex].edges.erase(i);
                    } else if(i->from == maxIndex)
                    {
                        nodes[maxNbrIndex].edges.erase(i);
                    }
                }

                //decrease error q-f by alpha
                nodes[maxIndex].error = alpha * nodes[maxIndex].error;
                nodes[maxNbrIndex].error = alpha * nodes[maxNbrIndex].error;
                // error r = error q
                nodes[n1.id].error = nodes[maxIndex].error;

            }

            //step 9: decrease all errors by d
            for(std::vector<Node>::iterator ni = nodes.begin(); ni != nodes.end(); ni++)
            {
                ni->error = d * ni->error;
            }

            //step 10: break if stopping condition is met like no of inputs, etc

            //-------------GNG algo finishes here-----------//

            //color each point for visuallization
            it->rgb = white;

        }

        // insert all nodes to a point cloud
        for(std::vector<Node>::iterator ni = nodes.begin(); ni != nodes.end(); ni++)
        {
            cloudTemp->push_back(ni->point);
        }

        //write to output cloud
        coloredCloud = cloudTemp;
	    return coloredCloud;
}

double findDistance(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it,std::vector<Node>::iterator ni)
{
    return sqrt( pow((ni->point.x - it->x),2) + pow((ni->point.y - it->y),2) + pow((ni->point.z - it->z),2) );
}
