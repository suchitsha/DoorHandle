#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "Detect.h"
#include "GNG.h"


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
float cylS[3] = {0.0f,0.0f,0.0f};
float cylE[3] = {0.0f,0.0f,0.1f};

// print usage help
void printUsage (const char* progName)
{
        std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
                                                << "Options:\n"
                                                << "-------------------------------------------\n"
                                                << "-h                                         this help\n"
                                                << "-r                                         RGB colour visualisation example\n"
                                                << "-c                                         Custom colour visualisation example\n"
                                                << "-a                                         Shapes visualisation example\n"
                                                << "-f <file_name.oni>                         Custom file name"
                                                << "\n\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);

        //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        //viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud, 0, 255, 255);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, single_color, "sample cloud");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        pcl::ModelCoefficients coeffs;
        coeffs.values.clear ();
        coeffs.values.push_back (cylS[0]);//x
        coeffs.values.push_back (cylS[1]);//y
        coeffs.values.push_back (cylS[2]);//z
        coeffs.values.push_back (cylE[0]);//x
        coeffs.values.push_back (cylE[1]);//y
        coeffs.values.push_back (cylE[2]);//z
        coeffs.values.push_back (0.02);//r
        viewer->addCylinder(coeffs, "cylender");

        return (viewer);
}

//callback function for interfaces
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
        //choose between detect or gng
        Detect detect;
        *outCloud = *detect.startDetection(cloud, cylS, cylE);
        //outCloud = gng.startGNG(cloud);

}

int main (int argc, char** argv)
{
        // --------------------------------------
        // -----Parse Command Line Arguments-----
        // --------------------------------------
        if (pcl::console::find_argument (argc, argv, "-h") >= 0)
        {
                printUsage (argv[0]);
                return 0;
        }
        bool rgb(false), customFile(false), shapes(false);

        if (pcl::console::find_argument (argc, argv, "-f") >= 0)
        {
                if(argv[2])
                {
                    customFile = true;
                    std::cout << "Using custom file " << argv[2] << "\n";
                } else
                {
                    printUsage (argv[0]);
                    return 0;
                }
        }
        else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
        {
                rgb = true;
                std::cout << "RGB colour visualisation\n";
        }
        else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
        {
                shapes = true;
                std::cout << "Shapes visualisation\n";
        }
        else
        {
                std::cout << "Running program with default file name\n";
        }

        pcl::Grabber* interface;
        //read file
        // for oni file
        if(customFile)
        {
            interface = new pcl::ONIGrabber(argv[2] , false, false);
        }else
        {
            interface = new pcl::ONIGrabber("../data/Captured.oni", false, false);
        }

        /*
        //for pcd file uncomment code below
        boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > interface;
        interface.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> ("../data/other/kitchen/Rf3.pcd", 1, true));
        */

        //register callback
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&cloud_cb_, _1);
        interface->registerCallback(f);
        interface->start();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        viewer = customColourVis(outCloud);

        while (!viewer->wasStopped ())
        {
                interface->start();
                viewer->spinOnce (100);
                viewer->updatePointCloud(outCloud, "sample cloud");

                viewer->removeAllShapes();
                pcl::ModelCoefficients coeffs;
                coeffs.values.clear();
                coeffs.values.push_back(cylS[0]);//x
                coeffs.values.push_back(cylS[1]);//y
                coeffs.values.push_back(cylS[2]);//z
                coeffs.values.push_back(cylE[0]);//x
                coeffs.values.push_back(cylE[1]);//y
                coeffs.values.push_back(cylE[2]);//z
                coeffs.values.push_back (0.02);//r
                viewer->addCylinder(coeffs, "cylender");

                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
}
