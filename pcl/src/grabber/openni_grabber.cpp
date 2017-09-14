 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>



class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
        pcl::PCDWriter writer;
        if (!viewer.wasStopped()){
          viewer.showCloud (cloud);
          std::cerr << "Average Point Cloud: " << cloud->points.size () << " data points." << std::endl;
          writer.write ("cloud.pcd", *cloud, false);
          sleep(5000);
       }

     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };



 int main ()
 {

    SimpleOpenNIViewer v;
    v.run ();
   return 0;

 }
