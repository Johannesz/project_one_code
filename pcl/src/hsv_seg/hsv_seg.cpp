#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


void copyIndices(pcl::IndicesPtr input, pcl::IndicesPtr output)
{
   std::vector<int>::const_iterator it;
   output->clear();
   for(it = input->begin() ; it!=input->end() ; it++) {
      output->push_back((*it));
   }
}



int main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  // Datasets
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Read in the cloud data
  reader.read ("cloud_with_led.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;


  pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_filtered);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
     cloud_filtered->points[i].x = cloud->points[i].x;
     cloud_filtered->points[i].y = cloud->points[i].y;
     cloud_filtered->points[i].z = cloud->points[i].z;


  }

  pcl::PassThrough<pcl::PointXYZHSV> pass;

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("s");
  pass.setFilterLimits(0.0, 0.2);
  pass.filter (*cloud_filtered2);

  pass.setInputCloud(cloud_filtered2);
  pass.setFilterFieldName("v");
  pass.setFilterLimits(0.8, 1.0);
  pass.filter (*cloud_filtered);

// bis hierhin funktioniert alles - was ich nicht check bis jetzt ist wie ich von hsv wieder zurück auf rgb komme
// bzw. wie ich die restlichen Indices aus dem HSV Bild nehm, dass im RGB Bild nur noch diese dr

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


  copyIndices(*cloud_filtered, *inliers);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>(*cloud, *inliers));

  //extract.setInputCloud (cloud);
  //extract.setIndices (inliers);

  //extract.setNegative (false);
  //extract.filter (*cloud_final);



  std::cerr << "PointCloud nach v has: " << newCloud->points.size () << " data points." << std::endl;
  writer.write ("hsv.pcd", *newCloud, false);



  return (0);
}



