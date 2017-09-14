#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{

   // All the objects needed
  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud  (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud3 (new pcl::PointCloud<PointT>);

  float x1 = 0.0;
  float x2 = 0.0;
  float y1 = 0.0;
  float y2 = 0.0;
  float z1 = 0.0;
  float z2 = 0.0;


  cloud3->width  = 640;
  cloud3->height = 480;
  cloud3->points.resize (cloud3->width * cloud3->height);

  // Read in the cloud data
  reader.read ("one.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  reader.read ("two.pcd", *cloud2);
  std::cerr << "PointCloud has: " << cloud2->points.size () << " data points." << std::endl;

  // Setting nan to zero
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
	  if (std::isnan(cloud->points[i].x))
	  {
		  cloud->points[i].x = 0.0;
	  }
	  if (std::isnan(cloud->points[i].y))
	  {
	  	  cloud->points[i].y = 0.0;
	  }
	  if (std::isnan(cloud->points[i].z))
	  {
	  	  cloud->points[i].z = 0.0;
	  }
  }

  for (size_t i = 0; i < cloud2->points.size (); ++i)
  {
	  if (std::isnan(cloud2->points[i].x))
	  {
		  cloud2->points[i].x = 0.0;
	  }
	  if (std::isnan(cloud2->points[i].y))
	  {
	 	  cloud2->points[i].y = 0.0;
	  }
	  if (std::isnan(cloud2->points[i].z))
	  {
	 	  cloud2->points[i].z = 0.0;
	  }
  }




  // summing up the point clouds and calculating the average
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
	  x2 = cloud2->points[i].x;
	  x1 = cloud->points[i].x;

	  y2 = cloud2->points[i].y;
	  y1 = cloud->points[i].y;

	  z2 = cloud2->points[i].z;
	  z1 = cloud->points[i].z;
	  //std::cerr << x1 << " " << x2 << std::endl;

	  if (x1 == 0.0 && x2 == 0.0 )
	  	  {
		  cloud3->points[i].x = 0.0;
	  	  }
	  else if (x1 == 0.0 && x2 != 0.0)
	  {
		  cloud3->points[i].x = x2;
	  }
	  else if (x1 != 0.0 && x2 == 0.0)
	  {
		  cloud3->points[i].x = x1;
	  }
	  else
	  	  {
		  cloud3->points[i].x = ((x1 + x2) *0.5);
	  	  }
	  x2 = 0.0;
	  x1 = 0.0;

	  if (y1 == 0.0 && y2 == 0.0 )
	 	  {
		  cloud3->points[i].y = 0.0;
		  }
	  else if (y1 == 0.0 && y2 != 0.0)
	  {
		  cloud3->points[i].y = y2;
	  }
	  else if (y1 != 0.0 && y2 == 0.0)
	  {
		  cloud3->points[i].y = y1;
	  }
	  else
	  	  {
		  cloud3->points[i].y = ((y1 + y2) *0.5);
	  	  }
	  y2 = 0.0;
	  y1 = 0.0;

	  if (z1 == 0.0 && z2 == 0.0 )
	  {
	  cloud3->points[i].z = 0.0;
	  }
	  else if (z1 == 0.0 && z2 != 0.0)
	  {
		  cloud3->points[i].z = z2;
	  }
	  else if (z1 != 0.0 && z2 == 0.0)
	  {
		  cloud3->points[i].z = z1;
	  }
	  else
	  {
	  cloud3->points[i].z = ((z1 + z2) *0.5);
	  }
	  z2 = 0.0;
	  z1 = 0.0;

  }

  std::cerr << "Average Point Cloud: " << cloud3->points.size () << " data points." << std::endl;
  writer.write ("average.pcd", *cloud3, false);

  return (0);
}
