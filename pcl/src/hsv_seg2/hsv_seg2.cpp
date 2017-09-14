#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
   // All the objects needed
   pcl::PCDReader reader;
   pcl::PCDWriter writer;

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
         new pcl::PointCloud<pcl::PointXYZRGB>);

   reader.read("white_cylinder.pcd", *cloud);
   std::cout << "PointCloud has: " << cloud->points.size() << " data points."
         << std::endl;

   cv::Mat result;

   if (cloud->isOrganized()) {
      result = cv::Mat(cloud->height, cloud->width, CV_8UC3);

      if (!cloud->empty()) {

         for (int h = 0; h < result.rows; h++) {
            for (int w = 0; w < result.cols; w++) {
               pcl::PointXYZRGB point = cloud->at(w, h);

               Eigen::Vector3i rgb = point.getRGBVector3i();

               result.at<cv::Vec3b>(h, w)[0] = rgb[2];
               result.at<cv::Vec3b>(h, w)[1] = rgb[1];
               result.at<cv::Vec3b>(h, w)[2] = rgb[0];
            }
         }
      }
   }

   cv::namedWindow("von pcl zu mat", cv::WINDOW_AUTOSIZE);
   cv::imshow("von pcl zu mat", result);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(
         new pcl::PointCloud<pcl::PointXYZRGB>());
   cv::Mat coords(3, result.cols * result.rows, CV_64FC1);
   for (int y = 0; y < result.rows; y++) {
      for (int x = 0; x < result.cols; x++) {
         pcl::PointXYZRGB point;
         point.x = coords.at<double>(0, y * result.cols + x);
         point.y = coords.at<double>(1, y * result.cols + x);
         point.z = coords.at<double>(2, y * result.cols + x);

         cv::Vec3b color = result.at<cv::Vec3b>(cv::Point(x, y));
         uint8_t r = (color[2]);
         uint8_t g = (color[1]);
         uint8_t b = (color[0]);

         int32_t rgb = (r << 16) | (g << 8) | b;
         point.rgb = *reinterpret_cast<float*>(&rgb);

         cloud2->points.push_back(point);
      }
   }

   std::cerr << "PointCloud nach Konvertierung hat: " << cloud2->points.size()
         << " data points." << std::endl;
   writer.write("back_cylinder.pcd", *cloud2, false);

   cv::waitKey(0);

   return 0;

}

/*
 * #include <pcl/visualization/cloud_viewer.h>

 #include <opencv2/core.hpp>
 #include <opencv2/imgproc.hpp>
 #include <opencv2/highgui.hpp>

 void draw_cloud(
 const std::string &text,
 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
 {
 pcl::visualization::CloudViewer viewer(text);
 viewer.showCloud(cloud);
 while (!viewer.wasStopped())
 {
 }
 }

 pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(
 const cv::Mat& image,
 const cv::Mat &coords)
 {
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

 for (int y=0;y<image.rows;y++)
 {
 for (int x=0;x<image.cols;x++)
 {
 pcl::PointXYZRGB point;
 point.x = coords.at<double>(0,y*image.cols+x);
 point.y = coords.at<double>(1,y*image.cols+x);
 point.z = coords.at<double>(2,y*image.cols+x);

 cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
 uint8_t r = (color[2]);
 uint8_t g = (color[1]);
 uint8_t b = (color[0]);

 int32_t rgb = (r << 16) | (g << 8) | b;
 point.rgb = *reinterpret_cast<float*>(&rgb);

 cloud->points.push_back(point);
 }
 }
 return cloud;
 }

 void cloud_to_img(
 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
 cv::Mat &coords,
 cv::Mat &image)
 {
 coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
 image = cv::Mat(480, 640, CV_8UC3);
 for(int y=0;y<image.rows;y++)
 {
 for(int x=0;x<image.cols;x++)
 {
 coords.at<double>(0,y*image.cols+x) = cloud->points.at(y*image.cols+x).x;
 coords.at<double>(1,y*image.cols+x) = cloud->points.at(y*image.cols+x).y;
 coords.at<double>(2,y*image.cols+x) = cloud->points.at(y*image.cols+x).z;

 cv::Vec3b color = cv::Vec3b(
 cloud->points.at(y*image.cols+x).b,
 cloud->points.at(y*image.cols+x).g,
 cloud->points.at(y*image.cols+x).r);

 image.at<cv::Vec3b>(cv::Point(x,y)) = color;
 }
 }
 }

 int main(int argc, char *argv[])
 {
 cv::Mat image = cv::imread("test.png");
 cv::resize(image, image, cv::Size(640, 480));
 cv::imshow("initial", image);

 cv::Mat coords(3, image.cols * image.rows, CV_64FC1);
 for (int col = 0; col < coords.cols; ++col)
 {
 coords.at<double>(0, col) = col % image.cols;
 coords.at<double>(1, col) = col / image.cols;
 coords.at<double>(2, col) = 10;
 }

 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(image, coords);
 draw_cloud("points", cloud);

 cloud_to_img(cloud, coords, image);
 cv::imshow("returned", image);

 cv::waitKey();
 return 0;
 }
 */
