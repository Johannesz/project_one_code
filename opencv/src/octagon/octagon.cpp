#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <math.h>
#include <algorithm>

int main(int argc, char **argv) {

   char* imageName = argv[1];
   cv::Mat image;
   image = imread(imageName, cv::IMREAD_COLOR);
   if (argc != 2 || !image.data) {
      printf(" No image data \n ");
      return -1;
   }

   // Alternativen evlt. noch testen find contours, template matching

   // Color Filter on Color Blue
   cv::Mat blue_filter;
   cv::inRange(image, cv::Scalar(200, 0, 0), cv::Scalar(255, 255, 255),
         blue_filter);

   // Erosion filter to that only the sqares will be left
   // Sqares will not have the correct size afterwards - they will be smaller
   cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(19, 19),
         cv::Point(8, 8));
   cv::Mat erosion;
   cv::erode(blue_filter, erosion, element);

   // Delitate filter to get the old size of the squares - problem with rotated squares
//   cv::Mat element2 = getStructuringElement( cv::MORPH_RECT,
//                        cv::Size( 9, 9 ),
//                        cv::Point( 4, 4 ) );
//   cv::Mat delitate;
//   cv::dilate( erosion, delitate, element2 );

   // Harris Corner detection to find the corners of the squares
   cv::Mat harris;
   cv::Mat harris_norm;
   cv::Mat harris_norm_scaled;

   cv::cornerHarris(erosion, harris, 4, 3, 0.16, cv::BORDER_DEFAULT);
   cv::normalize(harris, harris, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
   cv::convertScaleAbs(harris_norm, harris_norm_scaled);

   std::vector<cv::Vec2f> center;

   for (int j = 0; j < harris.rows; j++) {
      for (int i = 0; i < harris.cols; i++) {
         if ((int) harris.at<float>(j, i) > 140) {
            //cv::circle(image, cv::Point(i, j), 5, cv::Scalar(0), 2, 8, 0);
            center.push_back(cv::Vec2f(i, j));

         }
      }
   }

   // Show all found corners
//   for (auto const& circle : center) {
//      std::cout << circle << std::endl;
//   }

   std::cout << "Anzahl vollständig: " << center.size() << std::endl;

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // reduce found cornes to one per square corner
   std::vector<cv::Vec2f> filt_corner;

   float sumX = 0.0;
   float sumY = 0.0;
   int similarCorner = 0;

   /////////////////////// Loop through all found corners
   for (size_t current_corner = 0; current_corner < center.size();
         ++current_corner) {

      //Check if filtered corner vector is empty
      if (filt_corner.empty()) {
         //if empty loop through the first vector and look for similar values
         sumX = center[current_corner][0];
         sumY = center[current_corner][1];
         similarCorner++;

         for (size_t search_corner = current_corner;
               search_corner < center.size(); ++search_corner) {

            if ((abs(center[current_corner][0] - center[search_corner][0]) < 5)
                  && (abs(center[current_corner][1] - center[search_corner][1])
                        < 5)) {
               sumX += center[search_corner][0];
               sumY += center[search_corner][1];
               similarCorner++;
            }
         }

         sumX = sumX / similarCorner;
         sumY = sumY / similarCorner;
         filt_corner.push_back(cv::Vec2f(sumX, sumY));
      }
      /// after the first corner is in the new array
      else {
         //check if the actual corner of the first vector is already in the second vector for found arrays
         int count = 0;
         for (size_t search_corner2 = 0; search_corner2 < filt_corner.size();
               ++search_corner2) {
            //if distance of x and y is smaller than 5,  the corner is alrady in the second vector at this special place
            if (abs(center[current_corner][0] - filt_corner[search_corner2][0])
                  < 5) {
               if (abs(
                     center[current_corner][1] - filt_corner[search_corner2][1])
                     < 5) {
                  count++;
               }
            }
         }

         // if counted corners has the same size like fields in the second vector a new field in the second array has to be created
         if (count == 0) {
            similarCorner = 0;
            sumX = center[current_corner][0];
            sumY = center[current_corner][1];
            similarCorner++;

            for (size_t search_corner = current_corner;
                  search_corner < center.size(); ++search_corner) {

               if (abs(center[current_corner][0] - center[search_corner][0])
                     < 5) {
                  if (abs(center[current_corner][1] - center[search_corner][1])
                        < 5) {
                     sumX += center[search_corner][0];
                     sumY += center[search_corner][1];
                     similarCorner++;
                  }
               }

            }

            sumX = sumX / similarCorner;
            sumY = sumY / similarCorner;

            filt_corner.push_back(cv::Vec2f(sumX, sumY));

         }

         sumX = 0.0;
         sumY = 0.0;
         similarCorner = 0;

      }

   }

   std::cout << "Eckpunkte: " << std::endl << std::endl;
   // Show all found corners
   for (auto const& circle2 : filt_corner) {
      std::cout << circle2 << std::endl;
   }
   std::cout << "Anzahl reduziert: " << filt_corner.size() << std::endl;

   if (filt_corner.size() == 0)
      std::exit(-1);
   for (size_t current_corner3 = 0; current_corner3 < filt_corner.size();
         ++current_corner3) {
      cv::circle(image,
            cv::Point(filt_corner[current_corner3][0],
                  filt_corner[current_corner3][1]), 5, cv::Scalar(0), 2, 8, 0);
   }

   /////////////////////////////////////////////////
   //// calculate sqares (80 Pixel is a good start

   std::vector<cv::Vec3f> middle_point;
   float middleX = 0.0;
   float middleY = 0.0;
   float hypotenuse = 0.0;
   float adjacent = 0.0;

   for (size_t current_corner4 = 0; current_corner4 < filt_corner.size();
         ++current_corner4) {

      //put the first corner into the sqare iterator

      std::vector<cv::Vec2f> iterator;
      iterator.push_back(
            cv::Vec2f(filt_corner[current_corner4][0],
                  filt_corner[current_corner4][1]));
      //search for the 3 further corners of a squares within a certain area

      for (size_t current_corner5 = (current_corner4 + 1);
            current_corner5 < filt_corner.size(); ++current_corner5) {
         if (abs(iterator[0][0] - filt_corner[current_corner5][0]) < 80) {
            if (abs(iterator[0][1] - filt_corner[current_corner5][1]) < 80) {
               iterator.push_back(
                     cv::Vec2f(filt_corner[current_corner5][0],
                           filt_corner[current_corner5][1]));
            }
         }
      }

      // if four points are within a certain area the middlepoint of a square will calcualted
      if (iterator.size() == 4) {

//         std::cout << std::endl <<  "unsortiert: " <<  std::endl;
//               for (auto const& circle4 : iterator) {
//                       std::cout << circle4 << std::endl;
//                    }

         for (size_t it = 0; it < iterator.size(); ++it) {
            middleX += iterator[it][0];
            middleY += iterator[it][1];

         }
         middleX = middleX / 4;
         middleY = middleY / 4;

         std::sort(iterator.begin(), iterator.end(),
               [](const cv::Point2f &a, const cv::Point2f &b) {
                  //return ( a.x*a.x + a.y*a.y < b.x*b.x + b.y*b.y);
                  return ( a.y < b.y);

               });

//            std::cout << std::endl << "sortiert: "  << std::endl;
//            for (auto const& circle5 : iterator) {
//               std::cout << circle5 << std::endl;
//            }

         // Calculate the rotation of the square
         adjacent = abs(iterator[0][0] - iterator[1][0]);
         hypotenuse = (iterator[0][0] - iterator[1][0])
               * (iterator[0][0] - iterator[1][0])
               + (iterator[0][1] - iterator[1][1])
                     * (iterator[0][1] - iterator[1][1]);
         hypotenuse = sqrtf(hypotenuse);
         adjacent = acosf((adjacent / hypotenuse));

         middle_point.push_back(cv::Vec3f(middleX, middleY, adjacent));
      }

      iterator.clear();
      middleX = 0.0;
      middleY = 0.0;
      adjacent = 0.0;
      hypotenuse = 0.0;
   }

   std::cout << std::endl << "Mittelpunkt und Rotation: " << std::endl;
   for (auto const& circle6 : middle_point) {
      std::cout << circle6 << std::endl;
   }

   //windows

   cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
   cv::imshow("color", blue_filter);

   cv::namedWindow("erosion", cv::WINDOW_AUTOSIZE);
   cv::imshow("erosion", erosion);

   cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
   cv::imshow("original", image);

   cv::waitKey(0);

   return 0;
}

