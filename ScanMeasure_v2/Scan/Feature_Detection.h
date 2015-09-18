#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <OpenNI.h>
#include "../Scan/conversion.h"

class Feature
{
public:

	//keypoints
	static void detectKeypoints_SIFT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointWithScale>::Ptr & sifts);
	static void locateKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr sifts, pcl::PointIndices & indices);
	static void detectKeypoints_SIFT_Image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, openni::RGB888Pixel* pImages, int * image2Point, 
		int width, int height, pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints);
	static void detectKeypoints_SIFT_Image(cv::Mat matImg, std::vector<cv::KeyPoint> & keypoints);
	static bool MappingFronImageToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float x, float y, int * image2Point, int width, int height, pcl::PointXYZ & pt);

	//descriptor
	static void computeDescriptor_SIFT(cv::Mat matImg, std::vector<cv::KeyPoint> keypoints, cv::Mat & descriptor);


};



#endif