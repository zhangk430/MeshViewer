#include "Feature_Detection.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <opencv/highgui.h>
#include <time.h>

void Feature::detectKeypoints_SIFT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointWithScale>::Ptr & sifts)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>); 
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift; 
	const float min_scale = 2; 
	const int nr_octaves = 8; 
	const int nr_scales_per_octave = 8; 
	const float min_contrast = 1; 
	sift.setInputCloud(input);
	sift.setSearchMethod (tree);
	sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.compute (*sifts);
}

void Feature::locateKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr sifts, pcl::PointIndices & indices)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (int i = 0; i < input->size(); i++)
	{
		pcl::PointXYZ pt;
		pt.x = input->at(i).x;
		pt.y = input->at(i).y;
		pt.z = input->at(i).z;
		cloud.push_back(pt);
	}
	tree->setInputCloud(cloud.makeShared());
	for (int i = 0; i < sifts->size(); i++)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqrt_dist(1);
		int neigh_num = tree->nearestKSearch(sifts->at(i), 1, neigh_indices, neigh_sqrt_dist);
		if (neigh_num == 1)
			indices.indices.push_back(neigh_indices[0]);
	}
}

void Feature::detectKeypoints_SIFT_Image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, openni::RGB888Pixel* pImages, int * image2Point, 
										 int width, int height,pcl::PointCloud<pcl::PointXYZ>::Ptr & output_keypoints)
{
	clock_t t1, t2;
	t1 = clock();
	QImage *img = Conversion::OpenNIImageToQImage(pImages, width, height);
	IplImage * iplImg = Conversion::QImageToIplImage(img);
	const cv::Mat matImg(iplImg); 

	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("SIFT");
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(matImg, keypoints);
	for (int i = 0; i < keypoints.size(); i++)
	{
		pcl::PointXYZ pt;
		if (!MappingFronImageToPointCloud(input, keypoints[i].pt.x, keypoints[i].pt.y, image2Point, width, height, pt))
			continue;
		output_keypoints->push_back(pt);
	}
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("SIFT");

	// Compute the 128 dimension SIFT descriptor at each keypoint.
	// Each row in "descriptors" correspond to the SIFT descriptor for each keypoint
	cv::Mat descriptors;
	extractor->compute(matImg, keypoints, descriptors);
	t2 = clock();
	float seconds = float(t2 - t1) / CLOCKS_PER_SEC;
	std::cout<<"Total cost: " << seconds << " s\n";

	cv::Mat output;
	cv::drawKeypoints(matImg, keypoints, output);
	cv::imwrite("sift_result.jpg", output);
}

void Feature::detectKeypoints_SIFT_Image(cv::Mat matImg, std::vector<cv::KeyPoint> & keypoints)
{
	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("SIFT");
	detector->detect(matImg, keypoints);
}

void Feature::computeDescriptor_SIFT(cv::Mat matImg, std::vector<cv::KeyPoint> keypoints, cv::Mat & descriptor)
{
	// Compute the 128 dimension SIFT descriptor at each keypoint.
	// Each row in "descriptors" correspond to the SIFT descriptor for each keypoint
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("SIFT");
	extractor->compute(matImg, keypoints, descriptor);
}

bool Feature::MappingFronImageToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float x, float y, int * image2Point, int width, int height, pcl::PointXYZ & pt)
{
	int x1 = static_cast<int> (floor(x));
	int x2 = static_cast<int> (ceil(x));
	int y1 = static_cast<int> (floor(y));
	int y2 = static_cast<int> (ceil(y));
	if (x1 < 0 || y1 < 0 || x2 >= width || y2 >= height)
		return false;
	int p1idx = image2Point[y1 * width + x1];
	if (p1idx < 0 || p1idx >= input->size())
		return false;
	int p2idx = image2Point[y1 * width + x2];
	if (p2idx < 0 || p2idx >= input->size())
		return false;
	int p3idx = image2Point[y2 * width + x1];
	if (p3idx < 0 || p3idx >= input->size())
		return false;
	int p4idx = image2Point[y2 * width + x2];
	if (p4idx < 0 || p4idx >= input->size())
		return false;
	pcl::PointXYZRGB pt1 = input->at(p1idx);
	pcl::PointXYZRGB pt2 = input->at(p2idx);
	pcl::PointXYZRGB pt3 = input->at(p3idx);
	pcl::PointXYZRGB pt4 = input->at(p4idx);
	float w = x - x1;
	float h = y - y1;
	pt.x = (1-w) * (1-h) * pt1.x + w * (1-h) * pt2.x
		+ (1-w) * h * pt3.x + w * h * pt4.x;
	pt.y = (1-w) * (1-h) * pt1.y + w * (1-h) * pt2.y
		+ (1-w) * h * pt3.y + w * h * pt4.y;	
	pt.z = (1-w) * (1-h) * pt1.z + w * (1-h) * pt2.z
		+ (1-w) * h * pt3.z + w * h * pt4.z;
	return true;
}