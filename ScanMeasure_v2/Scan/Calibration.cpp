#include "Calibration.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/recognition/impl/cg/geometric_consistency.hpp>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <hash_map>

void Calibration::registerBetweenTwoScan(KinectReader & reader, int id1, int id2)
{
	if (id1 < 0 || id1 >= reader.pImages.size())
		return ;
	if (id2 < 0 || id2 >= reader.pImages.size())
		return ;
	if (id1 == id2)
		return;
	QImage * img1 = Conversion::OpenNIImageToQImage(reader.pImages[id1], reader.colorWidth, reader.colorHeight);
	IplImage * iplImg1 = Conversion::QImageToIplImage(img1);
	cv::Mat matImg1(iplImg1);
	QImage * img2 = Conversion::OpenNIImageToQImage(reader.pImages[id2], reader.colorWidth, reader.colorHeight);
	IplImage * iplImg2 = Conversion::QImageToIplImage(img2);
	cv::Mat matImg2(iplImg2);
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;
	Feature::detectKeypoints_SIFT_Image(matImg1, keypoints1);
	Feature::detectKeypoints_SIFT_Image(matImg2, keypoints2);
	Feature::computeDescriptor_SIFT(matImg1, keypoints1, descriptors1);
	Feature::computeDescriptor_SIFT(matImg2, keypoints2, descriptors2);
	cv::BruteForceMatcher<cv::L2<float> > matcher;
	std::vector<std::vector<cv::DMatch>> matches;
	matcher.knnMatch(descriptors1, descriptors2, matches, 20);

	// drawing the results
//	cv::namedWindow("matches", 1);
//	cv::Mat img_matches;
//	cv::drawMatches(matImg1, keypoints1, matImg2, keypoints2, matches, img_matches);
//	imshow("matches", img_matches);
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	double cg_size_  = 50;
	double cg_thresh_ = 20;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_keypoints1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_keypoints2(new pcl::PointCloud<pcl::PointXYZ>());
	stdext::hash_map<int, int> i2p1, i2p2;
	for (int i = 0; i < keypoints1.size(); i++)
	{
		pcl::PointXYZ pt;
		if (!Feature::MappingFronImageToPointCloud(reader.clouds[id1], keypoints1[i].pt.x, keypoints1[i].pt.y, 
			reader.image2Point[id1], reader.colorWidth, reader.colorHeight, pt))
			continue;
		i2p1[i] = pc_keypoints1->size();
		pc_keypoints1->push_back(pt);
	}
	for (int i = 0; i < keypoints2.size(); i++)
	{
		pcl::PointXYZ pt;
		if (!Feature::MappingFronImageToPointCloud(reader.clouds[id2], keypoints2[i].pt.x, keypoints2[i].pt.y, 
			reader.image2Point[id2], reader.colorWidth, reader.colorHeight, pt))
			continue;
		i2p2[i] = pc_keypoints2->size();
		pc_keypoints2->push_back(pt);
	}
	pcl::CorrespondencesPtr cloud2_cloud1_corrs (new pcl::Correspondences ());
	for (int i = 0; i < matches.size(); i++)
	{
		std::vector<cv::DMatch> matchForEachQuery = matches[i];
		for (int j = 0; j < matchForEachQuery.size(); j++)
		{
			cv::DMatch match = matchForEachQuery[j];
			if (i2p1.find(match.queryIdx) == i2p1.end() || i2p2.find(match.trainIdx) == i2p2.end())
				continue;
			int p1idx = i2p1[match.queryIdx];
			int p2idx = i2p2[match.trainIdx];
			if (fabs(static_cast<float>(reader.depths[reader.depths.size() - 1][p1idx] - reader.depths[reader.depths.size() - 2][p2idx])) < 500 )
			{
				pcl::Correspondence corr (i2p1[match.queryIdx], i2p2[match.trainIdx], match.distance);
				cloud2_cloud1_corrs->push_back (corr);
			}
		}
	}
	std::cout<<cloud2_cloud1_corrs->size()<<std::endl;
	// Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (pc_keypoints1);
		gc_clusterer.setSceneCloud (pc_keypoints2);
		gc_clusterer.setModelSceneCorrespondences (cloud2_cloud1_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}

	int max_corrs = 0;
	for (int i = 0; i < rototranslations.size(); i++)
	{
		//				std::cout << "\n    Instance " << i + 1 << ":" << std::endl;


		// Print the rotation matrix and translation vector
		std::cout<<clustered_corrs[i].size()<<std::endl;
//		if (clustered_corrs[i].size() < 20)
//			continue;
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
		if (max_corrs < clustered_corrs.size())  
		{
			reader.rt[id1] = rototranslations[i] * reader.rt[id1];
			max_corrs = clustered_corrs.size();
		}
	}
	pcl::transformPointCloud(*reader.clouds[id1], *reader.clouds[id1], reader.rt[id1]);
}

bool Calibration::registerBetweenLastAndCurrentScan(KinectReader & reader)
{
	if (reader.clouds.size() < 2)
		return false;
//	clock_t begin = clock();
	QImage * img1 = Conversion::OpenNIImageToQImage(reader.curScan, reader.colorWidth, reader.colorHeight);
	IplImage * iplImg1 = Conversion::QImageToIplImage(img1);
	cv::Mat matImg1(iplImg1);
	QImage * img2 = Conversion::OpenNIImageToQImage(reader.lastScan, reader.colorWidth, reader.colorHeight);
	IplImage * iplImg2 = Conversion::QImageToIplImage(img2);
//	std::cout << float( clock () - begin ) /  CLOCKS_PER_SEC << "\n";
	cv::Mat matImg2(iplImg2);
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;
//	begin = clock();
	Feature::detectKeypoints_SIFT_Image(matImg1, keypoints1);
	Feature::detectKeypoints_SIFT_Image(matImg2, keypoints2);
//	std::cout << float( clock () - begin ) /  CLOCKS_PER_SEC << "\n";
//	begin = clock();
	Feature::computeDescriptor_SIFT(matImg1, keypoints1, descriptors1);
	Feature::computeDescriptor_SIFT(matImg2, keypoints2, descriptors2);
//	std::cout << float( clock () - begin ) /  CLOCKS_PER_SEC << "\n";
	cv::BruteForceMatcher<cv::L2<float> > matcher;
	std::vector<std::vector<cv::DMatch>> matches;
//	begin = clock();
	matcher.knnMatch(descriptors1, descriptors2, matches, 20);
//	std::cout << float( clock () - begin ) /  CLOCKS_PER_SEC << "\n";


	// drawing the results
//	cv::namedWindow("matches", 1);
//	cv::Mat img_matches;
//	cv::drawMatches(matImg1, keypoints1, matImg2, keypoints2, matches, img_matches);
//	imshow("matches", img_matches);
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	double cg_size_  = 50;
	double cg_thresh_ = 20;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_keypoints1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_keypoints2(new pcl::PointCloud<pcl::PointXYZ>());
	stdext::hash_map<int, int> i2p1, i2p2;
//	begin = clock();
	for (int i = 0; i < keypoints1.size(); i++)
	{
		pcl::PointXYZ pt;
		if (!Feature::MappingFronImageToPointCloud(reader.clouds[reader.clouds.size() - 1], keypoints1[i].pt.x, keypoints1[i].pt.y, 
			reader.curI2P, reader.colorWidth, reader.colorHeight, pt))
			continue;
		i2p1[i] = pc_keypoints1->size();
		pc_keypoints1->push_back(pt);
	}
	for (int i = 0; i < keypoints2.size(); i++)
	{
		pcl::PointXYZ pt;
		if (!Feature::MappingFronImageToPointCloud(reader.clouds[reader.clouds.size() - 2], keypoints2[i].pt.x, keypoints2[i].pt.y, 
			reader.lastI2P, reader.colorWidth, reader.colorHeight, pt))
			continue;
		i2p2[i] = pc_keypoints2->size();
		pc_keypoints2->push_back(pt);
	}
	pcl::CorrespondencesPtr cloud2_cloud1_corrs (new pcl::Correspondences ());
	for (int i = 0; i < matches.size(); i++)
	{
		std::vector<cv::DMatch> matchForEachQuery = matches[i];
		for (int j = 0; j < matchForEachQuery.size(); j++)
		{
			cv::DMatch match = matchForEachQuery[j];
			if (i2p1.find(match.queryIdx) == i2p1.end() || i2p2.find(match.trainIdx) == i2p2.end())
				continue;
			int p1idx = i2p1[match.queryIdx];
			int p2idx = i2p2[match.trainIdx];
			if (fabs(static_cast<float>(reader.depths[reader.depths.size() - 1][p1idx] - reader.depths[reader.depths.size() - 2][p2idx])) < 500 )
			{
				pcl::Correspondence corr (i2p1[match.queryIdx], i2p2[match.trainIdx], match.distance);
				cloud2_cloud1_corrs->push_back (corr);
			}
		}
	}
//	std::cout << float( clock () - begin ) /  CLOCKS_PER_SEC << "\n";
	std::cout<<cloud2_cloud1_corrs->size()<<std::endl;
//	begin = clock();
	// Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (pc_keypoints1);
		gc_clusterer.setSceneCloud (pc_keypoints2);
		gc_clusterer.setModelSceneCorrespondences (cloud2_cloud1_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}
	if (rototranslations.size() == 0)
	{
		std::cout << " Lost! " <<std::endl;
		reader.clouds.pop_back();
		reader.depths.pop_back();
		return false;
	}

	int max_corrs = 0;
	for (int i = 0; i < rototranslations.size(); i++)
	{
		//				std::cout << "\n    Instance " << i + 1 << ":" << std::endl;


		// Print the rotation matrix and translation vector
		std::cout<<clustered_corrs[i].size()<<std::endl;
		//		if (clustered_corrs[i].size() < 20)
		//			continue;
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
		if (max_corrs < clustered_corrs.size())  
		{
			reader.rt[0] = rototranslations[i];
			max_corrs = clustered_corrs.size();
		}
	}
	pcl::transformPointCloud(*reader.clouds[reader.clouds.size() - 1], *reader.clouds[reader.clouds.size() - 1], reader.rt[0]);
	Eigen::Matrix4f lastPos = reader.cameraPos[0];
	Eigen::Matrix4f curPos = reader.rt[0].inverse() * lastPos;
	reader.cameraPos.push_back(curPos);
	return true;
}

void Calibration::refine(KinectReader & reader, int id1, int id2, openni::DepthPixel maxDepth)
{
	if (maxDepth == 10000)
		return;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	for (int i = 0; i < reader.clouds[id1]->size(); i++)
	{
		if (reader.depths[id1][i] < maxDepth)
			cloud1->push_back(reader.clouds[id1]->at(i));
	}
	for (int i = 0; i < reader.clouds[id2]->size(); i++)
	{
		if (reader.depths[id2][i] < maxDepth)
			cloud2->push_back(reader.clouds[id2]->at(i));
	}
	if (cloud1->size() < 50 || cloud2->size() < 50)
		return;
	std::cout << cloud1->size() << " " << cloud2->size() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(cloud1);
	icp.setInputTarget(cloud2);
	icp.setMaxCorrespondenceDistance(200);
	icp.align(*final_cloud);
	Eigen::Matrix4f final_trans = icp.getFinalTransformation();
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	reader.rt[id1] = final_trans * reader.rt[id1];
	pcl::transformPointCloud(*reader.clouds[id1], *reader.clouds[id1], final_trans);
	Eigen::Matrix3f rotation = final_trans.block<3,3>(0, 0);
	Eigen::Vector3f translation = final_trans.block<3,1>(0, 3);
	printf ("\n");
	printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	printf("\n");
	printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
}

double Calibration::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (7);
	std::vector<float> sqr_distances (7);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 7, indices, sqr_distances);
		if (nres == 7)
		{
			for (int j = 1; j < 7; j++)
				res += sqrt (sqr_distances[j]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= 6 * n_points;
	}
	return res;
}