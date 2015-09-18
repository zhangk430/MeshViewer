#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "KinectReader.h"
#include "Feature_Detection.h"

class Calibration
{
public:
	static void registerBetweenTwoScan(KinectReader & reader, int id1, int id2);
	static bool registerBetweenLastAndCurrentScan(KinectReader & reader);
	static void refine(KinectReader & reader, int id1, int id2, openni::DepthPixel maxDepth);
	static double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
};


#endif