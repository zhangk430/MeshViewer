#ifndef KINECT_READER_H
#define KINECT_READER_H

#include <OpenNI.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class KinectReader
{

public:
	enum READER_MODE
	{
		SLAM,
		BODY_SCAN,
	};
	KinectReader() : m_mode(SLAM), maxDepth(0){}
	~KinectReader(){
		for (int i = 0; i < pImages.size(); i++)
			delete pImages[i];
		for (int i = 0; i < image2Point.size(); i++)
			delete image2Point[i];
	}


	bool initialize();

	bool grab_data();

	void savePos();

	void saveConfig(const char filename[]);
	void loadConfig(const char filename[]);

	std::vector<openni::Device *>			m_devices;
	std::vector<openni::VideoStream *>    m_depthStreams;
	std::vector<openni::VideoStream *>    m_colorStreams;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rt;
	std::vector<bool> show;

	int depthWidth;
	int depthHeight;
	int colorWidth;
	int colorHeight;
	READER_MODE m_mode;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;   //store the point clouds
	std::vector<std::vector<openni::DepthPixel>> depths;
	std::vector<openni::RGB888Pixel* > pImages;                   //store the images
	std::vector<int *> image2Point;
	openni::DepthPixel maxDepth;


	//SLAM
	openni::RGB888Pixel* lastScan, *curScan;
	int * lastI2P, * curI2P;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > cameraPos;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif