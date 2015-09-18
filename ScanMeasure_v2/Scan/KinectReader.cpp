#include <pcl/common/transforms.h>
#include <fstream>
#include "KinectReader.h"
#include "conversion.h"


bool KinectReader::initialize()
{
	openni::Status rc = openni::STATUS_OK;

	printf("Initializing...\n");

	rc = openni::OpenNI::initialize();


	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);
	
	std::cout<< deviceList.getSize() << " devices are connected!" << std::endl;

	if (deviceList.getSize() == 0)
		return false;

//	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	for (int i = 0; i < deviceList.getSize(); i++)
	{
		openni::Device * m_device = new openni::Device;
		openni::VideoStream * m_depthStream = new openni::VideoStream;
		openni::VideoStream * m_colorStream = new openni::VideoStream;
		rc = m_device->open(deviceList[i].getUri());
		if (rc != openni::STATUS_OK)
		{
			printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return false;
		}

		rc = m_depthStream->create(*m_device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			rc = m_depthStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_depthStream->destroy();
				return false;
			}
		}
		else
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		rc = m_colorStream->create(*m_device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			rc = m_colorStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_colorStream->destroy();
				return false;
			}
		}
		if (!m_depthStream->isValid() || !m_colorStream->isValid())
		{
			printf("No valid streams. Exiting\n");
			openni::OpenNI::shutdown();
			return false;
		}
		m_depthStreams.push_back(m_depthStream);
		m_colorStreams.push_back(m_colorStream);
		m_devices.push_back(m_device);
		openni::VideoMode mode = m_colorStream->getVideoMode();
		depthHeight = colorHeight = mode.getResolutionY();
		depthWidth = colorWidth = mode.getResolutionX();
		if (m_mode != SLAM)
		{
			int * i2p = new int[colorHeight * colorWidth];
			for (int depthY = 0; depthY < depthHeight; depthY++)
				for (int depthX = 0; depthX < depthWidth; depthX++)
					i2p[depthY *  depthWidth + depthX] = -1;
			image2Point.push_back(i2p);
		}
		else
		{
			lastI2P = new int[colorHeight * colorWidth];
			for (int depthY = 0; depthY < depthHeight; depthY++)
				for (int depthX = 0; depthX < depthWidth; depthX++)
					lastI2P[depthY *  depthWidth + depthX] = -1;
			curI2P = new int[colorHeight * colorWidth];
			for (int depthY = 0; depthY < depthHeight; depthY++)
				for (int depthX = 0; depthX < depthWidth; depthX++)
					curI2P[depthY *  depthWidth + depthX] = -1;
			image2Point.push_back(lastI2P);
			image2Point.push_back(curI2P);
		}
		show.push_back(true);
		Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
		rt.push_back(trans);
		if (m_mode == SLAM)
		{
			Eigen::Matrix4f initialPos = Eigen::Matrix4f::Identity();
			cameraPos.push_back(initialPos);
		}
	}
	printf("Initialization Succeed\n");
	return true;
}

bool KinectReader::grab_data()
{

	if (m_mode != SLAM)
	{
		clouds.clear();
		for (int i = 0; i < pImages.size(); i++)
			delete pImages[i];
		pImages.clear();
		depths.clear();
		for (int i = 0; i < image2Point.size(); i++)
		{
			for (int depthY = 0; depthY < depthHeight; depthY++)
				for (int depthX = 0; depthX < depthWidth; depthX++)
					image2Point[i][depthY *  depthWidth + depthX] = -1;
		}
	}
	else
	{
		if (pImages.size() >= 1)
		{
			lastScan = pImages[pImages.size() - 1];
			for (int i = 0; i < pImages.size() - 1; i++)
				delete pImages[i];
			pImages.clear();
			pImages.push_back(lastScan);
			lastI2P = image2Point[1];
			curI2P = image2Point[0];
			for (int depthY = 0; depthY < depthHeight; depthY++)
				for (int depthX = 0; depthX < depthWidth; depthX++)
					curI2P[depthY *  depthWidth + depthX] = -1;
			image2Point.clear();
			image2Point.push_back(lastI2P);
			image2Point.push_back(curI2P);
		}
	}

	for (int i = 0; i < m_devices.size(); i++)
	{
		if (!show[i])
			continue;
		openni::VideoFrameRef		depthFrame;
		openni::VideoFrameRef		colorFrame;

		int * i2p;
		if (m_mode != SLAM)
			i2p = image2Point[i];
		else
			i2p = curI2P;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>());
		std::vector<openni::DepthPixel> depth;
		openni::VideoStream * m_depthStream = m_depthStreams[i];
		openni::VideoStream * m_colorStream = m_colorStreams[i];
		if (m_depthStream->isValid() && m_colorStream->isValid())
		{
			m_depthStream->readFrame(&depthFrame);
			m_colorStream->readFrame(&colorFrame);
		}
		else
			return false;

		openni::RGB888Pixel* pImage = (openni::RGB888Pixel*)colorFrame.getData();
		int imageRowSize = colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);
		int depthRowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		const openni::DepthPixel* pDepthImage = (const openni::DepthPixel*)depthFrame.getData();
		for (int depthY = 0; depthY < depthHeight; depthY++)
		{
			for (int depthX = 0; depthX < depthWidth; depthX++)
			{
				float x, y, z;
				const openni::DepthPixel pDepth = pDepthImage[depthY * depthRowSize + depthX];
				if (pDepth >= maxDepth)
					maxDepth = pDepth;
				openni::Status rc = openni::CoordinateConverter::convertDepthToWorld(*m_depthStream, depthX, depthY, pDepth, &x, &y, &z);
				if (x == 0 && y == 0 && z == 0)
					continue;
				int pColorX, pColorY;
				rc = openni::CoordinateConverter::convertDepthToColor(*m_depthStream, *m_colorStream, depthX, depthY, pDepth, &pColorX, &pColorY);
				if (pColorX >= colorWidth || pColorY >= colorHeight)
					continue;
				openni::RGB888Pixel pColor = pImage[pColorY * imageRowSize + pColorX];
				pcl::PointXYZRGB pt;
				pt.x = x;
				pt.y = y;
				pt.z = z;
				pt.r = pColor.r;
				pt.g = pColor.g;
				pt.b = pColor.b;
				i2p[pColorY * imageRowSize + pColorX] = cloud->size();
				cloud->push_back(pt);
				depth.push_back(pDepth);
//					std::cout<<pt.x<<" "<<pt.y<<" " << pt.z << " "<<pt.r << pt.g << pt.b <<std::endl;
			}
		}
		if (m_mode != SLAM)
			pcl::transformPointCloud(*cloud, *cloud, rt[i]);
		clouds.push_back(cloud);
		depths.push_back(depth);
		openni::RGB888Pixel * stored_pImage = new openni::RGB888Pixel[colorWidth * colorHeight];
		memcpy(stored_pImage, pImage, colorWidth * colorHeight * sizeof(openni::RGB888Pixel));
		QImage *img = Conversion::OpenNIImageToQImage(pImage, colorWidth, colorHeight);
		img->save("image.jpg");
		pImages.push_back(stored_pImage);
		if (m_mode == SLAM)
			curScan = stored_pImage;
	}

	
	return true;
}

void KinectReader::savePos()
{
	std::ofstream out("pos.cm");
	/*for (int i = 0; i < cameraPos.size(); i++)
	{
		out << "Pos " << i << ":\n";
		Eigen::Matrix4f pos = cameraPos.at(i);
		Eigen::Matrix3f rotation = pos.block<3,3>(0, 0);
		Eigen::Vector3f translation = pos.block<3,1>(0, 3);
		out << rotation (0,0) << " " << rotation (0,1) << " " << rotation (0,2) << "\n";
		out << rotation (1,0) << " " << rotation (1,1) << " " << rotation (1,2) << "\n";
		out << rotation (2,0) << " " << rotation (2,1) << " " << rotation (2,2) << "\n";
		out << translation (0) << " " << translation (1) << " " << translation (2) << "\n";
	}*/
	for (int i = 0; i < cameraPos.size(); i++)
	{
		Eigen::Matrix4f pos = cameraPos.at(i);
		Eigen::Vector3f translation = pos.block<3,1>(0, 3);
		out << "Vertex " << i+1 << " " << translation (0) << " " << translation (1) << " " << translation (2) << "\n";
	}
	for (int i = 0; i < cameraPos.size() - 1; i++)
		out << "Edge " << i+1 << " " << i+2 << "\n";
	out.close();
}

void KinectReader::saveConfig(const char filename[])
{
	std::ofstream out(filename);
	for (int i = 0; i < rt.size(); i++)
	{
		out << i << "\n";
		Eigen::Matrix4f pos = rt.at(i);
		Eigen::Matrix3f rotation = pos.block<3,3>(0, 0);
		Eigen::Vector3f translation = pos.block<3,1>(0, 3);
		out << rotation (0,0) << " " << rotation (0,1) << " " << rotation (0,2) << "\n";
		out << rotation (1,0) << " " << rotation (1,1) << " " << rotation (1,2) << "\n";
		out << rotation (2,0) << " " << rotation (2,1) << " " << rotation (2,2) << "\n";
		out << translation (0) << " " << translation (1) << " " << translation (2) << "\n";
	}
	out.close();
}

void KinectReader::loadConfig(const char filename[])
{
	std::ifstream in(filename);
	if (!in.is_open())
	{
		std::cout << "Cannot load config file" << std::endl;
		return;
	}
	while (!in.eof())
	{
		int device_id;
		in >> device_id;
		Eigen::Matrix4f pos = Eigen::Matrix4f::Identity();
		in >> pos (0,0) >> pos (0,1) >> pos (0,2);
		in >> pos (1,0) >> pos (1,1) >> pos (1,2);
		in >> pos (2,0) >> pos (2,1) >> pos (2,2);
		in >> pos (0,3) >> pos (1,3) >> pos (2,3);
		if (device_id == rt.size())
			rt.push_back(pos);
		if (device_id < rt.size())
			rt[device_id] = pos;
		if (device_id < clouds.size())
			pcl::transformPointCloud(*clouds[device_id], *clouds[device_id], rt[device_id]);
	}
	in.close();
}