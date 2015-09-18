#ifndef CONVERSION_H
#define CONVERSION_H

#include <opencv/cv.h>
#include <QtGui/QImage>
#include <OpenNI.h>

class Conversion
{
public:
	static IplImage *QImageToIplImage(const QImage * qImage)  
	{  
		int width = qImage->width();  
		int height = qImage->height();  
		CvSize Size;  
		Size.height = height;  
		Size.width = width;  
		IplImage *IplImageBuffer = cvCreateImage(Size, IPL_DEPTH_8U, 3);  
		for (int y = 0; y < height; ++y)  
		{  
			for (int x = 0; x < width; ++x)  
			{  
				QRgb rgb = qImage->pixel(x, y);  
				cvSet2D(IplImageBuffer, y, x, CV_RGB(qRed(rgb), qGreen(rgb), qBlue(rgb)));  
			}  
		}  
		return IplImageBuffer;  
	}  
	static QImage *OpenNIImageToQImage(openni::RGB888Pixel* pImage, int width, int height)
	{
		QImage *img = new QImage((const unsigned char*)pImage, width,  height, QImage::Format_RGB888);
		return img;
	}
};

#endif