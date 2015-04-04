#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace cv;

void Show_Depth_PCL(Mat depth,double Focal_Length);

void Compare_Depth_PCL(Mat depth1,Mat depth2,double Focal_Length);

void Bilateral_Filter(Mat src_depth,Mat &dst_depth,int window_size,double sigma_s,double sigma_r);

void Cross_Bilateral_Filter(Mat src_depth,Mat src_rgb,Mat &dst_depth,int window_size,double sigma_s,double sigma_r);

/*****************************************
function: main
Main function: read images, show images, apply bilateral filtering, show result.
parameters:
	-RGB_Image_Path: Format:8UC3
	-Depth_Image_Path: Format:16UC1
	-Filter_Type: 0 for Bilateral Filter, 1 for Cross Bilateral Filter
	-window_size: window size of the filter
	-sigma_s: standard deviation of the space gaussian kernel
	-sigma_r: standard deviation of the range gaussian kernel
*****************************************/
int main(int argc, char** argv )
{
	if (argc!=7)
	{
		printf("usage: ./pcl_viewer <RGB_Image_Path>(Format:8UC3) <Depth_Image_Path>(Format:16UC1) <Filter_Type>(0 for Bilateral Filter, 1 for Cross Bilateral Filter) <window_size> <sigma_s> <sigma_r>\n");
		return -1;
	}
	int Filter_Type=atoi(argv[3]);
	int window_size=atoi(argv[4]);
	double sigma_s=atof(argv[5]);
	double sigma_r=atof(argv[6]);

	//read images, make sure formats are correct
	Mat rgb;
	rgb=imread(argv[1]);
	if (!rgb.data)
	{
		printf("Not a valid RGB image path\n");
		return -1;
	}
	if (rgb.type()!=16)
	{
		printf("Wrong RGB image type, please use 8UC3\n");
		return -1;
	}
	Mat depth;
	depth=imread(argv[2],-1);
	if (!depth.data)
	{
		printf("Not a valid depth image path\n");
		return -1;
	}
	if (depth.type()!=2)
	{
		printf("Wrong depth image type, please use 16UC1\n");
		return -1;
	}

	//show the raw depth image
	printf("Showing raw depth image.\n");
	Show_Depth_PCL(depth,531.15);
	printf("Showing the RGB image.\n");
	namedWindow("RGB Image",CV_WINDOW_AUTOSIZE);
	imshow("RGB Image",rgb);
	waitKey(0);

	//apply bilateral filter with only depth image, show the result
	if (Filter_Type==0)
	{
		printf("Showing result of bilateral filter with only depth image.\n");
		Mat depth_bf=Mat::zeros(depth.rows,depth.cols,CV_16UC1);
		Bilateral_Filter(depth,depth_bf,window_size,sigma_s,sigma_r);
		Compare_Depth_PCL(depth,depth_bf,531.15);
	}

	//apply cross bilateral filter with only depth image, show the result
	if (Filter_Type==1)
	{
		printf("Showing result of bilateral filter with only depth image.\n");
		Mat depth_cbf=Mat::zeros(depth.rows,depth.cols,CV_16UC1);
		Cross_Bilateral_Filter(depth,rgb,depth_cbf,window_size,sigma_s,sigma_r);
		Compare_Depth_PCL(depth,depth_cbf,531.15);
	}

	return 0;
}

/*****************************************
function: Show_Depth_PCL
Convert depth image to point cloud, and show point cloud in 3D
parameters:
	-depth: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-Focal_Length: focal length of the camera (in pixel), for 640x480 Kinect v1, Focal_Length = 531.15
Other notes:
	-This function assumes camera center is same as image center
*****************************************/
void Show_Depth_PCL(Mat depth,double Focal_Length)
{
	//creat point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& point_cloud=*point_cloud_ptr;

	//convert pixels to 3D points
	for (int i=0;i<depth.rows;i++)
	{
		short* p;
		p=depth.ptr<short>(i);
		for (int j=0;j<depth.cols;j++)
		{
			pcl::PointXYZ point;
			int d=p[j];
			if (d>0)
			{
				point.x=-((double)(j-depth.cols/2))/Focal_Length*d/1000.0;
				point.y=-((double)(i-depth.rows/2))/Focal_Length*d/1000.0;
				point.z=(double)d/1000.0;
				point_cloud.points.push_back(point);
			}
		}
	}
	point_cloud.width=1;
	point_cloud.height=point_cloud.points.size();
	
	//show point cloud in 3D
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud<pcl::PointXYZ>(point_cloud_ptr,"sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

/*****************************************
function: Compare_Depth_PCL
Compare two depth images in 3D
parameters:
	-depth1: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-depth2: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-Focal_Length: focal length of the camera (in pixel), for 640x480 Kinect v1, Focal_Length = 531.15
Other notes:
	-This function assumes camera center is same as image center
*****************************************/
void Compare_Depth_PCL(Mat depth1,Mat depth2,double Focal_Length)
{
	//creat point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& show_cloud=*show_cloud_ptr;

	//convert pixels to 3D points
	for (int i=0;i<depth1.rows;i++)
	{
		short* p;
		p=depth1.ptr<short>(i);
		for (int j=0;j<depth1.cols;j++)
		{
			pcl::PointXYZRGB point;
			int d=p[j];
			if (d>0)
			{
				point.x=-((double)(j-depth1.cols/2))/Focal_Length*d/1000.0;
				point.y=-((double)(i-depth1.rows/2))/Focal_Length*d/1000.0;
				point.z=(double)d/1000.0;
				uint8_t r=255,g=255,b=255;
				uint32_t rgb=((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
				point.rgb=*reinterpret_cast<float*>(&rgb);
				show_cloud.points.push_back(point);
			}
		}
	}
	for (int i=0;i<depth2.rows;i++)
	{
		short* p;
		p=depth2.ptr<short>(i);
		for (int j=0;j<depth2.cols;j++)
		{
			pcl::PointXYZRGB point;
			int d=p[j];
			if (d>0)
			{
				point.x=-((double)(j-depth2.cols/2))/Focal_Length*d/1000.0;
				point.y=-((double)(i-depth2.rows/2))/Focal_Length*d/1000.0;
				point.z=(double)d/1000.0;
				uint8_t r=255,g=0,b=0;
				uint32_t rgb=((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
				point.rgb=*reinterpret_cast<float*>(&rgb);
				show_cloud.points.push_back(point);
			}
		}
	}
	show_cloud.width=1;
	show_cloud.height=show_cloud.points.size();

	//show point cloud in 3D
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> show_color(show_cloud_ptr);
	viewer->addPointCloud(show_cloud_ptr,show_color,"point cloud");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

/*****************************************
function: Bilateral_Filter
Bilateral filtering with only the depth image
parameters:
	-src_depth: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-dst_depth: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-window_size: window size of the filter
	-sigma_s: standard deviation of the space gaussian kernel
	-sigma_r: standard deviation of the range gaussian kernel
*****************************************/
void Bilateral_Filter(Mat src_depth,Mat &dst_depth,int window_size,double sigma_s,double sigma_r)
{
	for (int i=0;i<dst_depth.rows;i++)
	{
		short* psi;
		psi=src_depth.ptr<short>(i);
		short* pdi;
		pdi=dst_depth.ptr<short>(i);
		for (int j=0;j<dst_depth.cols;j++)
		{
			int intensityij=psi[j];
			if (intensityij>0)
			{
				int xmin=(j-window_size/2)>0 ? (j-window_size/2):0;
				int xmax=(j+window_size/2)<(dst_depth.cols-1) ? (j+window_size/2):(dst_depth.cols-1);
				int ymin=(i-window_size/2)>0 ? (i-window_size/2):0;
				int ymax=(i+window_size/2)<(dst_depth.rows-1) ? (i+window_size/2):(dst_depth.rows-1);
				double normalize_factor=0.0;
				double accumulator=0.0;
				for (int m=ymin;m<=ymax;m++)
				{
					short* psm;
					psm=src_depth.ptr<short>(m);
					for (int n=xmin;n<=xmax;n++)
					{
						int intensitymn=psm[n];
						if (intensitymn>0)
						{
							double weight_s=exp(-((double)((m-i)*(m-i)+(n-j)*(n-j))) / (2.0*sigma_s*sigma_s));
							double weight_r=exp(-((double)((intensitymn-intensityij)*(intensitymn-intensityij))) / (2.0*sigma_r*sigma_r));
							double weight=weight_s*weight_r;
							normalize_factor+=weight;
							accumulator+=weight*intensitymn;
						}
					}
				}
				int newintensityij=(int)(accumulator/normalize_factor);
				pdi[j]=newintensityij;
			}
		}
	}
}

/*****************************************
function: Cross_Bilateral_Filter
Cross bilateral filtering on the depth image with rgb image
parameters:
	-src_depth: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-src_rgb: rgb image 8UC3, pixel-ly aligned with src_depth
	-dst_depth: depth image 16UC1, 0 intensity for pixels whose depth are unknown
	-window_size: window size of the filter
	-sigma_s: standard deviation of the space gaussian kernel
	-sigma_r: standard deviation of the range gaussian kernel
*****************************************/
void Cross_Bilateral_Filter(Mat src_depth,Mat src_rgb,Mat &dst_depth,int window_size,double sigma_s,double sigma_r)
{
	for (int i=0;i<dst_depth.rows;i++)
	{
		short* psi;
		psi=src_depth.ptr<short>(i);
		short* pdi;
		pdi=dst_depth.ptr<short>(i);
		uchar* psrgbi;
		psrgbi=src_rgb.ptr<uchar>(i);
		for (int j=0;j<dst_depth.cols;j++)
		{
			int intensityij=psi[j];
			if (intensityij>0)
			{
				int xmin=(j-window_size/2)>0 ? (j-window_size/2):0;
				int xmax=(j+window_size/2)<(dst_depth.cols-1) ? (j+window_size/2):(dst_depth.cols-1);
				int ymin=(i-window_size/2)>0 ? (i-window_size/2):0;
				int ymax=(i+window_size/2)<(dst_depth.rows-1) ? (i+window_size/2):(dst_depth.rows-1);
				double normalize_factor=0.0;
				double accumulator=0.0;
				for (int m=ymin;m<=ymax;m++)
				{
					short* psm;
					psm=src_depth.ptr<short>(m);
					uchar* psrgbm;
					psrgbm=src_rgb.ptr<uchar>(m);
					for (int n=xmin;n<=xmax;n++)
					{
						int intensitymn=psm[n];
						if (intensitymn>0)
						{
							double weight_s=exp(-((double)((m-i)*(m-i)+(n-j)*(n-j))) / (2.0*sigma_s*sigma_s));
							int b_diff=psrgbi[j*3]-psrgbm[n*3];
							int g_diff=psrgbi[j*3+1]-psrgbm[n*3+1];
							int r_diff=psrgbi[j*3+2]-psrgbm[n*3+2];
							double weight_r=exp(-((double)(b_diff*b_diff+g_diff*g_diff+r_diff*r_diff)) / (2.0*sigma_r*sigma_r));
							double weight=weight_s*weight_r;
							normalize_factor+=weight;
							accumulator+=weight*intensitymn;
						}
					}
				}
				int newintensityij=(int)(accumulator/normalize_factor);
				if (normalize_factor>0.1)
				{
					pdi[j]=newintensityij;
				}
			}
		}
	}
}