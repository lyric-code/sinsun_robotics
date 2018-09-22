/******************************/
/*        立体匹配和测距        */
/******************************/
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include<math.h>
//#include"Thread.h"
#include"ServoControlInterface.h"
extern "C" {
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<math.h>
}

using namespace Pylon;
using namespace std;
using namespace cv;

#define _PI_ 3.1415926

int GrabImg(string limgn, string rimgn)
{
	const uint32_t c_countOfImagesToGrab = 1;
	const size_t c_maxCamerasToUse = 2;
	int exitCode = 0;
	Pylon::PylonAutoInitTerm autoInitTerm;
	try
    {
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		DeviceInfoList_t devices;

		//若没有检测到摄像头则抛出异常
		if ( tlFactory.EnumerateDevices(devices) == 0 )
			throw RUNTIME_EXCEPTION( "No camera present.");
		CInstantCameraArray cameras( min( devices.size(), c_maxCamerasToUse));
		for ( size_t i = 0; i < cameras.GetSize(); ++i)
		{
			cameras[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
			cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName() << endl;
		}

		// 用于pylon到opencv的格式转换
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;
		CPylonImage pylonImage;
		cv::Mat opencvImage;

		CGrabResultPtr ptrGrabResult;
		for ( size_t i = 0; i < cameras.GetSize(); ++i)
		{
			cameras[i].StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);
			while (cameras[i].IsGrabbing())
			{
				cameras[i].RetrieveResult(5000,ptrGrabResult,TimeoutHandling_ThrowException);
				if (ptrGrabResult->GrabSucceeded())
				{
					cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
					cout << "SizeY " << ptrGrabResult->GetHeight() << endl;
					formatConverter.Convert(pylonImage, ptrGrabResult);
					opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

					// 保存图片
					vector<string> img_name;
					img_name.push_back(limgn);
					img_name.push_back(rimgn);
					imwrite(img_name[i], opencvImage);

					//显示采集得到的图像
					namedWindow("OpenCV Display Window", CV_WINDOW_NORMAL);
					imshow("OpenCV Display Window", opencvImage);
					waitKey(2000);
				}
				else
					exitCode = 1;
			}
		}
    }
	// 异常处理
    catch (const GenericException &e)
    {
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }
//    cerr << endl << "Press Enter to exit." << endl;
//    while( cin.get() != '\n');
    PylonTerminate();
    return exitCode;
}
bool comp(Vec3f a, Vec3f b){
     return a[0] < b[0];
}

Vec3f Median(vector<Vec3f> vec, int median_or_average=0)
{
	if (median_or_average==0)	//取中位数
	{
		sort(vec.begin(),vec.end(),comp);
		return vec[vec.size()/2];
	}
	else							//取平均
	{
		Vec3f dst;
		int i = 0;
		for (; i < vec.size(); i++)
		{
			dst += vec[i];
			cout << vec[i] << endl;
		}
		return dst / i;
	}
}


Vec3f HoughDetect(string ori_img_name)
{
	Mat srcImage = imread(ori_img_name);
	Mat midImage, dstImage;
//	imshow("【原始图】", srcImage);

	cvtColor(srcImage, midImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
	GaussianBlur(midImage, midImage, Size(5, 5), 2, 2);

//	//这里的canny边缘检测仅仅用于参数优选，正式使用时hough检测中含有canny
//	Canny(midImage, midImage, 25, 50);
//	imshow("ori_pic", midImage);
//	waitKey(5000);
//	return 0;

	vector<Vec3f> circles;
	HoughCircles(midImage, circles, CV_HOUGH_GRADIENT, 1.5, 2, 50, 55, 20, 30);
	//HoughCircles(dst_img, circles, CV_HOUGH_GRADIENT,
	//	1,		//图像与累加器分辨率比值
	//	2,		//圆心之间的最小距离
	//	40,		//canny边缘检测算子的高阈值，而低阈值为高阈值的一半
	//	50,		//累加器阈值，即圆度
	//	10,		//最小半径
	//	30);	//最大半径
	if (circles.size()==0)
	{
		cout<<"霍夫检测未检测到圆"<<endl;
		return Vec3f(-1,-1,-1) ;
	}
	// 绘制结果并显示
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		//绘制圆心
		circle(midImage, center, 3, Scalar(255, 0, 0), -1, 8, 0);
		//绘制圆轮廓
		circle(midImage, center, radius, Scalar(155, 50, 255), 1, 8, 0);
	}
	namedWindow("dst_pic", CV_WINDOW_NORMAL);
	imshow("dst_pic", midImage);
	waitKey(3000);

	//直接取平均去除异常点的干扰
	Vec3f dst_xyr = Vec3f(0,0,0);
	dst_xyr= Median(circles);
	cout << "实际圆心与半径" << dst_xyr << endl;
	return dst_xyr;
}


Vec3f Distance(string left_img_name, string right_img_name, Vec3f cir_xyr)
{
	const int imageWidth = 1280;                             //摄像头的分辨率
	const int imageHeight = 960;
	Size imageSize = Size(imageWidth, imageHeight);

	Mat rgbImageL, grayImageL;
	Mat rgbImageR, grayImageR;
	Mat rectifyImageL, rectifyImageR;

	Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
	Rect validROIR;

	Mat mapLx, mapLy, mapRx, mapRy;     //映射表
	Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q，这些是返回参数
	Mat xyz;              //三维坐标

	Point origin;         //鼠标按下的起始点
	Rect selection;      //定义矩形选框
//	bool selectObject = false;    //是否选择对象

	/*事先标定好的相机的参数
	fx 0 cx
	0 fy cy
	0 0  1	*/
	Mat cameraMatrixL = (Mat_<double>(3, 3) << 1364.49200, 0, 702.71310,
		0, 1364.32544, 490.78728,
		0, 0, 1);
	Mat distCoeffL = (Mat_<double>(5, 1) << -0.11256, 0.13040, 0.00036, -0.00038, 0.00000);

	Mat cameraMatrixR = (Mat_<double>(3, 3) << 1361.54173, 0, 661.86331,
		0, 1361.56883, 481.31642,
		0, 0, 1);
	Mat distCoeffR = (Mat_<double>(5, 1) << -0.11242, 0.14425, -0.00065, 0.00044, 0.00000);

	Mat T = (Mat_<double>(3, 1) << -100.49276, -0.27423, -0.70800);//T平移向量
	Mat rec = (Mat_<double>(3, 1) << -0.00362, -0.00243, -0.00044);//rec旋转向量
	Mat R;//R 旋转矩阵

	int blockSize = 6, uniquenessRatio = 10, numDisparities = 20;		//回调参数
	Ptr<StereoBM> bm = StereoBM::create(16, 9);

	/*立体校正*/
	Rodrigues(rec, R); //Rodrigues变换，旋转向量->旋转矩阵
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);	//确定图像校正的映射关系
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	/*读取图片*/
	rgbImageL = imread( left_img_name, CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
	rgbImageR = imread( right_img_name, CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

	//imshow("ImageL Before Rectify", grayImageL);		//显示去畸变之前的图像
	//imshow("ImageR Before Rectify", grayImageR);

	/*经过remap之后，左右相机的图像已经共面并且行对准了，整个过程是逆向映射*/
	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);	//最后一个参数表示双线性插值，默认，双指的是二维图像
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	/*立体匹配*/
	bm->setROI1(validROIL);
	bm->setROI2(validROIR);
	//preFilterType：预处理滤波器的类型，主要是用于降低亮度失真（photometric distortions）、消除噪声和增强纹理等, 有两种可选类型：CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应） 或者 CV_STEREO_BM_XSOBEL（水平方向Sobel算子，默认类型）, 该参数为 int 型；
	//preFilterSize：预处理滤波器窗口大小，容许范围是[5, 255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int 型
	bm->setPreFilterCap(61);	//预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值
	bm->setMinDisparity(50);					//最小视差，默认值为0, 可以是负值，int型

	//回调参数
	bm->setBlockSize(2 * blockSize + 5);			//SAD窗口大小，5~21之间为宜
	bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
	bm->setUniquenessRatio(uniquenessRatio);		//uniquenessRatio主要可以防止误匹配

	//去掉干扰点，但是这些条件设置过于严格也会导致大片黑色区域（因纹理过少，散斑效应而无法测距的区域）
	bm->setTextureThreshold(3);			//不考虑纹理过少的地方，因为这些地方容易受随机噪声的干扰——与周边像素倒数过小的点不考虑
	bm->setSpeckleWindowSize(10);		//克服散斑——区域内计算得到的视差差距超过阈值时，该点不考虑
	bm->setSpeckleRange(1);
	bm->setDisp12MaxDiff(-1);			//左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。

	// 视差相关处理
	Mat disp, disp8;
	bm->compute(rectifyImageL, rectifyImageR, disp);//输入图像必须为灰度图
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式,有符号

	// 彩色显示
//	int isColor=1;
//	Mat disparityImage = cv::Mat::zeros(disp.rows, disp.cols, CV_8UC3);
//	if (isColor)
//	{
//		for (int y = 0; y<disp.rows; y++)
//		{
//			for (int x = 0; x<disp.cols; x++)
//			{
//				uchar val = disp8.at<uchar>(y, x);
//				uchar r, g, b;
//				if (val == 0)
//					r = g = b = 0;
//				else
//				{
//					r = 255 - val;
//					g = val < 128 ? val * 2 : (uchar)((255 - val) * 2);
//					b = val;
//				}
//				disparityImage.at<cv::Vec3b>(y, x) = cv::Vec3b(r, g, b);
//			}
//		}
//	}
	cir_xyr[0]-=16;
	//画框并显示
	float roi_ratio =1.6;
	Rect distance_roi(cir_xyr[0]-1.1*roi_ratio*cir_xyr[2],cir_xyr[1]-roi_ratio*cir_xyr[2],2*roi_ratio*cir_xyr[2],2*roi_ratio*cir_xyr[2]);
	rectangle(disp8, distance_roi, Scalar(255,0,0),3,8);
	imshow("disparity", disp8);
	waitKey(5000);
//	while( cin.get() != '\n');

	// 3D重建
	reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
	xyz = xyz * 16;

	//霍夫圆检测得到的圆心位置是低纹理区，不可直接以一点的重投影作为相机坐标系中的结果
	//采用霍夫圆检测的外接矩形内部有效点平均值作为最终结果
	Point cir_xy;
	Vec3f cir_xyz;
	Vec3f sum_xyz = Vec3f(0,0,0);
	int sum=0;
	for(int i=cir_xyr[0]-roi_ratio*cir_xyr[2]; i<=cir_xyr[0]+roi_ratio*cir_xyr[2];i++)
	{
		for(int j=cir_xyr[1]-roi_ratio*cir_xyr[2]; j<=cir_xyr[1]+roi_ratio*cir_xyr[2];j++)
		{
			cir_xy = Point(i,j);
			cir_xyz = xyz.at<Vec3f>(cir_xy);
			if( cir_xyz[2] <1000 )
			{
				sum_xyz +=cir_xyz;
				sum++;
				cout<< "二维点:" <<i<<","<<j<<"	空间点:"<<cir_xyz<<endl;
			}
//			else
//				cout<< "二维点:" <<i<<","<<j<<"	不可测"<<endl;
		}
	}
	sum_xyz /= sum;
	cout <<"目标点在摄像机坐标系位置"<<sum_xyz <<endl;
	return sum_xyz;
}

//该函数仅仅给出8个运动学逆解中的右手上肘非翻腕解
vector<float> Inverse(float T[4][4])
{
	//T = base_mat.inv() * T * tool_mat.inv();		// 去除base与tool的影响
	vector<float> th (6,0.0);	// 初始关节角解矩阵
	// DH参数
	float a1 = 0.04, a2 = 0.315, a3 = 0.07;
	float d1 = 0.33, d4 = 0.31, d6 = 0.07;
	// 齐次位姿矩阵参数
	float nx = T[0][0], ny = T[1][0], nz = T[2][0];
	float ox = T[0][1], oy = T[1][1], oz = T[2][1];
	float ax = T[0][2], ay = T[1][2], az = T[2][2];
	float px = T[0][3], py = T[1][3], pz = T[2][3];

	// th1，手部异构
	float C[3] = { px - ax*d6, py - ay*d6, pz - az*d6 };
	th[0] = atan2(C[1], C[0]);

	// th2与th3，肘部异构
	float A[3] = { a1*cos(th[0]), a1*sin(th[0]), d1 };
	float AC[3] = { C[0] - A[0] , C[1] - A[1], C[2] - A[2] };
	float len_ac = sqrt(pow(AC[0], 2) + pow(AC[1], 2) + pow(AC[2], 2));
	float abc0 = _PI_ - atan(d4 / a3);
	float Cabc1 = (pow(a2, 2) + pow(a3, 2) + pow(d4, 2) - pow(len_ac, 2)) / (2 * a2*sqrt(pow(a3, 2) + pow(d4, 2)));

	if (abs(Cabc1)>1)
	{
		cout << "BC不构成三角形，当前位置处于奇异点" << endl;
		th[0] = 4;		//将th[0]设置为一个明显不合理的值，以判断是否正常返回
		return th;
	}
	float abc1 = acos(Cabc1);
	th[2] = abc0 - abc1;    // 上肘

	float bac0 = acos((pow(a2, 2) - pow(a3, 2) - pow(d4, 2) + pow(len_ac, 2)) / (2 * a2*len_ac));
	float bac1 = atan2(AC[2], sqrt(pow(AC[0], 2) + pow(AC[1], 2)));

	th[1] = bac0 + bac1;

	// th4 / th5 / th6，腕部异构
	float r33 = ay*cos(th[0]) - ax*sin(th[0]);
	float r13 = az*sin(th[1] - th[2]) + ax*cos(th[1] - th[2])*cos(th[0]) + ay*cos(th[1] - th[2])*sin(th[0]);

	float r23 = ax*sin(th[1] - th[2])*cos(th[0]) - az*cos(th[1] - th[2]) + ay*sin(th[1] - th[2])*sin(th[0]);

	float r22 = ox*sin(th[1] - th[2])*cos(th[0]) - oz*cos(th[1] - th[2]) + oy*sin(th[1] - th[2])*sin(th[0]);
	float r21 = nx*sin(th[1] - th[2])*cos(th[0]) - nz*cos(th[1] - th[2]) + ny*sin(th[1] - th[2])*sin(th[0]);

	th[3] = atan2(r33, -r13);    //非翻腕
	th[4] = atan2(sqrt(pow(r13, 2) + pow(r33, 2)), r23);   // 也可以直接用acos(r23)
	th[5] = atan2(-r22, r21);
	cout<<"关节空间角度值：";
	for (size_t i=0;i<th.size();i++)
		cout<<i<<":"<<th[i]<<"	";
	cout<<endl;
	return th;
}

template<class T,int N,int M>
void MatMuti(T (&a)[N][N],T (&b)[N][M],T (&c)[N][M])
{
	//矩阵相乘
	for (int i = 0; i < N; ++i)
	{
		for (int j = 0; j < M; ++j)
		{
			for (int k = 0; k < N; ++k)
			{
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

#define RECORD_LIMIT 15000
#define RECORD_DIM 6
#define AXIS_MAX_SPEED 52428800		//52428800
#define AXIS_MAX_ENCODER 10000000000
#define AXIS_CIRCLE_ENCODER 1048576
int AxisMaxSpeed[MOTOR_CNT];
int PosLimit[MOTOR_CNT];
int NegLimit[MOTOR_CNT];


int ArmMove(vector< vector<float> > thetas, int times[3])
{
	volatile int num = 0;
	int output = 1;
	int AxisSpeedCmd[MOTOR_CNT] = { 0, 0, 0, 0, 0, 0 };
	int step[6] = { 0, 0, 0, 0, 0, 0 };
	int step_length[6] = { 0, 0, 0, 0, 0, 0 };

	uint16_t AxisStatus[6];
	int axis_encoder[6];
	int torqueRatio[6];//各个电机即时力矩与额定数值的比
	int index;
	//减速比与初始编码值
	float reduction_ratio[6] = {-100,121,101,-92.1,60,-60};
	int origin_encoder[6] = {251935, 247623, 874860, 701281, 7222860,54317};//初始encoder数值
	int time_num1 = 250 * times[0] ;			//规划运动时间对应的num值
	int time_num2 = 250 * times[1] ;
	int time_num3 = 250 * times[2] ;
	vector<float> theta;
	int speed_limit = 1000;

	int theta_encoder[6];
	const int inter_xyz_plus_one = thetas.size();
	int dst_encoder[inter_xyz_plus_one][6];
	int stage_num = time_num2/(inter_xyz_plus_one-2);
	int stage_index ;
	//设置目标位置的encoder值
	for(int i=0;i<inter_xyz_plus_one;i++)
	{
		theta=thetas[i];
		//转换至新松坐标系
		float theta_sinsun[6] {theta[0],theta[1]-_PI_/2,-theta[2],theta[3],-(theta[4]-_PI_/2),theta[5]};
		for (index = 0; index<MOTOR_CNT; index++){
			// 与零位相对值
			theta_encoder[index] = theta_sinsun[index] * AXIS_CIRCLE_ENCODER * reduction_ratio[index] / (2*_PI_);
			// 得到绝对值
			dst_encoder[i][index] = theta_encoder[index] + origin_encoder[index];
		}
	}
	//输出各轴绝对数值
	cout << "伞柄顶端编码值（已乘减速比）：" <<endl;
	cout << "第" <<index<< "轴：" <<dst_encoder[0][index]<<"	";
	cout << endl;

	//设置一系列位置与速度限制
	servoControllerInit();
	for (index = 0; index < MOTOR_CNT; index++) {
		AxisMaxSpeed[index] = AXIS_MAX_SPEED;
		PosLimit[index] = AXIS_MAX_ENCODER;
		NegLimit[index] = -AXIS_MAX_ENCODER;
	}
	setAxisSpeedLimit(AxisMaxSpeed);
	setAxisEncoderLimit(PosLimit, NegLimit);

	pthread_t cyclsend_id, cyclreciev_id, switchmod_id;
	CreateThreads(&cyclsend_id, &cyclreciev_id, &switchmod_id);
	sleep(1);
	servoOn();
	//////////////////////////////////////////第一阶段///////////////////////////////////////////////
	Hold();
	num=0;
	int time_num1_1 = time_num1*2/3;
	while (time_num1>0) {
		getServoStatus(AxisStatus, axis_encoder,torqueRatio);
		//每1s输出一次当前状态
		if (num % 250== 0) {
			cout<<"第一阶段："<<num<<endl;
			for (index = 0; index < 6; index++) {
				cout << "motor\t" << index
						<<"\tStatus\t"<< hex<< AxisStatus[index]
						<<"\tencoder\t"<< dec<< axis_encoder[index]
						<<"\ttorque\t"<< dec<< torqueRatio[index]
						<<"\terr\t"<< dec<< step[index]-axis_encoder[index]
						<< endl;
			}
			cout<<endl;
		}
		//设置当前encoder值
		if (0 == num) {
			// io口设置，控制爪子开合
			output = output << 1;
			setIOData(output);
			uint32_t ioData = getIOData();
			printf("当前IO状态: %d\n", ioData);
		}
		if (0==num || time_num1_1==num)
		{
			for (index = 0; index < 6; index++) {
				step[index] = axis_encoder[index];
			}
			//设置单位时间周期的编码制步进量，限位保护
			for (index = 0; index < 6; index++) {
				if ( 0 == num)
					step_length[index] = (dst_encoder[0][index] - axis_encoder[index])/(reduction_ratio[index]*time_num1_1);
				else
					step_length[index] = (dst_encoder[1][index] - axis_encoder[index])/(reduction_ratio[index]*(time_num1-time_num1_1));
				cout << "前进过程单位时间编码变化速度（未乘减速比）：" <<endl;
				cout << "第"<<index<<"轴："<<step_length[index]<<"	";
				cout << endl;
				if(abs(step_length[index])>500+1000*(index==5))//步进量限制在500
				{
					printf("当前轴超过速度限制\n");
					quickStop();
					sleep(3);
					servoOff();
					sleep(3);
					printf("退出\n");
					return 0;
				}
			}
		}
		//主体程序，在time_use时间内完成移动到目标值
		else if (num < time_num1) {
			for (index = 0; index < 6; index++) {
				if (5 >= index)
					step[index] += step_length[index]*reduction_ratio[index];
			}
			setAbsEncoder(step);
		}

		//到限定时间之后退出
		else if (num >= time_num1) {
			Hold();
			printf("前进动作完成\n");
			break;
		}

		num++;
		usleep(PDO_CYCL_TIME);		//4ms
	}
	/////////////////////////////////////////第二阶段//////////////////////////////////////////////
	num=0;
	while(time_num2>0)
	{
		getServoStatus(AxisStatus, axis_encoder,torqueRatio);
		//每1s输出一次当前状态
		if (num % 250== 0) {
			cout<<"第二阶段："<<num<<endl;
			for (index = 0; index < 6; index++) {
				cout << "motor\t" << index
						<<"\tStatus\t"<< hex<< AxisStatus[index]
						<<"\tencoder\t"<< dec<< axis_encoder[index]
						<<"\ttorque\t"<< dec<< torqueRatio[index]
						<<"\terr\t"<< dec<< step[index]-axis_encoder[index]
						<< endl;
			}
			cout<<endl;
		}
		stage_index = num / stage_num+1;
		if(stage_index>=inter_xyz_plus_one-1)		//防止整除情况出现数组越位访问
			stage_index=inter_xyz_plus_one-2;
		if(num % stage_num == 0)
		{
			//返回过程的路径规划
			for (index = 0; index < 6; index++) {
				step_length[index] = (dst_encoder[stage_index+1][index] - axis_encoder[index])/(reduction_ratio[index]*stage_num);
				cout << "旋拧过程单位时间编码变化速度（未乘减速比）：朝stage_index="<< stage_index<<endl;
				cout << "第"<<index<<"轴："<<step_length[index]<<"	";
				cout << endl;
				if(abs(step_length[index])>500+1000*(index==5))
				{
						printf("当前轴超过速度限制\n");
						quickStop();
						sleep(3);
						servoOff();
						sleep(3);
						printf("退出\n");
						return 0;
				}
			}
		}

		if(num == 0)
		{
			for (index = 0; index < 6; index++) {
				step[index] = axis_encoder[index];
			}
		}
		else if(num < time_num2 ){
			for (index = 0; index < 6; index++) {
				if (5 >= index)
					step[index] += step_length[index]*reduction_ratio[index];
			}
			setAbsEncoder(step);
		}
		else if(num >= time_num2 )
		{
			Hold();
			printf("旋拧动作完成\n");
			break;
		}
		num++;
		usleep(PDO_CYCL_TIME);		//4ms
	}
	/////////////////////////////////////////第三阶段//////////////////////////////////////////////
	num=0;
	while(time_num3>0)
	{
		getServoStatus(AxisStatus, axis_encoder,torqueRatio);
		//每1s输出一次当前状态
		if (num % 250== 0) {
			cout<<"第三阶段："<<num<<endl;
			for (index = 0; index < 6; index++) {
				cout << "motor\t" << index
						<<"\tStatus\t"<< hex<< AxisStatus[index]
						<<"\tencoder\t"<< dec<< axis_encoder[index]
						<<"\ttorque\t"<< dec<< torqueRatio[index]
						<<"\terr\t"<< dec<< step[index]-axis_encoder[index]
						<< endl;
			}
			cout<<endl;
		}
		// 移动完成之后控制爪子开合
		if(num == 0)
		{
			// io口设置，控制爪子开合
			output = output >> 1;
			setIOData(output);
			uint32_t ioData = getIOData();
			printf("当前IO状态: %d\n", ioData);

			for (index = 0; index < 6; index++) {
				step[index] = axis_encoder[index];
			}

			//返回过程的路径规划
			for (index = 0; index < 6; index++) {
				step_length[index] = (origin_encoder[index] - axis_encoder[index])/(reduction_ratio[index]*time_num3);
				cout << "返回过程单位时间编码变化速度（未乘减速比）：" <<endl;
				cout << "第"<<index<<"轴："<<step_length[index]<<"	";
				cout << endl;
				if(abs(step_length[index])>500+1000*(index==5))
				{
					printf("当前轴超过速度限制\n");
					quickStop();
					sleep(3);
					servoOff();
					sleep(3);
					printf("退出\n");
					return 0;
				}
			}
		}
		else if(num < time_num3 ){
			for (index = 0; index < 6; index++) {
				if (5 >= index)
					step[index] += step_length[index]*reduction_ratio[index];
			}
			setAbsEncoder(step);
		}
		else if(num >= time_num3 )
		{
			Hold();
			printf("返回动作完成\n");
			break;
		}
		num++;
		usleep(PDO_CYCL_TIME);		//4ms
	}

	sleep(3);
	servoOff();
	sleep(3);
	printf("quit\n");
	return 0;
}

/*****主函数*****/
int main()
{
	string limgn = "1.jpg", rimgn = "2.jpg";
	float tool_len = 38.1;

	GrabImg(limgn, rimgn);
	Vec3f cir_xyr = HoughDetect(limgn);
	if (cir_xyr[0]<0)
	{
		cout<<"霍夫检测未检测到圆，程序退出"<<endl;
		return 0 ;
	}
	Vec3f cir_xyz = Distance(limgn,rimgn, cir_xyr);

	//两坐标系之间的齐次矩阵
	float cam2cir[4][1] = { cir_xyz[0],cir_xyz[1],cir_xyz[2],1 };
	float robot2cam[4][4] = {{0.0412,0.9987,-0.0292,680.7},
								{0.9987,-0.0420,-0.0299,-36.6},
								{-0.0311,-0.0279,-0.9991,1234.4},
								{0,0,0,1}};
	float robot2cir[4][1] = { 0,0,0,0 };
	MatMuti(robot2cam,cam2cir,robot2cir);
	cout<<"目标点在机械臂坐标系下的坐标: ";
	for (size_t i=0;i<4;i++)
		cout<<i<<":"<<robot2cir[i][0]<<"	";
	cout<<endl;

	float pose_mat[4][4]= {	{1,0,0,robot2cir[0][0]/1000},
								{0,1,0,robot2cir[1][0]/1000},
								{0,0,-1,(robot2cir[2][0]+tool_len)/1000},
								{0,0,0,1}};

	int inter_xyz = 16;				//物理空间插值段数
	int circle_num = 3;				//旋拧过程圈数
	float  forward_dis = 0.009;		//旋拧距离
	float mid_dis = 0.1;

	vector< vector<float> > thetas;
	vector<float> theta;
	vector<float> theta_mid;

	for(int i=0;i <= inter_xyz+1;i++)
	{
		if (0==i)
		{
			pose_mat[2][3] += mid_dis;
			cout<<"中间位置:";
			theta = Inverse(pose_mat);
			pose_mat[2][3] -= mid_dis;
			thetas.push_back(theta);
		}
		else
		{
			theta = Inverse(pose_mat);
			theta[5] +=  i*circle_num*2*_PI_/inter_xyz;
			thetas.push_back(theta);
			pose_mat[2][3] -= forward_dis/inter_xyz;
			cout << pose_mat[2][3]<<" : "<<theta[5]<<endl;
		}
		if (theta[0]>_PI_)
		{
			cout<<"该位置没有有效逆解，程序退出"<<endl;
			return 0;
		}
	}

	int times[3]={15,15,15};
//	vector<float> theta(6,0);
//	theta[1]=_PI_/2;
//	theta[4]=_PI_/2;
	ArmMove(thetas,times);
//	pause();
	return 0;
}
