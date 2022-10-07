/*
* @name		基于双目视觉的测距程序
* @brief		利用opencv库以及双目相机实现测距
* @author	MIKU
* @vertion	beta1.0
* @date		2022-8-30
*/

#if 1



#include<opencv.hpp>
#include<iostream>
#include<fstream>
#include<direct.h>

using namespace cv;
using namespace std;

#define CAP_FRAME_WIDTH		1280			//相机帧宽度
#define CAP_FRAME_HEIGHT		480			//相机帧高度
#define CHESSBOARD_SIZE_A	9			//棋盘每行内角点个数
#define CHESSBOARD_SIZE_B	6			//棋盘每列内角点个数
#define CHESSBOARD_LENGTH   40			//棋盘格子的长度(mm)
//#define B					8.550		//基线长度(cm)

int point_x = 0, point_y =0;



void main(void) {
	void PhotoCapture(void);
	void DepthMeasure(void);

	while(1){
		cout << " -------------------------------------------- " << endl;
		cout << "       欢迎使用双目测距工具箱beta版！         " << endl;
		cout << " -------------------------------------------- " << endl;
		cout << "                                              " << endl;
		cout << "           输入1打开标定图采集程序            " << endl;
		cout << "           输入2打开深度测量程序              " << endl;
		cout << "           输入3清理输出字符                  " << endl;
		cout << "           输入其他键退出工具箱               " << endl;
		cout << "                                              " << endl;
		cout << " -------------------------------------------- " << endl;
		cout << endl << endl;

		char selection = '0';
		cin >> selection;
		if (selection == '1')		/*标定图采集程序*/				
		{
			PhotoCapture();
		}
		else if (selection == '2')	/*深度测量程序*/
		{
			DepthMeasure();
		}
		else if (selection == '3')
		{
			system("cls");
		}
		else							/*退出程序*/
		{
			return;
		}
	}
}



/*标定图像采集程序*/
void PhotoCapture(void){
	ifstream photo_left(".\\PathTxt\\cali_photo_left.txt");					//左视图路径储存地址
	ifstream photo_right(".\\PathTxt\\cali_photo_right.txt");				//右视图路径储存地址
	string photo_left_name;				//缓存左视图储存地址
	string photo_right_name;				//缓存右视图储存地址
	int img_count = 0;
	
	cout << "正在打开摄像头。。。\n";
	VideoCapture Camera(0);		//打开默认摄像头
	if (!Camera.isOpened())
	{
		cout << "摄像头无法打开！\n\n\n\n\n\n\n";
		return;
	}
	Camera.set(CAP_PROP_FRAME_HEIGHT, CAP_FRAME_HEIGHT);		//设置帧高度为480
	Camera.set(CAP_PROP_FRAME_WIDTH, CAP_FRAME_WIDTH);		//设置帧宽度为1280
	cout << "摄像头成功打开，按回车键拍摄照片，按ESC退出标定图采集程序\n";

	while (1) {
		Mat double_img, left_img, right_img;
		Camera >> double_img;		//从摄像获取一帧双目图像
		left_img = double_img(Rect(0, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));						//裁剪获得左视图
		right_img = double_img(Rect(CAP_FRAME_WIDTH / 2, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));		//裁剪获得右视图
		imshow("双目图像", double_img);			//窗口显示图像

		char inputkey = waitKey(1);		//检测按键
		if (inputkey == 13)							//回车键储存图像		
		{
			if (getline(photo_left, photo_left_name) && getline(photo_right, photo_right_name))				//读取文件储存路径
			{
				imwrite(photo_left_name, left_img);					//写入左右图片
				imwrite(photo_right_name, right_img);
				img_count++;
				cout << "成功获取第" << img_count << "套左右视图！\n";
			}
			else
			{
				cout << "所有路径已全部储存完毕\n";
				break;
			}
		}
		else if (inputkey == 27)						//返回键退出程序
		{
			break;
		}
	}
	cout << "拍摄完毕!\n\n\n\n\n\n\n";
	destroyAllWindows();
}



/*深度测量程序*/
void DepthMeasure(void) {
	string cali_photo_left = ".\\PathTxt\\cali_photo_left.txt";					//标定图储存路径文件
	string cali_photo_right = ".\\PathTxt\\cali_photo_right.txt";
	
	ifstream inputPhotoStreamLeft(cali_photo_left);								//文件输入流
	ifstream inputPhotoStreamRight(cali_photo_right);


	/****************************************/
	/***************初始化过程***************/
	cout << "正在初始化。。。\n";
	//读取每一幅图像，从中提取出角点，并对角点进行亚像素精确化
	//
	int img_num = 0;													//读取图像的数量
	Size img_size(CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT);			//图像的尺寸,cv::size size(cols,rows)
	Size boardCornerSize(CHESSBOARD_SIZE_A, CHESSBOARD_SIZE_B);		//棋盘内角点的行列数
	vector<Point2f>img_corners_left_each;							//缓存每一张左视图片检测到的所有内角点坐标
	vector<vector<Point2f>>img_corners_left_sum;						//储存所有左视图片检测到的内角点坐标
	vector<Point2f>img_corners_right_each;							//缓存每一张右视视图片检测到的所有内角点坐标
	vector<vector<Point2f>>img_corners_right_sum;					//储存所有右视图片检测到的内角点坐标
	string photoname_left;											//储存读取的图片路径
	string photoname_right;											//储存读取的图片路径

	while (getline(inputPhotoStreamLeft, photoname_left) && getline(inputPhotoStreamRight, photoname_right))				//从输入流中读取字符，并储存在photoname里，每一句结束符为'\n'
	{
		/*读取图像*/
		img_num++;												//标记图片的成功读取
		Mat img_mat_left = imread(photoname_left, 0);			//读取灰度图
		Mat img_mat_right = imread(photoname_right, 0);			//读取灰度图
		/*提取角点*/
		if ((findChessboardCorners(img_mat_left, boardCornerSize, img_corners_left_each) == 0) || (findChessboardCorners(img_mat_right, boardCornerSize, img_corners_right_each) == 0))
		{
			cout << "初始化失败,标定图中的第" << img_num << "套视图无法提取到角点\n\n\n\n\n\n\n";
			exit(1);
		}
		else
		{
			/*亚像素精确化*/
			find4QuadCornerSubpix(img_mat_left, img_corners_left_each, Size(5, 5));			//提取亚像素点信息，提高精确度
			find4QuadCornerSubpix(img_mat_right, img_corners_right_each, Size(5, 5));
			img_corners_left_sum.push_back(img_corners_left_each);									//将一副图片内角点保存至总的空间
			img_corners_right_sum.push_back(img_corners_right_each);
		}
	}
	int total = img_corners_left_sum.size();			//获取内角点的组数



	//开始进行摄像头的标定
	//
	/*棋盘三维信息*/
	Size board_square_size(CHESSBOARD_LENGTH, CHESSBOARD_LENGTH);		//实际测量得到的棋盘图上每个方格的大小（mm）
	vector<vector<Point3f>> object_points;			
	vector<int>points_counts;
	/*初始化标定板上的三维坐标（假定每幅图棋盘图都完整）*/
	int i, j, t;
	for (t = 0; t < img_num; t++)
	{
		vector<Point3f>tempPointSet;
		for (i = 0; i < boardCornerSize.height; i++)
		{
			for (j = 0; j < boardCornerSize.width; j++)
			{
				Point3f realPoint;
				realPoint.x = i * board_square_size.width;
				realPoint.y = j * board_square_size.height;
				realPoint.z = 0;				//世界坐标系xoy平面固定于棋盘上,z=0
				tempPointSet.push_back(realPoint);			//把一个角点的三维坐标储存到一副图的里面
			}
		}
		object_points.push_back(tempPointSet);			//把一副图的三维坐标储存到总的的里面
	}
	for (i = 0; i < img_num; i++)			//保存每一幅的角点数量
	{
		points_counts.push_back(boardCornerSize.height * boardCornerSize.width);
	}
	/*摄像头的各种参数*/
	Mat cameraMatrix_left = Mat(3, 3, CV_32FC1, Scalar::all(0));				//左相机的内参数矩阵
	Mat distCoeffs_left = Mat(1, 5, CV_32FC1, Scalar::all(0));				//左相机的5个畸变系数:k1,k2,p1,p2,k3
	vector<Mat>tvecsMat_left;												//左相机所有每幅图像的平移向量
	vector<Mat>rvecsMat_left;												//左相机所有每幅图像的旋转向量
	Mat cameraMatrix_right = Mat(3, 3, CV_32FC1, Scalar::all(0));			//右相机的内参数矩阵
	Mat distCoeffs_right = Mat(1, 5, CV_32FC1, Scalar::all(0));				//右相机的5个畸变系数:k1,k2,p1,p2,k3
	vector<Mat>tvecsMat_right;												//右相机所有每幅图像的平移向量
	vector<Mat>rvecsMat_right;												//右相机所有每幅图像的旋转向量
	Mat R;				//两个相机坐标系之间的旋转矩阵
	Mat T;				//两个相机坐标系之间的平移向量
	Mat E;				//两个相机之间的本征矩阵
	Mat F;				//两个相机之间的基本矩阵

	/*利用calibrateCamera()函数对左右相机进行标定*/
	calibrateCamera(object_points, img_corners_left_sum, img_size, cameraMatrix_left, distCoeffs_left, rvecsMat_left, tvecsMat_left);			//调用opencv函数标定左相机
	calibrateCamera(object_points, img_corners_right_sum, img_size, cameraMatrix_right, distCoeffs_right, rvecsMat_right, tvecsMat_right);		//调用opencv函数标定左相机
	/*利用stereoCalibrate()函数标定两个摄像机之间的参数*/
	stereoCalibrate(object_points, img_corners_left_sum, img_corners_right_sum, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, img_size, R, T, E, F, CALIB_USE_INTRINSIC_GUESS);
	
	//图像校正参数计算
	// 
	/*计算校正变换矩阵*/
	Mat R1, R2, P1, P2, Q;
	stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, img_size, R, T, R1, R2, P1, P2, Q, 0);
	/*计算校正映射矩阵*/
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, img_size, CV_16SC2, map21, map22);

	cout << "初始化完毕。。。\n";
	/***************初始化结束***************/
	/****************************************/

	//测量阶段
	//
	VideoCapture Camera(0);		//打开默认摄像头
	if (!Camera.isOpened())
	{
		cout << "摄像头无法打开！\n\n\n\n\n\n\n";
		return;
	}
	Camera.set(CAP_PROP_FRAME_HEIGHT, CAP_FRAME_HEIGHT);			//设置帧高度为480
	Camera.set(CAP_PROP_FRAME_WIDTH, CAP_FRAME_WIDTH);			//设置帧宽度为1280
	cout << "摄像头成功打开，按回车键拍摄照片,返回键退出深度测量程序\n";

	while (1) 
	{
		while (1) 
		{
			Mat frame, frame_left, frame_right, frame_left_c, frame_right_c;
			Camera >> frame;																				//从摄像获取一帧双目图像
			frame_left = frame(Rect(0, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));						//裁剪获得左视图
			frame_right = frame(Rect(CAP_FRAME_WIDTH / 2, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));	//裁剪获得右视图
			remap(frame_left, frame_left_c, map11, map12, INTER_LINEAR);									//映射校正左视图
			remap(frame_right, frame_right_c, map21, map22, INTER_LINEAR);								//映射校正右视图
			hconcat(frame_left_c, frame_right_c, frame);													//合并左右视图		
			imshow("Camera", frame);																		//窗口显示图像

			char key = waitKey(1);									//判断键值
			if (key == 13)
			{
				imwrite(".\\temp_left_c.bmp", frame_left_c);				//回车键保存临时图像
				imwrite(".\\temp_right_c.bmp", frame_right_c);
				destroyAllWindows();
				break;
			}
			else if (key == 27)
			{
				destroyAllWindows();
				return;
			}
		}

		Mat corrected_frame_left, corrected_frame_right, delta;
		corrected_frame_left = imread(".\\temp_left_c.bmp");				//读取临时图像
		corrected_frame_right = imread(".\\temp_right_c.bmp");

		int cn = corrected_frame_left.channels();
		Ptr<StereoSGBM> matcher = StereoSGBM::create(0, 128, 3, 8 * cn * 3 * 3, 32 * cn * 3 * 3, 2, 63, 15, 100, 1, StereoSGBM::MODE_SGBM_3WAY);//创建SGBM类

		matcher->setMinDisparity(0);					//最小视差值
		matcher->setNumDisparities(128);				//视差窗口，最大视差值与最小视差值之差，为16的倍数
		matcher->setBlockSize(9);					//匹配块大小，3-11的奇数
		matcher->setP1(8 * cn * 9 * 9);				//平滑参数P1
		matcher->setP2(32 * cn * 9 * 9);				//平滑参数P2,越大越平滑，P2必须大于P1
		matcher->setDisp12MaxDiff(1);				//左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。设置为-1不执行检查
		matcher->setPreFilterCap(63);				//预处理滤波器的截断值
		matcher->setUniquenessRatio(12);				//视差唯一性百分比,一般取5-15
		matcher->setSpeckleWindowSize(125);			//连通区域窗口大小，一般取50-200，0为取消连通区域检查，该窗口用于过滤小的区域（噪声等）
		matcher->setSpeckleRange(1);					//当窗口内视差变化大于该阈值(的16倍)时，该窗口内的视差为0,1或者2已经足够好了。
		matcher->setMode(StereoSGBM::MODE_SGBM);		//匹配模式

		matcher->compute(corrected_frame_left, corrected_frame_right, delta);
		delta.convertTo(delta, CV_32F, 1.0 / 16.0);			//除以16得到真实视差值

		Mat post_delta(CAP_FRAME_HEIGHT, CAP_FRAME_WIDTH/2, CV_8UC3);
		double max = 0;
		minMaxLoc(delta, nullptr, &max);
		for (int row = 0; row < post_delta.rows; row++)
		{
			for (int col = 0; col < post_delta.cols; col++)
			{
				int elem = (int)delta.at<float>(row, col);
				if (elem <0)
				{
					post_delta.at<Vec3b>(row, col) = Vec3b(0, 0, 0);
				}
				else
				{
					double degree = (double)elem / max;
					uchar color = (uchar)(degree * 255);
					post_delta.at<Vec3b>(row, col) = Vec3b(color, color, 0);
				}
			}
		}

		/*读取测量的坐标并计算出距离*/
		void mouse(int event, int x, int y, int flags, void*);		//鼠标操作回调函数
		while (1)
		{
			while (1)
			{
				imshow("img", corrected_frame_left);
				imshow("视差图",post_delta);
				setMouseCallback("视差图", mouse);					//设置鼠标回调函数
				int a = waitKey(1);
				if (a == 13)
					break;
			}
			
			//float d = delta.at<short>(point_y, point_x) / 32.0;
			float d = delta.at<float>(point_y, point_x);		//读取视差值
			double fx = (cameraMatrix_left.at<double>(0, 0) + cameraMatrix_right.at<double>(0, 0)) / 2.0;		//读取焦距
			double B = (double)abs(T.at<double>(0));		//读取基线长(mm)
			if (d > 0)
			{
				
				cout << "所测深度为 " << (fx * B) /(d*10) << "cm\n\n\n\n";		//z=(f*B)/(XR-XL)
				destroyAllWindows();
				break;
			}
			else
			{
				cout << "请重新选择位置\n";
			}
		}

		remove(".\\temp_left.bmp");				//删除临时图像
		remove(".\\temp_right.bmp");
	}
	
}


/*鼠标操作回调函数*/
void mouse(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN)
	{
		point_x = x;
		point_y = y;
	}
}


#endif