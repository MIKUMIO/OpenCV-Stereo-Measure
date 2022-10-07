/*
* @name		����˫Ŀ�Ӿ��Ĳ�����
* @brief		����opencv���Լ�˫Ŀ���ʵ�ֲ��
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

#define CAP_FRAME_WIDTH		1280			//���֡���
#define CAP_FRAME_HEIGHT		480			//���֡�߶�
#define CHESSBOARD_SIZE_A	9			//����ÿ���ڽǵ����
#define CHESSBOARD_SIZE_B	6			//����ÿ���ڽǵ����
#define CHESSBOARD_LENGTH   40			//���̸��ӵĳ���(mm)
//#define B					8.550		//���߳���(cm)

int point_x = 0, point_y =0;



void main(void) {
	void PhotoCapture(void);
	void DepthMeasure(void);

	while(1){
		cout << " -------------------------------------------- " << endl;
		cout << "       ��ӭʹ��˫Ŀ��๤����beta�棡         " << endl;
		cout << " -------------------------------------------- " << endl;
		cout << "                                              " << endl;
		cout << "           ����1�򿪱궨ͼ�ɼ�����            " << endl;
		cout << "           ����2����Ȳ�������              " << endl;
		cout << "           ����3��������ַ�                  " << endl;
		cout << "           �����������˳�������               " << endl;
		cout << "                                              " << endl;
		cout << " -------------------------------------------- " << endl;
		cout << endl << endl;

		char selection = '0';
		cin >> selection;
		if (selection == '1')		/*�궨ͼ�ɼ�����*/				
		{
			PhotoCapture();
		}
		else if (selection == '2')	/*��Ȳ�������*/
		{
			DepthMeasure();
		}
		else if (selection == '3')
		{
			system("cls");
		}
		else							/*�˳�����*/
		{
			return;
		}
	}
}



/*�궨ͼ��ɼ�����*/
void PhotoCapture(void){
	ifstream photo_left(".\\PathTxt\\cali_photo_left.txt");					//����ͼ·�������ַ
	ifstream photo_right(".\\PathTxt\\cali_photo_right.txt");				//����ͼ·�������ַ
	string photo_left_name;				//��������ͼ�����ַ
	string photo_right_name;				//��������ͼ�����ַ
	int img_count = 0;
	
	cout << "���ڴ�����ͷ������\n";
	VideoCapture Camera(0);		//��Ĭ������ͷ
	if (!Camera.isOpened())
	{
		cout << "����ͷ�޷��򿪣�\n\n\n\n\n\n\n";
		return;
	}
	Camera.set(CAP_PROP_FRAME_HEIGHT, CAP_FRAME_HEIGHT);		//����֡�߶�Ϊ480
	Camera.set(CAP_PROP_FRAME_WIDTH, CAP_FRAME_WIDTH);		//����֡���Ϊ1280
	cout << "����ͷ�ɹ��򿪣����س���������Ƭ����ESC�˳��궨ͼ�ɼ�����\n";

	while (1) {
		Mat double_img, left_img, right_img;
		Camera >> double_img;		//�������ȡһ֡˫Ŀͼ��
		left_img = double_img(Rect(0, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));						//�ü��������ͼ
		right_img = double_img(Rect(CAP_FRAME_WIDTH / 2, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));		//�ü��������ͼ
		imshow("˫Ŀͼ��", double_img);			//������ʾͼ��

		char inputkey = waitKey(1);		//��ⰴ��
		if (inputkey == 13)							//�س�������ͼ��		
		{
			if (getline(photo_left, photo_left_name) && getline(photo_right, photo_right_name))				//��ȡ�ļ�����·��
			{
				imwrite(photo_left_name, left_img);					//д������ͼƬ
				imwrite(photo_right_name, right_img);
				img_count++;
				cout << "�ɹ���ȡ��" << img_count << "��������ͼ��\n";
			}
			else
			{
				cout << "����·����ȫ���������\n";
				break;
			}
		}
		else if (inputkey == 27)						//���ؼ��˳�����
		{
			break;
		}
	}
	cout << "�������!\n\n\n\n\n\n\n";
	destroyAllWindows();
}



/*��Ȳ�������*/
void DepthMeasure(void) {
	string cali_photo_left = ".\\PathTxt\\cali_photo_left.txt";					//�궨ͼ����·���ļ�
	string cali_photo_right = ".\\PathTxt\\cali_photo_right.txt";
	
	ifstream inputPhotoStreamLeft(cali_photo_left);								//�ļ�������
	ifstream inputPhotoStreamRight(cali_photo_right);


	/****************************************/
	/***************��ʼ������***************/
	cout << "���ڳ�ʼ��������\n";
	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬���Խǵ���������ؾ�ȷ��
	//
	int img_num = 0;													//��ȡͼ�������
	Size img_size(CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT);			//ͼ��ĳߴ�,cv::size size(cols,rows)
	Size boardCornerSize(CHESSBOARD_SIZE_A, CHESSBOARD_SIZE_B);		//�����ڽǵ��������
	vector<Point2f>img_corners_left_each;							//����ÿһ������ͼƬ��⵽�������ڽǵ�����
	vector<vector<Point2f>>img_corners_left_sum;						//������������ͼƬ��⵽���ڽǵ�����
	vector<Point2f>img_corners_right_each;							//����ÿһ��������ͼƬ��⵽�������ڽǵ�����
	vector<vector<Point2f>>img_corners_right_sum;					//������������ͼƬ��⵽���ڽǵ�����
	string photoname_left;											//�����ȡ��ͼƬ·��
	string photoname_right;											//�����ȡ��ͼƬ·��

	while (getline(inputPhotoStreamLeft, photoname_left) && getline(inputPhotoStreamRight, photoname_right))				//���������ж�ȡ�ַ�����������photoname�ÿһ�������Ϊ'\n'
	{
		/*��ȡͼ��*/
		img_num++;												//���ͼƬ�ĳɹ���ȡ
		Mat img_mat_left = imread(photoname_left, 0);			//��ȡ�Ҷ�ͼ
		Mat img_mat_right = imread(photoname_right, 0);			//��ȡ�Ҷ�ͼ
		/*��ȡ�ǵ�*/
		if ((findChessboardCorners(img_mat_left, boardCornerSize, img_corners_left_each) == 0) || (findChessboardCorners(img_mat_right, boardCornerSize, img_corners_right_each) == 0))
		{
			cout << "��ʼ��ʧ��,�궨ͼ�еĵ�" << img_num << "����ͼ�޷���ȡ���ǵ�\n\n\n\n\n\n\n";
			exit(1);
		}
		else
		{
			/*�����ؾ�ȷ��*/
			find4QuadCornerSubpix(img_mat_left, img_corners_left_each, Size(5, 5));			//��ȡ�����ص���Ϣ����߾�ȷ��
			find4QuadCornerSubpix(img_mat_right, img_corners_right_each, Size(5, 5));
			img_corners_left_sum.push_back(img_corners_left_each);									//��һ��ͼƬ�ڽǵ㱣�����ܵĿռ�
			img_corners_right_sum.push_back(img_corners_right_each);
		}
	}
	int total = img_corners_left_sum.size();			//��ȡ�ڽǵ������



	//��ʼ��������ͷ�ı궨
	//
	/*������ά��Ϣ*/
	Size board_square_size(CHESSBOARD_LENGTH, CHESSBOARD_LENGTH);		//ʵ�ʲ����õ�������ͼ��ÿ������Ĵ�С��mm��
	vector<vector<Point3f>> object_points;			
	vector<int>points_counts;
	/*��ʼ���궨���ϵ���ά���꣨�ٶ�ÿ��ͼ����ͼ��������*/
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
				realPoint.z = 0;				//��������ϵxoyƽ��̶���������,z=0
				tempPointSet.push_back(realPoint);			//��һ���ǵ����ά���괢�浽һ��ͼ������
			}
		}
		object_points.push_back(tempPointSet);			//��һ��ͼ����ά���괢�浽�ܵĵ�����
	}
	for (i = 0; i < img_num; i++)			//����ÿһ���Ľǵ�����
	{
		points_counts.push_back(boardCornerSize.height * boardCornerSize.width);
	}
	/*����ͷ�ĸ��ֲ���*/
	Mat cameraMatrix_left = Mat(3, 3, CV_32FC1, Scalar::all(0));				//��������ڲ�������
	Mat distCoeffs_left = Mat(1, 5, CV_32FC1, Scalar::all(0));				//�������5������ϵ��:k1,k2,p1,p2,k3
	vector<Mat>tvecsMat_left;												//���������ÿ��ͼ���ƽ������
	vector<Mat>rvecsMat_left;												//���������ÿ��ͼ�����ת����
	Mat cameraMatrix_right = Mat(3, 3, CV_32FC1, Scalar::all(0));			//��������ڲ�������
	Mat distCoeffs_right = Mat(1, 5, CV_32FC1, Scalar::all(0));				//�������5������ϵ��:k1,k2,p1,p2,k3
	vector<Mat>tvecsMat_right;												//���������ÿ��ͼ���ƽ������
	vector<Mat>rvecsMat_right;												//���������ÿ��ͼ�����ת����
	Mat R;				//�����������ϵ֮�����ת����
	Mat T;				//�����������ϵ֮���ƽ������
	Mat E;				//�������֮��ı�������
	Mat F;				//�������֮��Ļ�������

	/*����calibrateCamera()����������������б궨*/
	calibrateCamera(object_points, img_corners_left_sum, img_size, cameraMatrix_left, distCoeffs_left, rvecsMat_left, tvecsMat_left);			//����opencv�����궨�����
	calibrateCamera(object_points, img_corners_right_sum, img_size, cameraMatrix_right, distCoeffs_right, rvecsMat_right, tvecsMat_right);		//����opencv�����궨�����
	/*����stereoCalibrate()�����궨���������֮��Ĳ���*/
	stereoCalibrate(object_points, img_corners_left_sum, img_corners_right_sum, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, img_size, R, T, E, F, CALIB_USE_INTRINSIC_GUESS);
	
	//ͼ��У����������
	// 
	/*����У���任����*/
	Mat R1, R2, P1, P2, Q;
	stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, img_size, R, T, R1, R2, P1, P2, Q, 0);
	/*����У��ӳ�����*/
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, img_size, CV_16SC2, map21, map22);

	cout << "��ʼ����ϡ�����\n";
	/***************��ʼ������***************/
	/****************************************/

	//�����׶�
	//
	VideoCapture Camera(0);		//��Ĭ������ͷ
	if (!Camera.isOpened())
	{
		cout << "����ͷ�޷��򿪣�\n\n\n\n\n\n\n";
		return;
	}
	Camera.set(CAP_PROP_FRAME_HEIGHT, CAP_FRAME_HEIGHT);			//����֡�߶�Ϊ480
	Camera.set(CAP_PROP_FRAME_WIDTH, CAP_FRAME_WIDTH);			//����֡���Ϊ1280
	cout << "����ͷ�ɹ��򿪣����س���������Ƭ,���ؼ��˳���Ȳ�������\n";

	while (1) 
	{
		while (1) 
		{
			Mat frame, frame_left, frame_right, frame_left_c, frame_right_c;
			Camera >> frame;																				//�������ȡһ֡˫Ŀͼ��
			frame_left = frame(Rect(0, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));						//�ü��������ͼ
			frame_right = frame(Rect(CAP_FRAME_WIDTH / 2, 0, CAP_FRAME_WIDTH / 2, CAP_FRAME_HEIGHT));	//�ü��������ͼ
			remap(frame_left, frame_left_c, map11, map12, INTER_LINEAR);									//ӳ��У������ͼ
			remap(frame_right, frame_right_c, map21, map22, INTER_LINEAR);								//ӳ��У������ͼ
			hconcat(frame_left_c, frame_right_c, frame);													//�ϲ�������ͼ		
			imshow("Camera", frame);																		//������ʾͼ��

			char key = waitKey(1);									//�жϼ�ֵ
			if (key == 13)
			{
				imwrite(".\\temp_left_c.bmp", frame_left_c);				//�س���������ʱͼ��
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
		corrected_frame_left = imread(".\\temp_left_c.bmp");				//��ȡ��ʱͼ��
		corrected_frame_right = imread(".\\temp_right_c.bmp");

		int cn = corrected_frame_left.channels();
		Ptr<StereoSGBM> matcher = StereoSGBM::create(0, 128, 3, 8 * cn * 3 * 3, 32 * cn * 3 * 3, 2, 63, 15, 100, 1, StereoSGBM::MODE_SGBM_3WAY);//����SGBM��

		matcher->setMinDisparity(0);					//��С�Ӳ�ֵ
		matcher->setNumDisparities(128);				//�Ӳ�ڣ�����Ӳ�ֵ����С�Ӳ�ֵ֮�Ϊ16�ı���
		matcher->setBlockSize(9);					//ƥ����С��3-11������
		matcher->setP1(8 * cn * 9 * 9);				//ƽ������P1
		matcher->setP2(32 * cn * 9 * 9);				//ƽ������P2,Խ��Խƽ����P2�������P1
		matcher->setDisp12MaxDiff(1);				//���Ӳ�ͼ��ֱ�Ӽ���ó��������Ӳ�ͼ��ͨ��cvValidateDisparity����ó���֮������������졣��������ֵ���Ӳ�ֵ�������㡣����Ϊ-1��ִ�м��
		matcher->setPreFilterCap(63);				//Ԥ�����˲����Ľض�ֵ
		matcher->setUniquenessRatio(12);				//�Ӳ�Ψһ�԰ٷֱ�,һ��ȡ5-15
		matcher->setSpeckleWindowSize(125);			//��ͨ���򴰿ڴ�С��һ��ȡ50-200��0Ϊȡ����ͨ�����飬�ô������ڹ���С�����������ȣ�
		matcher->setSpeckleRange(1);					//���������Ӳ�仯���ڸ���ֵ(��16��)ʱ���ô����ڵ��Ӳ�Ϊ0,1����2�Ѿ��㹻���ˡ�
		matcher->setMode(StereoSGBM::MODE_SGBM);		//ƥ��ģʽ

		matcher->compute(corrected_frame_left, corrected_frame_right, delta);
		delta.convertTo(delta, CV_32F, 1.0 / 16.0);			//����16�õ���ʵ�Ӳ�ֵ

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

		/*��ȡ���������겢���������*/
		void mouse(int event, int x, int y, int flags, void*);		//�������ص�����
		while (1)
		{
			while (1)
			{
				imshow("img", corrected_frame_left);
				imshow("�Ӳ�ͼ",post_delta);
				setMouseCallback("�Ӳ�ͼ", mouse);					//�������ص�����
				int a = waitKey(1);
				if (a == 13)
					break;
			}
			
			//float d = delta.at<short>(point_y, point_x) / 32.0;
			float d = delta.at<float>(point_y, point_x);		//��ȡ�Ӳ�ֵ
			double fx = (cameraMatrix_left.at<double>(0, 0) + cameraMatrix_right.at<double>(0, 0)) / 2.0;		//��ȡ����
			double B = (double)abs(T.at<double>(0));		//��ȡ���߳�(mm)
			if (d > 0)
			{
				
				cout << "�������Ϊ " << (fx * B) /(d*10) << "cm\n\n\n\n";		//z=(f*B)/(XR-XL)
				destroyAllWindows();
				break;
			}
			else
			{
				cout << "������ѡ��λ��\n";
			}
		}

		remove(".\\temp_left.bmp");				//ɾ����ʱͼ��
		remove(".\\temp_right.bmp");
	}
	
}


/*�������ص�����*/
void mouse(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN)
	{
		point_x = x;
		point_y = y;
	}
}


#endif