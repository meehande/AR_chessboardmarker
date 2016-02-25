#include "Utilities.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/calib3d.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

#define HOME_IM 0
#define CHESSBOARD_OBJECT 1
#define SAMPLE_FRAME 2

Mat createExtrinsic(Mat rotationMatrix, Mat tvec);
Mat cvToglModelView(Mat cvMatrix);

int main(int argc, const char** argv)
{
	char* file_location = "Media/";
	char* image_files[] = {


		"TrinityRegentHouse.jpg", //0
		"OpenCV_Chessboard.png", //1
		"c4.jpg",//2
		"chess_object.jpg",
		"chessboard_object.jpg"
		

		
    };
	// Load images
	int number_of_images = sizeof(image_files)/sizeof(image_files[0]);
	Mat* image = new Mat[number_of_images];
	for (int file_no=0; (file_no < number_of_images); file_no++)
	{
		string filename(file_location);
		filename.append(image_files[file_no]);
		image[file_no] = imread(filename, -1);
		if (image[file_no].empty())
		{
			cout << "Could not open " << image[file_no] << endl;
			return -1;
		}
	}
	// Load video(s)
	char* video_files[] = { 
		"chessboard3.avi",
		"chessboard.avi",
		"chessboard2.avi"
		 };
	int number_of_videos = sizeof(video_files)/sizeof(video_files[0]);
	VideoCapture* video = new VideoCapture[number_of_videos];
	for (int video_file_no=0; (video_file_no < number_of_videos); video_file_no++)
	{
		string filename(file_location);
		filename.append(video_files[video_file_no]);
		video[video_file_no].open(filename);
		if( !video[video_file_no].isOpened() )
		{
			cout << "Cannot open video file: " << filename << endl;
			return -1;
		}
	}
	

	Mat default_image = ComputeDefaultImage( image[0] );
	cout << "OpenCV version : " << CV_VERSION << endl;
	/*
	* CAMERA CALIBRATION
	* saves intrinsic matrix + distortion parameters in camera.xml
	*//*
	string filename(file_location);
	filename.append("default.xml");
	cout << filename << endl;
	CameraCalibration( filename );*/

	/*
	* DETECT MARKER
	* find chessboard corners in each frame and draw them onto it if found
	*/
	
	vector<Point2f> corners;
	Size boardSize(9,6);

	bool found = false;

	//FIND OBJECT POINTS OF CHESSBOARD
	vector<Point2f> objectpoints_2f;
	Mat viewGray;
	cvtColor(image[CHESSBOARD_OBJECT], viewGray, CV_BGR2GRAY);
	imshow("cray", viewGray);
	found = findChessboardCorners(viewGray, boardSize, objectpoints_2f,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	//cout << found << endl;	
	if(found) {
		cornerSubPix( viewGray, objectpoints_2f, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		drawChessboardCorners( image[1], boardSize, Mat(objectpoints_2f), found );
		//imshow(to_string(k), frame);
	}
	imshow("found", image[CHESSBOARD_OBJECT]);
	vector<Point3f> objectpoints_3f;
	for( int i = 0; i< objectpoints_2f.size(); i++ )
     {
		// cout << endl << objectpoints_2f[i] << endl;
		 objectpoints_3f.push_back(Point3f(objectpoints_2f[i].x, objectpoints_2f[i].y, 0.0));
		// cout << objectpoints_3f[i] << endl;
     }


	//READ CAMERA FROM CAMERA.XML
	string filename = "camera_college.xml";
	FileStorage fs(filename, FileStorage::READ);
	Mat cameraMatrix;
	//cameraMatrix =(Mat) fs["Camera_Matrix"];
	fs["Camera_Matrix"] >> cameraMatrix;
	Mat distortionCoefficients ;
	fs["Distortion_Coefficients"] >> distortionCoefficients;


	fs.release();  //close file

	cout << "camera \n"<< cameraMatrix << endl;
	cout << "distortion\n"<< distortionCoefficients << endl;

	//FIND IMAGE POINTS

	Mat frame;
	int k =0;
	int num_frames = video[0].get(CV_CAP_PROP_FRAME_COUNT);
/*	while(k<num_frames){
		video[0].read(frame);
		Mat viewGray;
		cvtColor(frame, viewGray, CV_BGR2GRAY);
		found = findChessboardCorners(viewGray, boardSize, corners,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		
		if(found) {
			cornerSubPix( viewGray, corners, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			drawChessboardCorners( frame, boardSize, Mat(corners), found );
			//imshow(to_string(k), frame);
		}
		imshow("found", frame);
		corners.clear();
		cvWaitKey(1);
		cout << k << endl;
		k++;
	}
	*/

	//Mat viewGray;
	frame = image[SAMPLE_FRAME];
	cvtColor(frame, viewGray, CV_BGR2GRAY);
	found = findChessboardCorners(viewGray, boardSize, corners,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		
	if(found) {
		cornerSubPix( viewGray, corners, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		drawChessboardCorners( frame, boardSize, Mat(corners), found );
		//imshow(to_string(k), frame);
	}
	imshow("found", frame);
	
	
	//FIND EXTRINSIC - SOLVEPNP
	Mat rvec(3,1,cv::DataType<double>::type);
	Mat tvec(3,1,cv::DataType<double>::type);
	bool success = solvePnP(objectpoints_3f, corners, cameraMatrix, distortionCoefficients, rvec, tvec);
	if(success)
	cout <<"Hooray!\n\n";
	cout << "rvec\n" << rvec << endl;
	cout << "tvec\n" << tvec << endl;
	Mat rotationMatrix;
	Rodrigues(rvec, rotationMatrix); /**report! explain this!**/
	cout << "r mat\n" << rotationMatrix <<endl;

	//FORMAT EXTRNISIC AS MODELVIEW FOR OPENGL
	Mat extrinsicmatrix_4x4=Mat::zeros(4,4, cv::DataType<double>::type);
	extrinsicmatrix_4x4 = createExtrinsic(rotationMatrix, tvec);
	cout << "EXTRINSIC\n" << extrinsicmatrix_4x4 << endl;
	cout << "ROTATION\n" <<rotationMatrix << endl;
	cout << "TRANSLATION\n" << tvec << endl;	//extrinsicmatrix_4x4.col(3) = tvec
	Mat glModelView = cvToglModelView(extrinsicmatrix_4x4);
	cout << "model\view \n" << glModelView<<endl;





	int choice =0;
	do
	{
		imshow("Welcome", default_image);

		choice = cvWaitKey();
	
			
	} while ((choice != 'x') && (choice != 'X'));
}

Mat createExtrinsic(Mat rotationMatrix, Mat tvec){
	/**Maybe need to transpose rotation first or transpose result...**/
	Mat extrinsicmatrix_4x4=Mat::zeros(4,4, cv::DataType<double>::type);	
	//extrinsicmatrix_4x4.col(3) = tvec;
	//Mat(0,0,0,1).copyTo(extrinsicmatrix_4x4.row(4)) ;
	extrinsicmatrix_4x4.at<double>(0,0) = rotationMatrix.at<double>(0,0);
	extrinsicmatrix_4x4.at<double>(0,1) = rotationMatrix.at<double>(0,1);
	extrinsicmatrix_4x4.at<double>(0,2) = rotationMatrix.at<double>(0,2);
	extrinsicmatrix_4x4.at<double>(1,0) = rotationMatrix.at<double>(1,0);
	extrinsicmatrix_4x4.at<double>(1,1) = rotationMatrix.at<double>(1,1);
	extrinsicmatrix_4x4.at<double>(1,2) = rotationMatrix.at<double>(1,2);
	extrinsicmatrix_4x4.at<double>(2,0) = rotationMatrix.at<double>(2,0);
	extrinsicmatrix_4x4.at<double>(2,1) = rotationMatrix.at<double>(2,1);
	extrinsicmatrix_4x4.at<double>(2,2) = rotationMatrix.at<double>(2,2);
	extrinsicmatrix_4x4.at<double>(0,3) = tvec.at<double>(0);
	extrinsicmatrix_4x4.at<double>(1,3) = tvec.at<double>(1);
	extrinsicmatrix_4x4.at<double>(2,3) = tvec.at<double>(2);
	extrinsicmatrix_4x4.at<double>(3,3) = 1;
	return extrinsicmatrix_4x4;
}

Mat cvToglModelView(Mat cvMatrix){
	//invert axes because different coordinate systems
	Mat invertAxes = Mat::eye(4,4, cv::DataType<double>::type);
	invertAxes.at<double>(1,1) = -1;
	invertAxes.at<double>(2,2) = -1;
	return invertAxes*cvMatrix;
}
