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
	cout << found << endl;	
	if(found) {
		cornerSubPix( viewGray, objectpoints_2f, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		drawChessboardCorners( image[1], boardSize, Mat(objectpoints_2f), found );
		//imshow(to_string(k), frame);
	}
	imshow("found", image[CHESSBOARD_OBJECT]);
	vector<Point3f> objectpoints_3f;
	/*for( int i = 0; i< objectpoints_2f.size(); i++ )
     {
		 objectpoints_3f.push_back(Point3f(objectpoints_2f.
     }*/




	//READ CAMERA FROM CAMERA.XML
	string filename = "camera_college.xml";
	FileStorage fs(filename, FileStorage::READ);
	Mat cameraMatrix;
	//cameraMatrix =(Mat) fs["Camera_Matrix"];
	fs["Camera_Matrix"] >> cameraMatrix;
	Mat distortionCoefficients ;
	fs["Distortion_Coefficients"] >> distortionCoefficients;


	fs.release();  //close file

	cout << cameraMatrix << endl;
	cout << distortionCoefficients << endl;

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
	//vector<Mat> rotationvector;
	//vector<Mat> translationvector;
	//bool success = solvePnP(objectpoints, corners, cameraMatrix, distortionCoefficients, rotationvector, translationvector);
	//if(success)
		cout <<"Hooray!\n";

	


	/*vector<Point2f> specialcorners;
	specialcorners.push_back(corners[0]);
	specialcorners.push_back(corners[8]);
	specialcorners.push_back(corners[53]);
	specialcorners.push_back(corners[45]);
	for( int i = 0; i< specialcorners.size(); i++ )
     {
       Scalar color = Scalar( 1,0,0 );
       drawContours( image[1], specialcorners, i, color );
     }
	imshow("contoured", image[1]); */

	/**
	* corners of bounding rect are 
	corners[0]
	corners[8]
	corners[53]
	corners[45]
	- clockwise from bottom left
	**/

	int choice =0;
	do
	{
		imshow("Welcome", default_image);

		choice = cvWaitKey();
	
			
	} while ((choice != 'x') && (choice != 'X'));
}