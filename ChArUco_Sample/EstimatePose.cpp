#include "Header.h"

#include <vector>

// カメラパラメータを読み込む関数
bool readCameraParameters(const cv::String& filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

// カメラの姿勢推定を行う関数 (問1-2)
int EstimatePose() {
	cv::Mat cameraMatrix, distCoeffs;
	if (!readCameraParameters("camera.xml", cameraMatrix, distCoeffs)) {
		std::cerr << "カメラパラメータファイルを開けません" << std::endl;
		return -1;
	}

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.05f, 0.03f, dictionary);

	// 動画ファイルの読み込み
	cv::VideoCapture cap;
	cap.open("video.mov");
	if (!cap.isOpened()) {
		std::cerr << "ファイルが開けません" << std::endl;
		return -1;
	}

	// 動画サイズの取得のため1フレームだけ読み取る
	cv::Mat inputImage;
	cap >> inputImage;
	cv::Size imgSize = inputImage.size();

	char key = 0;
	while (key != 'q') {
		std::vector<int> markerIds, charucoIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		std::vector<cv::Point2f> charucoCorners;

		cap >> inputImage;
		if (inputImage.empty()) {
			break;
		}

		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);
		if (markerIds.size() > 0) {
			cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
		}

		cv::Vec3d rvecs, tvecs;			// カメラ座標系におけるマーカーの位置
		bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvecs, tvecs);

		cv::Mat imageCopy;
		inputImage.copyTo(imageCopy);

		if (valid) {
			// 姿勢推定ができていればマーカー座標系の軸を描画
			cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);

			// マーカー座標系におけるカメラ位置の計算
			cv::Mat rmtx;
			cv::Rodrigues(rvecs, rmtx);
			cv::Mat cameraPos = -rmtx.inv() * tvecs;
			std::cout << cameraPos << std::endl;
		}

		cv::imshow("video", imageCopy);
		key = (char)cv::waitKey(1);
	}

	return 0;
}