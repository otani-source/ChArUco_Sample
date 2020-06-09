#include "Header.h"

#include <vector>

// ChArUco�{�[�h���g���ăL�����u���[�V�������s���֐�(��1-1)
int Calibration() {
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.05f, 0.03f, dictionary);

	std::vector<std::vector<cv::Point2f>> allCharucoCorners;
	std::vector<std::vector<int>> allCharucoIds;

	// ����t�@�C���̓ǂݍ���
	cv::VideoCapture cap;
	cap.open("video.mov");
	if (!cap.isOpened()) {
		std::cerr << "�t�@�C�����J���܂���" << std::endl;
		return -1;
	}

	// ����T�C�Y�̎擾�̂���1�t���[�������ǂݎ��
	cv::Mat inputImage;
	cap >> inputImage;
	cv::Size imgSize = inputImage.size();


	// �e�t���[����ChArUco�{�[�h�����o���āAallCharucoCorners��allCharucoIds�Ƀf�[�^���i�[����
	char key = 0;
	while (key != 'q') {
		std::vector<std::vector<cv::Point2f>> markerCorners;
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> markerIds, charucoIds;

		cap >> inputImage;
		
		if (inputImage.empty()) {
			break;
		}

		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

		cv::Mat imageCopy;
		inputImage.copyTo(imageCopy);
		cv::putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
			cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

		if (markerIds.size() > 0) {
			cv::aruco::drawDetectedMarkers(imageCopy, markerCorners);
			cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, charucoCorners, charucoIds);

			if (charucoCorners.size() > 0) {
				cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds);
			}

			if (key == 'c') {
				allCharucoCorners.push_back(charucoCorners);
				allCharucoIds.push_back(charucoIds);
			}
		}

		cv::imshow("video", imageCopy);
		key = (char)cv::waitKey(1);
	}

	if (allCharucoIds.size() < 1) {
		std::cerr << "Not enough captures for calibration" << std::endl;
		return 0;
	}

	if (allCharucoCorners.size() < 4) {
		std::cerr << "Not enough corners for calibration" << std::endl;
		return 0;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;

	double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

	std::cout << "re-projection error: " << repError << std::endl;

	cv::FileStorage fs("camera.xml", cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		std::cerr << "�J�����p�����[�^���������߂܂���" << std::endl;
		return -1;
	}
	fs << "camera_matrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;

	return 0;
}