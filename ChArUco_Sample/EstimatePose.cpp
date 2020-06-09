#include "Header.h"

#include <vector>

// �J�����p�����[�^��ǂݍ��ފ֐�
bool readCameraParameters(const cv::String& filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

// �J�����̎p��������s���֐� (��1-2)
int EstimatePose() {
	cv::Mat cameraMatrix, distCoeffs;
	if (!readCameraParameters("camera.xml", cameraMatrix, distCoeffs)) {
		std::cerr << "�J�����p�����[�^�t�@�C�����J���܂���" << std::endl;
		return -1;
	}

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.05f, 0.03f, dictionary);

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

		cv::Vec3d rvecs, tvecs;			// �J�������W�n�ɂ�����}�[�J�[�̈ʒu
		bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvecs, tvecs);

		cv::Mat imageCopy;
		inputImage.copyTo(imageCopy);

		if (valid) {
			// �p�����肪�ł��Ă���΃}�[�J�[���W�n�̎���`��
			cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);

			// �}�[�J�[���W�n�ɂ�����J�����ʒu�̌v�Z
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