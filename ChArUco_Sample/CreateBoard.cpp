#include "Header.h"

// ChArUco�{�[�h���쐬���A�摜�Ƃ��ăt�@�C���ɏ����o���֐�
int CreateBoard() {
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.05f, 0.03f, dictionary);

	cv::Mat boardImage;							// �{�[�h�̉摜

	board->draw(cv::Size(800, 800), boardImage, 30);
	cv::String out = "charucoBoard.png";		// �t�@�C����
	imshow("charucoBoard", boardImage);
	imwrite(out, boardImage);
	cv::waitKey(0);

	return 0;
}