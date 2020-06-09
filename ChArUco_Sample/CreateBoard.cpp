#include "Header.h"

// ChArUcoボードを作成し、画像としてファイルに書き出す関数
int CreateBoard() {
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.05f, 0.03f, dictionary);

	cv::Mat boardImage;							// ボードの画像

	board->draw(cv::Size(800, 800), boardImage, 30);
	cv::String out = "charucoBoard.png";		// ファイル名
	imshow("charucoBoard", boardImage);
	imwrite(out, boardImage);
	cv::waitKey(0);

	return 0;
}