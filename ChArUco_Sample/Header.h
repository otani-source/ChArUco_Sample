#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

int CreateBoard();
int Calibration();
int EstimatePose();