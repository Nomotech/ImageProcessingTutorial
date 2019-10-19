#include <iostream>
#include <opencv2/opencv.hpp>

#include "../utility/fileSystem.hpp"

#define PAT_ROW		7			/* パターンの行数 */
#define PAT_COL		10			/* パターンの列数 */
#define CHESS_SIZE	50.0		/* パターン1マスの1辺サイズ[mm] */


int calibration(std::vector<cv::Mat> images, int patRow, int patCol, float chessSize) {
	cv::Size patternSize = cv::Size2i(patCol, patRow);
	std::vector<cv::Point2f> corners;
	std::vector<std::vector<cv::Point2f>> imgPoints;

	// setting 3d points
	std::vector<cv::Point3f> points;
	for (int j = 0; j < patRow; j++) {
		for (int k = 0; k < patCol; k++) {
			cv::Point3f p(j * chessSize, k * chessSize, 0.0);
			points.push_back(p);
		}
	}

	std::vector<std::vector<cv::Point3f>> objPoints;
	for (int i = 0; i < images.size(); i++) {
		objPoints.push_back(points);
	}

	// findChessboardCorners
	int foundNum = 0;
	cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);
	for (int i = 0; i < images.size(); i++) {
		bool found = cv::findChessboardCorners(images[i], patternSize, corners);
		if (found) foundNum++;

		// drawChessboardCorners
		cv::Mat src_gray = cv::Mat(images[i].size(), CV_8UC1);
		cv::cvtColor(images[i], src_gray, cv::COLOR_BGR2GRAY);
		cv::find4QuadCornerSubpix(src_gray, corners, cv::Size(3, 3));
		cv::drawChessboardCorners(images[i], patternSize, corners, found);
		imgPoints.push_back(corners);

		cv::imshow("Calibration", images[i]);
		cv::waitKey(1);
	}
	cv::destroyWindow("Calibration");

	if (foundNum != images.size()) {
		std::cerr << "Calibration Images are insufficient." << std::endl;
		return -1;
	}

	// -------------------------------< calibration >-------------------------------
	cv::Mat camMat; // カメラ内部パラメータ行列
	cv::Mat distCoefs; // 歪み係数
	std::vector<cv::Mat> rvecs, tvecs; // 各ビューの回転ベクトルと並進ベクトル
	cv::calibrateCamera(
		objPoints,
		imgPoints,
		images[0].size(),
		camMat,
		distCoefs,
		rvecs,
		tvecs
	);

	// export
	cv::FileStorage fs("./calibrationData.xml", cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		std::cerr << "File can not be opened." << std::endl;
		return -1;
	}

	fs << "camMat" << camMat;
	fs << "distCoefs" << distCoefs;
	fs << "rvecs" << rvecs;
	fs << "tvecs" << tvecs;
	fs.release();

	return 0;
}

int reprojection(std::vector<cv::Mat> images, int patRow, int patCol, float chessSize) {
	cv::Size patternSize = cv::Size2i(patCol, patRow);
	std::vector<cv::Point2f> corners;
	std::vector<std::vector<cv::Point2f>> imgPoints;

	int foundNum = 0;
	for (int i = 0; i < images.size(); i++) {
		bool found = cv::findChessboardCorners(images[i], patternSize, corners);
		if (found) foundNum++;

		// drawChessboardCorners
		cv::Mat src_gray = cv::Mat(images[i].size(), CV_8UC1);
		cv::cvtColor(images[i], src_gray, cv::COLOR_BGR2GRAY);
		cv::find4QuadCornerSubpix(src_gray, corners, cv::Size(3, 3));
		cv::drawChessboardCorners(images[i], patternSize, corners, found);
		imgPoints.push_back(corners);

		cv::imshow("Calibration", images[i]);
		cv::waitKey(1);
	}

	// import
	cv::FileStorage fs("./calibrationData.xml", cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cout << "File can not be opened." << std::endl;
		return -1;
	}
	cv::Mat camMat, distCoefs;
	std::vector<cv::Mat> rvecs, tvecs;
	fs["camMat"] >> camMat;
	fs["distCoefs"] >> distCoefs;
	fs["rvecs"] >> rvecs;
	fs["tvecs"] >> tvecs;



	// -------------------------------------< reprojection >-------------------------------------
	// create result image
	int w = 640;
	int h = 640;
	cv::Mat result(cv::Size(w, h), CV_8UC3, cv::Scalar::all(255));
	cv::Point2f center = cv::Point2f(w / 2, h / 2);
	for (int i = 0; i < w / 2; i += 50) {
		cv::Scalar color = (i % 250 == 0) ? cv::Scalar(20, 20, 20) : cv::Scalar(150, 150, 150);
		cv::line(result, cv::Point(w / 2 + i, 0), cv::Point(w / 2 + i, h), color, 1, 4);
		cv::line(result, cv::Point(w / 2 - i, 0), cv::Point(w / 2 - i, h), color, 1, 4);
	}
	for (int i = 0; i < h / 2; i += 50) {
		cv::Scalar color = (i % 250 == 0) ? cv::Scalar(20, 20, 20) : cv::Scalar(150, 150, 150);
		cv::line(result, cv::Point(0, h / 2 + i), cv::Point(w, h / 2 + i), color, 1, 4);
		cv::line(result, cv::Point(0, h / 2 - i), cv::Point(w, h / 2 - i), color, 1, 4);
	}
	cv::putText(result, "scale : 0.2 pixcel", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 50, 200), 2);


	// reprojection
	for (int i = 0; i < images.size(); i++) {
		double u, v;
		double reprojectionError = 0.0;
		cv::Mat R, Rt;
		cv::Rodrigues(rvecs[i], R);
		cv::hconcat(R, tvecs[i], Rt);

		for (int j = 0; j < patRow; j++) {
			for (int k = 0; k < patCol; k++) {
				cv::Mat objectPoint = (cv::Mat_<double>(4, 1) << j * chessSize, k * chessSize, 0.0, 1.0);
				cv::Mat p = camMat * Rt * objectPoint;
				u = p.at<double>(0) / p.at<double>(2);
				v = p.at<double>(1) / p.at<double>(2);

				cv::Point2f crossPoint = imgPoints[i][j * patCol + k];
				cv::Point2f reprojectionPoint = cv::Point2f(u, v);
				cv::circle(images[i], crossPoint, 1, cv::Scalar(0, 200, 0), 3, 4);	// 交点
				cv::circle(images[i], reprojectionPoint, 1, cv::Scalar(0, 0, 200), 3, 4);	// 再投影点
				double norm = cv::norm(reprojectionPoint - crossPoint);
				reprojectionError += norm; // 再投影誤差


				cv::Mat c(1, 1, CV_8UC3, cv::Scalar(norm * 50, 255, 255));
				cv::cvtColor(c, c, cv::COLOR_HSV2RGB);
				cv::Scalar color = cv::Scalar(c.data[0], c.data[1], c.data[2]);
				cv::circle(result, center + (reprojectionPoint - crossPoint) * 100, 1, color, 3, 4);	// 再投影点
			}
		}
		reprojectionError /= patRow * patCol;

		std::ostringstream str;
		str << "image" << i << " : reprojection error [pixcel/point] = " << reprojectionError;
		cv::putText(images[i], str.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 200), 1.5);
		cv::putText(images[i], "Press any key", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 200), 2);
		cv::imshow("reprojection", images[i]);
		cv::waitKey(0);
	}

	// show result
	cv::imshow("reprojection erros", result);
	cv::waitKey(0);
	return 0;
}

int main() {
	std::string dirName;
	std::cout << "input image dirctory path : ";
	std::getline(std::cin, dirName);
	
	std::vector<std::string> fileList = getImageList(dirName);
	std::vector<cv::Mat> inputData;
	for (auto f : fileList) {
		std::stringstream ss;
		ss << dirName << f;
		std::cout << ss.str() << std::endl;
		inputData.push_back(cv::imread(ss.str()));
	}

	if (inputData.size() == 0) {
		return -1;
	}

	// calibration(inputData, PAT_ROW, PAT_COL, CHESS_SIZE);
	reprojection(inputData, PAT_ROW, PAT_COL, CHESS_SIZE);
	
	return 0;
}