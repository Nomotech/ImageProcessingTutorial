#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>


#include "../utility/baslerCam.hpp"
#include "../utility/shader.hpp"

#define USE_BASLER 0

int main() {
	int Width = 640;
	int Height = 480;
	int FPS = 120;

	int procCount = 0;
	int capCount = 0;
	bool loopFlag = true;
	bool binFlag = false;
	bool edgeFlag = false;
	bool polyFlag = false;
	bool trampFlag = false;
	bool labelFlag = false;
	int xorNum = 0;
	bool cubeFlag = false;

	const int cols = 32;
	const int rows = (int)(cols * 89.0 / 58.0);

	// tramp data set
	std::vector<cv::Mat> tramp;
	for (int i = 0; i < 13; i++) {
		std::stringstream sst;
		sst << "./img/s" << i + 1 << ".jpg";
		cv::Mat img = cv::imread(sst.str(), 0);
		cv::resize(img, img, cv::Size(), (double)cols / img.cols, (double)rows / img.rows);
		tramp.push_back(img);
	}

	//cube
	float w = (float)cols;
	float l = (float)rows;
	float h = l / 2.0;
	std::vector<cv::Point3f> cube;
	cube.push_back(cv::Point3f(0, 0, 0));
	cube.push_back(cv::Point3f(w, 0, 0));
	cube.push_back(cv::Point3f(w, l, 0));
	cube.push_back(cv::Point3f(0, l, 0));
	cube.push_back(cv::Point3f(0, 0, -h));
	cube.push_back(cv::Point3f(w, 0, -h));
	cube.push_back(cv::Point3f(w, l, -h));
	cube.push_back(cv::Point3f(0, l, -h));

	cv::Point2f dstPoints[4];
	dstPoints[0] = { 0,				0 };
	dstPoints[1] = { 0,				(float)rows };
	dstPoints[2] = { (float)cols,	(float)rows };
	dstPoints[3] = { (float)cols,	0 };

	//camera parameters
	double fx = 525.0; //focal length
	double fy = 525.0; //focal length
	double cx = 319.5; //optical centre x
	double cy = 239.5; //optical centre y

	// import
	cv::FileStorage fs("./calibrationData.xml", cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cout << "File can not be opened." << std::endl;
		return -1;
	}
	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	fs["camMat"] >> cameraMatrix;
	fs["distCoefs"] >> distCoeffs;
	fs["rvecs"] >> rvecs;
	fs["tvecs"] >> tvecs;
	std::cout << cameraMatrix << std::endl;

#if USE_BASLER
	cv::Mat frame(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat output(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat grayImage(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat binImage(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat dst(rows, cols, CV_64FC4);
	cv::Mat img_xor(rows, cols, CV_64FC4);

#else
	cv::VideoCapture cap(1);
	Width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	Height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	cv::Mat frame(cv::Size(Width, Height), CV_8UC3, cv::Scalar::all(255));
	cv::Mat output(cv::Size(Width, Height), CV_8UC3, cv::Scalar::all(255));
	cv::Mat grayImage(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat binImage(cv::Size(Width, Height), CV_8UC1, cv::Scalar::all(255));
	cv::Mat dst(rows, cols, CV_64FC4);
	cv::Mat img_xor(rows, cols, CV_64FC4);
#endif

	std::thread thrCap([&] {
#if USE_BASLER
		BASLER cam;
		cam.init(0);
		cam.setParam(1000000.0 / FPS - 50.0, Width, Height, false);
		cam.start();
		while (loopFlag) {
			if (cam.getData(frame.data) > 0) {
				memcpy(grayImage.data, frame.data, Width * Height * 1);
				cv::threshold(grayImage, binImage, 128.0, 255.0, cv::THRESH_OTSU);
				capCount++;
			}
		}
		cam.stop();
#else
		while (loopFlag) {
			capCount++;
			cv::Mat tmp;
			cap >> tmp;
			// ÉLÉÉÉvÉ`ÉÉÇ≈Ç´ÇƒÇ¢Ç»ÇØÇÍÇŒèàóùÇîÚÇŒÇ∑
			if (tmp.data) {
				std::memcpy(frame.data, tmp.data, sizeof(unsigned char) * Width * Height * 3);
				cv::cvtColor(tmp, grayImage, cv::COLOR_BGR2GRAY);
				cv::threshold(grayImage, binImage, 128.0, 255.0, cv::THRESH_OTSU);

			} else {
				continue;
			}
		}
#endif
	});

	std::thread thrProc([&] {
		// esc ÇâüÇ∑Ç‹Ç≈
		while (loopFlag) {
			procCount++;
			//std::memcpy(output.data, frame.data, sizeof(unsigned char) * Width * Height * 3);

			// ó÷äsíäèo
			std::vector<std::vector< cv::Point >> contours;
			cv::findContours(binImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
			// åüèoÇ≥ÇÍÇΩó÷äsê¸ÇÃï`âÊ
			if (edgeFlag) {
				for (auto contour = contours.begin(); contour != contours.end(); contour++) {
					cv::polylines(frame, *contour, true, cv::Scalar(0, 255, 0), 2);
				}
			}

			// ó÷äsÇ™éläpå`Ç©ÇÃîªíË
			for (auto contour = contours.begin(); contour != contours.end(); contour++) {
				// ó÷äsÇíºê¸ãﬂéó
				std::vector< cv::Point > approx;
				cv::approxPolyDP(cv::Mat(*contour), approx, 50.0, true);
				// ãﬂéóÇ™4ê¸Ç©Ç¬ñ êœÇ™àÍíËà»è„Ç»ÇÁéläpå`
				double area = cv::contourArea(approx);
				if (approx.size() == 4 && area > 100.0) {
					

					float da = min(cv::norm(approx[0] - approx[1]), cv::norm(approx[2] - approx[3]));
					float db = min(cv::norm(approx[1] - approx[2]), cv::norm(approx[3] - approx[0]));


					if (da < db) {
						approx.push_back(approx[0]);
						approx.erase(approx.begin());
					}
					

					cv::Point2f aprPoints[4]{ approx[0], approx[1], approx[2], approx[3] };
					if (polyFlag) cv::polylines(frame, approx, true, cv::Scalar(255, 0, 0), 2);

					cv::Mat H = getPerspectiveTransform(aprPoints, dstPoints);
					warpPerspective(binImage, dst, H, dst.size(), cv::INTER_NEAREST);


					float score = 255;
					int num = 0;

					for (int i = 0; i < 13; i++) {
						bitwise_xor(dst, tramp[i], img_xor);
						float mean = cv::mean(img_xor)[0];
						if (mean < score) {
							score = mean;
							num = i + 1;
						}
					}


					if (score < 20) {
						if (labelFlag) cv::putText(frame, std::to_string(num), (approx[0] + approx[2]) / 2 + cv::Point{ -5, 5 }, cv::FONT_HERSHEY_PLAIN, 6.0, cv::Scalar(0, 0, 255), 3);

						// solve PnP
						cv::Mat rvec(3, 1, cv::DataType<double>::type);
						cv::Mat tvec(3, 1, cv::DataType<double>::type);

						std::vector<cv::Point3f > objectPoints;
						objectPoints.push_back(cv::Point3f(0, 0, 10));
						objectPoints.push_back(cv::Point3f(0, (float)rows, 10));
						objectPoints.push_back(cv::Point3f((float)cols, (float)rows, 10));
						objectPoints.push_back(cv::Point3f((float)cols, 0, 10));

						std::vector<cv::Point2f> imagePoints;
						imagePoints.push_back((cv::Point2f)approx[0]);
						imagePoints.push_back((cv::Point2f)approx[1]);
						imagePoints.push_back((cv::Point2f)approx[2]);
						imagePoints.push_back((cv::Point2f)approx[3]);

						cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

						


						if (cubeFlag) {
							std::vector<cv::Point2f> cube2D;
							cv::projectPoints(cube, rvec, tvec, cameraMatrix, distCoeffs, cube2D);
							cv::line(frame, cube2D[0], cube2D[1], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[1], cube2D[2], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[2], cube2D[3], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[3], cube2D[0], cv::Scalar(255, 255, 255), 1, 4);

							cv::line(frame, cube2D[4], cube2D[5], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[5], cube2D[6], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[6], cube2D[7], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[7], cube2D[4], cv::Scalar(255, 255, 255), 1, 4);

							cv::line(frame, cube2D[0], cube2D[4], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[1], cube2D[5], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[2], cube2D[6], cv::Scalar(255, 255, 255), 1, 4);
							cv::line(frame, cube2D[3], cube2D[7], cv::Scalar(255, 255, 255), 1, 4);
						}
					}
					
				}
			}
		}
	});
	

	clock_t start = clock();
	while (loopFlag) {
		clock_t end = clock();
		if (binFlag) cv::imshow("bin", binImage);
		else cv::destroyWindow("bin");

		if (xorNum > 0) {
			bitwise_xor(dst, tramp[xorNum - 1], img_xor);
			cv::imshow("xor", img_xor);
		}
		else {
			cv::destroyWindow("xor");
		}


		if (trampFlag) cv::imshow("dst", dst);
		else cv::destroyWindow("dst");


		cv::imshow("output", frame);



		switch (cv::waitKey(1)) {
		case '1': binFlag = !binFlag; break;
		case '2': edgeFlag = !edgeFlag; break;
		case '3': polyFlag = !polyFlag; break;
		case '4': trampFlag = !trampFlag; break;
		case '5': xorNum = (xorNum + 1) % 14; break;
		case '6': labelFlag = !labelFlag; break;
		case '7': cubeFlag = !cubeFlag; break;
		case 's': cv::imwrite("./img/test.jpg", frame); break;
		case 0x1b:
			loopFlag = false;
			break;
		}

		const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC;
		if (time > 1.0) {
			std::cout << "cap : " << capCount / time << " fps proc : " << procCount / time << std::endl;
			capCount = 0;
			procCount = 0;
			start = clock();
		}
	}
	
	thrProc.join();
	thrCap.join();
}

