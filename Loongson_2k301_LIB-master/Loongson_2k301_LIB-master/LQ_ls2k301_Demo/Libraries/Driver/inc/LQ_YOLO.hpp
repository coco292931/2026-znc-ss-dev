#ifndef LQ_YOLO_h
#define LQ_YOLO_h

#include <opencv2/core/mat.hpp>   // 或者 <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <chrono>
#include <cstdint>
#include <thread>
#include <signal.h>

using namespace cv;
using namespace std;
using namespace cv::dnn;
using namespace std::chrono;
using namespace std::this_thread;
using cv::Mat;
using std::cout;
using std::endl;
using std::string;
using std::vector;

void YOLO_Init();
void YOLO_Detecion(const Mat& image);

#endif