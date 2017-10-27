#pragma once

#include <vector>
#include <unordered_map>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class FFFrameProcessor {
private:
	CascadeClassifier face_cascade;
public:
	FFFrameProcessor();

	Point2i process(Mat frame);
};

double frameCompare(Mat frame1, Mat frame2);

bool vector_contains(vector<pair<Rect, string>> vec, pair<Rect, string> elem);
