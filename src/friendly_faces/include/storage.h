#pragma once

#include <string>
#include <unordered_map>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void learn(string name, Mat face);

unordered_map<string, Mat> getDB();

string findFriend(Mat frame);

void reset();
