#include <string>
#include <algorithm>
#include <stdio.h>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "image_proc.h"
#include "fps.h"

using namespace tf;
using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;

string greetings[] = {"hello", "hey there", "how are you", "nice to see you again"};
int greetings_sz = 4;

string goodbyes[] = {"good bye", "see you later", "bye", "I'll miss you"};
int goodbyes_sz = 4;

string doors[] = {"d3_414b1","d3_414b2", "d3_414a1", "d3_414a2"};
int doors_sz = 4;

//init

//loop stuff
bool idle = true;
FFFrameProcessor frame_proc;
FPSCounter fps;
Mat frame;
VideoCapture cap(0);

class SegbotProcessor {
private:
	bool processing = true;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	

	bool first = true;
	bool killed = false;
	double lastIdle = -1;

	void callback(const sensor_msgs::ImageConstPtr& msg) {
		if (!processing) {
			return;
		}

		CvImagePtr cv_ptr;
		try {
			cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		//run(cv_ptr->image.clone());
	}

public:
	void run(Mat frame) {
		Mat outputFrame = frame.clone();
		Point2i faces = frame_proc.process(frame);
		
		//for (auto elem : faces) {
		//rectangle(outputFrame, elem.first, Scalar(0, 255, 0));
		circle(outputFrame, faces, 5, Scalar(0, 255, 0));
		//}
		
		imshow("cam", outputFrame);


		if (waitKey(10) >= 0) {
			processing = false;
		}
	}
	
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		//image_sub = it.subscribe("/nav_kinect/rgb/image_raw", 1, &SegbotProcessor::callback, this);
		//image_sub = it.subscribe("/gscam/image_raw", 1, &SegbotProcessor::callback, this);

	}
};

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	NodeHandle nh;

	SegbotProcessor sp(nh);
	bool running = true;
	while (running) {
		Mat frame;
		cap >> frame;
		sp.run(frame);
		
		if (cvWaitKey(10) >= 0) {
			cvDestroyWindow("cam");
			running = false;
		}
	}

	return 0;
}
