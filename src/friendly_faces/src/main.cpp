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
#include "net.h"

#include "visualization_msgs/Marker.h"

using namespace tf;
using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;

//loop stuff
FFFrameProcessor frame_proc;
FPSCounter fps;
Mat frame;
VideoCapture cap(0);
UDPBroadcast bcast;

class SegbotProcessor {
private:
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	ros::Subscriber vis_sub;

	bool processing = false;

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
		
		run(cv_ptr->image.clone());
	}
	
	void vis_callback(const visualization_msgs::Marker& msg) {
		printf("%s\n", msg.ns.c_str());
	}

public:
	void run(Mat frame) {
		Mat outputFrame = frame.clone();
		Point2i faces = frame_proc.process(frame);

		circle(outputFrame, faces, 5, Scalar(0, 255, 0));

		imshow("cam", outputFrame);

		bcast.send("3,2,1,6,5,4");
	}
	
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		vis_sub = nh.subscribe("visualization_marker", 1, &SegbotProcessor::vis_callback, this);
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
