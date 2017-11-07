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
UDPBroadcast bcast;

//3 rigid transformations <(eye1_pos, eye2_pos, head_pos), (eye1_rot, eye2_rot, head_rot)>
typedef pair<tuple<Vector3, Vector3, Vector3>, tuple<Vector3, Vector3, Vector3>> tri_rigid;

class SegbotProcessor {
private:
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	ros::Subscriber vis_sub;
	bool processing;

	void callback(const sensor_msgs::ImageConstPtr& msg) {
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
	
	void img_callback(const visualization_msgs::Marker& msg) {
		printf("%s\n", msg.ns.c_str());
	}
	
	//justin's math to convert virtual to physical coordinates
	Vector3 v2p(Vector3& v) {
		return v;
	}
	
	//math that converts physical coordinates to 3 rigid transformations (position & rotation)
	tri_rigid math(Vector3& p) {
		tri_rigid vv;
		return vv;
	}

public:
	void run(Mat frame) {
		Mat outputFrame = frame.clone();
		/*Point2i faces = frame_proc.process(frame);
		
		Vector3 v(0, 0, 0);
		
		Vector3 vCoords = v2p(v);
		
		tri_rigid set = math(vCoords);
		
		//"3,2,1,3,2,1,3,2,1,3,2,1"
		char buffer [100];
		sprintf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", std::get<0>(set.first).m_floats[0],std::get<0>(set.first).m_floats[1],std::get<0>(set.first).m_floats[2],
		                                                                     std::get<1>(set.first).m_floats[0],std::get<1>(set.first).m_floats[1],std::get<1>(set.first).m_floats[2],
		                                                                     std::get<2>(set.first).m_floats[0],std::get<2>(set.first).m_floats[1],std::get<2>(set.first).m_floats[2],
		                                                                     std::get<0>(set.second).m_floats[0],std::get<0>(set.second).m_floats[1],std::get<0>(set.second).m_floats[2],
		                                                                     std::get<1>(set.second).m_floats[0],std::get<1>(set.second).m_floats[1],std::get<1>(set.second).m_floats[2],
		                                                                     std::get<2>(set.second).m_floats[0],std::get<2>(set.second).m_floats[1],std::get<2>(set.second).m_floats[2]);
		//bcast.send(buffer);
		printf(buffer);
		
		//display
		circle(outputFrame, faces, 5, Scalar(0, 255, 0));*/

		imshow("cam", outputFrame);
		
		if (waitKey(50) > 0) {
			processing = false;
			destroyAllWindows();
			shutdown();
		}
	}
	
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		//vis_sub = nh.subscribe("/image_raw", 1, &SegbotProcessor::vis_callback, this);
		image_sub = it.subscribe("/image_raw", 1, &SegbotProcessor::callback, this);
	}
};

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	namedWindow("cam");
	
	startWindowThread();

	NodeHandle nh;

	SegbotProcessor sp(nh);
	
	spin();

	return 0;
}
