/*
 * Copyright (c) 2009, Morgan Quigley, Clemens Eppner, Tully Foote
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>

sensor_msgs::CameraInfo cam_info;
ros::ServiceServer set_camera_info_service;
using namespace std;

template<class T>
string arrayToString(T v) {
	stringstream ss;
	for (size_t i = 0; i < v.size(); i++) {
		ss << v[i] << " ";
	}
	return (ss.str());
}

template<class T>
void restoreVec(string s, T &vec) {
	stringstream ss(s);
	double a = 0;
	int i = 0;
	while (ss >> a && vec.size()>i) {
		vec[i++] = a;
	}
}

void load_params(ros::NodeHandle &nh) {
	string str_calib;
	stringstream ss_calib;

	nh.param("D", str_calib, string(""));
	restoreVec(str_calib, cam_info.D);

	nh.param("K", str_calib, string(""));
	 (str_calib, cam_info.K);

	nh.param("R", str_calib, string(""));
	restoreVec(str_calib, cam_info.R);

	nh.param("P", str_calib, string(""));
	restoreVec(str_calib, cam_info.P);
	
	std::string frame_id;
	nh.param<std::string>("frame_id",frame_id,"/camera");// /viz/uav_cam_front/camera
	cam_info.header.frame_id = frame_id;

	cout
			<< "Current cam_info:"
			<< endl;
	cout << "    <param name=\"D\" type=\"string\" value=\"" << arrayToString(
			cam_info.D) << "\"/>" << endl;
	cout << "    <param name=\"K\" type=\"string\" value=\"" << arrayToString(
			cam_info.K) << "\"/>" << endl;
	cout << "    <param name=\"R\" type=\"string\" value=\"" << arrayToString(
			cam_info.R) << "\"/>" << endl;
	cout << "    <param name=\"P\" type=\"string\" value=\"" << arrayToString(
			cam_info.P) << "\"/>" << endl;
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &rsp) {
	ROS_INFO("setCameraInfo: received new camera parameters");

	cam_info = req.camera_info;

	cout
			<< "Add these lines to your camera.launch file to make the changes permanent:"
			<< endl;
	cout << "    <param name=\"D\" type=\"string\" value=\"" << arrayToString(
			cam_info.D) << "\"/>" << endl;
	cout << "    <param name=\"K\" type=\"string\" value=\"" << arrayToString(
			cam_info.K) << "\"/>" << endl;
	cout << "    <param name=\"R\" type=\"string\" value=\"" << arrayToString(
			cam_info.R) << "\"/>" << endl;
	cout << "    <param name=\"P\" type=\"string\" value=\"" << arrayToString(
			cam_info.P) << "\"/>" << endl;

	return 1;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "uvc_cam");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	std::string device;
	std::string camera_ns;
	std::string topic_name;
	std::string camera_info_name;
	n_private.param<std::string> ("device", device, "/dev/video0");
	n_private.param<std::string> ("camera", camera_ns, "/uav_cam_front");//link
	n_private.param<std::string> ("topic_name", topic_name , "/image_raw");
	n_private.param<std::string> ("camera_info_name",camera_info_name, "/set_camera_info");

	int width, height, fps, modetect_lum, modetect_count, camera_mode;
	n_private.param("width", width, 320);
	n_private.param("height", height, 240);
	n_private.param("fps", fps, 15);
	n_private.param("motion_threshold_luminance", modetect_lum, 100);
	n_private.param("motion_threshold_count", modetect_count, -1);
	n_private.param("camera_mode", camera_mode, 1);// 0: off, 1: Manual Mode, 3: Aperture Priority Mode


	load_params(n_private);
	ros::NodeHandle ns_camera(camera_ns);
	image_transport::ImageTransport it(ns_camera);
	image_transport::CameraPublisher camera_publisher = it.advertiseCamera(topic_name, 1);//"/viz/uav_cam_front/image_raw"//("image_raw", 1)

	set_camera_info_service = ns_camera.advertiseService(camera_info_name, &setCameraInfo);//"/viz/uav_cam_front/set_camera_info" //set_camera_info

	ROS_INFO("opening uvc_cam at %dx%d, %d fps", width, height, fps);
	uvc_cam::Cam
			cam(device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
	cam.set_motion_thresholds(modetect_lum, modetect_count);
	cam.set_control(0x9a0901, camera_mode);

	ros::Time t_prev(ros::Time::now());
	int count = 0, skip_count = 0;

	ros::Rate loop_rate(fps);

	while (n.ok()) {
		unsigned char *frame = NULL;
		uint32_t bytes_used;
		int buf_idx = cam.grab(&frame, bytes_used);
		if (count++ % fps == 0) {
			ros::Time t(ros::Time::now());
			ros::Duration d(t - t_prev);
			ROS_INFO("%.1f fps skip %d", (double)fps / d.toSec(), skip_count);
			t_prev = t;
		}
		if (frame) {
			sensor_msgs::Image image;
			image.header.stamp = ros::Time::now();
			image.encoding = sensor_msgs::image_encodings::RGB8;
			image.height = height;
			image.width = width;
			image.step = 3 * width;

			image.data.resize(image.step * image.height);
			/*
			 uint8_t* bgr = &(image.data[0]);
			 for (uint32_t y = 0; y < height; y++)
			 for (uint32_t x = 0; x < width; x++)
			 {
			 // hack... flip bgr to rgb
			 uint8_t *p = frame + y * width * 3 + x * 3;
			 uint8_t *q = bgr   + y * width * 3 + x * 3;
			 q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
			 }
			 */
			memcpy(&image.data[0], frame, width * height * 3);
			image.header.frame_id=cam_info.header.frame_id;
			cam_info.width = width;
			cam_info.height = height;
			cam_info.roi.width = width;
			cam_info.roi.height = height;
			cam_info.header = image.header;

			camera_publisher.publish(image, cam_info);
			cam.release(buf_idx);
		} else
			skip_count++;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

