// STD
#include <cstdio>
#include <iostream>
#include <math.h>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// msg
#include <lenti_mark/markers.h>
#include <lenti_mark/marker.h>

// lenti_mark
#include "LentiMarkTracker.h"
#include "lenti_mark.h"

using namespace std;
using namespace lenti_mark;


// constructor
LentiMarkNode::LentiMarkNode()
{
  // marker params file
  nh.getParam("/lenti_mark/mk_param_file", mk_file);

  // type 0:file, 1:topic (CameraInfo)
  nh.getParam("/lenti_mark/campara_type", camparam_type);

  // camera params file
  if (camparam_type == 0) {
    nh.getParam("/lenti_mark/cam_param_file", cam_file);
  }

  // debug ----------------
  // frame_id world
  nh.getParam("/lenti_mark/world_flg", tf_world_flg);
  // camera position
  nh.getParam("/lenti_mark/cam_pos_x", cam_pos_x);
  nh.getParam("/lenti_mark/cam_pos_y", cam_pos_y);
  nh.getParam("/lenti_mark/cam_pos_z", cam_pos_z);
  // debug ----------------
}

// setParams
int LentiMarkNode::setParams(void)
{
  int ret, ret_cam, ret_mk;

  // camera params
  // file
  if (camparam_type == 0){
    //ROS_INFO("CameraParamFile.");
    ret_cam = LMT.setCamParams(cam_file);
  } 
  // topic (CameraInfo)
  else if(camparam_type == 1) {
    ret_cam = LMT.setCamParams(img_size, cam_matrix, dist_coeffs);
    ret_cam = -1;
  } 
  else {
    ret_cam = -1;
  }

  // marker params
  ret_mk = LMT.setMarkerParams(mk_file);

  if (!ret_cam && !ret_mk){
    ret = 0;	 
  } else {
    ret = -1;
    if (ret_cam) cout << "[ERROR] Camera Params." << endl;
    if (ret_mk)  cout << "[ERROR] Marker Params." << endl;
  }

  return ret;
}


// detect
int LentiMarkNode::detect(void)
{
  int ret = -1;
  int conf_id = -1;
  try
  {
    // params
    if(setParams() != 0) return -10;

    // image
    if (image.rows < 1 || image.cols < 1) {
      cout << "[ERROR] image read." << endl;
      return -11;
    }
    
    // detect
    if (LMT.detect(image) < 0) {
      return -12;
    }


    // result
    int candidateNum = LMT.getResult(m_data);
    if (candidateNum > 0) 
    {
      // tf : camera
      tf::StampedTransform st_cam;
      tf::Vector3 t_cam;
      tf::Quaternion r_cam;

      t_cam.setValue(cam_pos_x, cam_pos_y, cam_pos_z);
      r_cam.setRPY((-90*(CV_PI/180)), (0*(CV_PI/180)), (-90*(CV_PI/180)));

      st_cam.setOrigin(t_cam);
      st_cam.setRotation(r_cam);

      if (tf_world_flg == true) {
        tf_br.sendTransform(
          tf::StampedTransform(st_cam, ros::Time::now(), "world", "camera"));
      } else {
        tf_br.sendTransform(
          tf::StampedTransform(st_cam, ros::Time::now(), "camera", "camera"));
      } 

      // topic : markers
      lenti_mark::markers mks;
      lenti_mark::marker mk;
      geometry_msgs::Pose pose_msg;

      mks.header.frame_id = "camera";
      mks.header.stamp = ros::Time::now();
      mks.detect = candidateNum;      

      for (int roop = 0; roop < candidateNum; roop++) 
      {
        // frame_id
        string _frame_id;
        _frame_id = "mk_" + to_string(m_data[roop].id);

        // tf : marker
        double mk_x, mk_y, mk_z;
        mk_x = m_data[roop].x/1000;  // mm -> m
        mk_y = m_data[roop].y/1000;
        mk_z = m_data[roop].z/1000;
        tf::Vector3 mk_orig(mk_x, mk_y, mk_z);

        tf::Matrix3x3 mk_rot(
          m_data[roop].rot[0], m_data[roop].rot[1], m_data[roop].rot[2],
          m_data[roop].rot[3], m_data[roop].rot[4], m_data[roop].rot[5],
          m_data[roop].rot[6], m_data[roop].rot[7], m_data[roop].rot[8]);

        tf::Transform t_mk(mk_rot, mk_orig);
        tf_br.sendTransform(
          tf::StampedTransform(t_mk, ros::Time::now(), "camera", _frame_id));

        // topic : marker
        mk.header.frame_id = "camera";
        mk.child_frame_id = _frame_id;
        mk.header.stamp = ros::Time::now();
        mk.id = m_data[roop].id;
	mk.score = m_data[roop].score;
	mk.contrast = m_data[roop].contrast;
        tf::poseTFToMsg(t_mk, pose_msg);
        mk.pose = pose_msg;

        mks.markers.push_back(mk);
      }
      

      // topic
      data_pub.publish(mks);

      //ROS_INFO("Transform Published");
    }

  }
  catch (char *e)
  {
    cerr << "[EXCEPTION] " << e << endl;
    return -1;
  }

  return 0;
}


// CameraSubscriber
void LentiMarkNode::cameraCallback(
  const sensor_msgs::ImageConstPtr& msg,
  const sensor_msgs::CameraInfoConstPtr& info)
{
  // image
  try {
    image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // CameraInfo
  img_size.height = info->height;
  img_size.width  = info->width;
  cam_matrix = cv::Mat(3, 3, CV_64FC1, (void *) info->K.data());
  dist_coeffs = cv::Mat(1, 5, CV_64FC1, (void *) info->D.data());

  // LMTLibrary
  detect();
}


// Subscriber
void LentiMarkNode::imageCallback( const sensor_msgs::ImageConstPtr& msg )
{
  // image
  try {
    image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // LMTLibrary
  detect();
}


// run
void LentiMarkNode::run()
{
  image_transport::ImageTransport it(nh);

  if ( camparam_type == 0 ) {
    img_sub = it.subscribe("/image_raw", 10, &LentiMarkNode::imageCallback, this);
  } 
  else if ( camparam_type == 1 ) {
    cam_sub = it.subscribeCamera("/image_raw", 10, &LentiMarkNode::cameraCallback, this);
  } 
  else {
    cam_sub = it.subscribeCamera("/image_raw", 10, &LentiMarkNode::cameraCallback, this);
  }
  data_pub = nh.advertise<lenti_mark::markers>("/lenti_mark", 10);
  ros::spin();
}


// main
int main( int argc, char **argv )
{
  ros::init(argc, argv, "lenti_mark");

  LentiMarkNode lmn;
  lmn.run();

  return 0;
}
