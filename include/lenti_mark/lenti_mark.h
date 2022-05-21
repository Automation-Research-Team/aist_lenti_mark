/*!
  \file		lenti_mark.h
  \author	Toshio Ueshiba
*/ 
#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "LentiMarkTracker.h"

namespace lenti_mark
{
/************************************************************************
*  class LentiMarkNode							*
************************************************************************/
class LentiMarkNode
{
  public:
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    
  public:
		LentiMarkNode(const ros::NodeHandle& nh)		;
		~LentiMarkNode()					{}

    void	run();

  private:
    void	cameraCallback(const image_cp& img,  
			       const camera_info_cp& cinfo)		;
    void	imageCallback(const image_cp& img)			;
  
  private:
    ros::NodeHandle			_nh;
    image_transport::ImageTransport	_it;
    image_transport::CameraSubscriber	_camera_sub;
    image_transport::Subscriber		_image_sub;
    const ros::Publisher		_markers_pub;
    tf::TransformBroadcaster		_tf_broadcaster;

    leag::LentiMarkTracker		_LMT;
};
}	// namespace lenti_mark
