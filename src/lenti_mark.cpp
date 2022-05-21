/*!
  \file		lenti_mark.cpp
  \author	Toshio Ueshiba
*/
#include "lenti_mark.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <lenti_mark/markers.h>
#include <lenti_mark/marker.h>

namespace lenti_mark
{
/************************************************************************
*  class LentiMarkNode							*
************************************************************************/
LentiMarkNode::LentiMarkNode(const ros::NodeHandle& nh)
    :_nh(nh),
     _it(_nh),
     _camera_sub(_it.subscribeCamera("/image_raw", 10,
				     &LentiMarkNode::cameraCallback, this)),
     _image_sub(_it.subscribe("/image_raw", 10,
			      &LentiMarkNode::imageCallback, this)),
     _markers_pub(_nh.advertise<lenti_mark::markers>("lenti_mark", 10)),
     _tf_broadcaster(),
     _LMT()
{
    const auto	cam_file = _nh.param<std::string>("cam_param_file", "");
    if (cam_file != "" && _LMT.setCamParams(cam_file))
	throw std::runtime_error("Failed to set camera parameters from file["
				 + cam_file + ']');
    
    const auto	mk_file = _nh.param<std::string>("mk_param_file", "");
    if (mk_file != "")
	throw std::runtime_error("Marker parameter file is not specified!");
    else if (_LMT.setMarkerParams(mk_file))
	throw std::runtime_error("Failed to set marker parameters from file["
				 + mk_file + ']');
}

void
LentiMarkNode::run()
{
    ros::spin();
}

void
LentiMarkNode::cameraCallback(const image_cp& img, const camera_info_cp& cinfo)
{
    const cv::Size2i	img_size(cinfo->width, cinfo->height);
    const cv::Mat	cam_matrix( 3, 3, CV_64FC1, (void*)cinfo->K.data());
    const cv::Mat	dist_coeffs(1, 5, CV_64FC1, (void*)cinfo->D.data());
    _LMT.setCamParams(img_size, cam_matrix, dist_coeffs);
    
    imageCallback(img);
}

void
LentiMarkNode::imageCallback(const image_cp& img)
{
    try
    {
	using namespace	sensor_msgs;

      // Convert the input image message to cv::Mat.
	const auto image = cv_bridge::toCvShare(img, image_encodings::BGR8)
			 ->image;

      // Check image size.
	if (image.rows < 1 || image.cols < 1)
	    throw std::runtime_error("illegal image size["
				     + std::to_string(image.cols) + 'x'
				     + std::to_string(image.rows) + ']');

      // Detect markers.
	if (_LMT.detect(image) < 0)
	    throw std::runtime_error("LentiMarkTracker::detect() failed.");

      // Get marker detection results.
	std::vector<leag::LentiMarkTracker::ResultData>	m_data;
	_LMT.getResult(m_data);

	if (m_data.size() > 0) 
	{
	  // Construct a message of marker array.
	    lenti_mark::markers	markers;
	    markers.header = img->header;
	    markers.detect = m_data.size();      

	    for (const auto& data : m_data) 
	    {
	      // Broadcast transform from marker frame to camera frame.
		const auto marker_frame_id = "mk_" + std::to_string(data.id);
		const tf::Transform
		    transform({data.rot[0], data.rot[1], data.rot[2],
			       data.rot[3], data.rot[4], data.rot[5],
			       data.rot[6], data.rot[7], data.rot[8]},
			      {data.x/1000, data.y/1000, data.z/1000});
		_tf_broadcaster.sendTransform(
		    tf::StampedTransform(transform, img->header.stamp,
					 img->header.frame_id,
					 marker_frame_id));

	      // Construct a marker element and push it into marker array.
		geometry_msgs::Pose	pose_msg;
		tf::poseTFToMsg(transform, pose_msg);

		lenti_mark::marker	marker;
		marker.id	= data.id;
		marker.score	= data.score;
		marker.contrast	= data.contrast;
		marker.pose	= pose_msg;
		
		markers.markers.push_back(marker);
	    }

	  // Publish marker array.
	    _markers_pub.publish(markers);
	}
	else
	    ROS_WARN_STREAM("(lenti_mark) no markers detected.");
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(lenti_mark) " << err.what());
    }
}

}	// namespace lenti_mark

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char *argv[])
{
    ros::init(argc, argv, "lenti_mark");

    try
    {
	ros::NodeHandle			nh("~");
	lenti_mark::LentiMarkNode	node(nh);

	node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
