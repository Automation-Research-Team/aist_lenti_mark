/*!
  \file		aist_lenti_mark.cpp
  \author	Toshio Ueshiba
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aist_lenti_mark/markers.h>
#include <aist_lenti_mark/marker.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "LentiMarkTracker.h"

namespace aist_lenti_mark
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
		LentiMarkNode(const ros::NodeHandle& nh,
			      const std::string& nodelet_name)		;
		~LentiMarkNode()					{}

  private:
    void	camera_cb(const image_cp& img,
			  const camera_info_cp& cinfo)			;
    void	image_cb(const image_cp& img)				;

    const std::string&
		getName()	const	{ return _nodelet_name; }

  private:
    ros::NodeHandle			_nh;
    const std::string			_nodelet_name;
    const std::string			_marker_frame;
    image_transport::ImageTransport	_it;
    image_transport::CameraSubscriber	_camera_sub;
    image_transport::Subscriber		_image_sub;
    const ros::Publisher		_markers_pub;
    tf2_ros::TransformBroadcaster	_broadcaster;

    leag::LentiMarkTracker		_LMT;
};

LentiMarkNode::LentiMarkNode(const ros::NodeHandle& nh,
			     const std::string& nodelet_name)
    :_nh(nh),
     _nodelet_name(nodelet_name),
     _marker_frame(_nh.param<std::string>("marker_frame", "marker_frame")),
     _it(_nh),
     _camera_sub(_nh.param<bool>("subscribe_camera", false) ?
		 _it.subscribeCamera("/image", 10,
				     &LentiMarkNode::camera_cb, this) :
		 image_transport::CameraSubscriber()),
     _image_sub(_nh.param<bool>("subscribe_camera", false) ?
		image_transport::Subscriber() :
		_it.subscribe("/image_raw", 10, &LentiMarkNode::image_cb, this)),
     _markers_pub(_nh.advertise<aist_lenti_mark::markers>("lenti_mark", 10)),
     _broadcaster(),
     _LMT()
{
    const auto	cam_file = _nh.param<std::string>("cam_param_file", "");
    if (cam_file != "" && _LMT.setCamParams(cam_file))
	throw std::runtime_error("Failed to set camera parameters from file["
				 + cam_file + ']');

    const auto	mk_file = _nh.param<std::string>("mk_param_file", "");
    if (mk_file == "")
	throw std::runtime_error("Marker parameter file is not specified!");
    else if (_LMT.setMarkerParams(mk_file))
	throw std::runtime_error("Failed to set marker parameters from file["
				 + mk_file + ']');

    NODELET_INFO_STREAM('(' << getName()
			<< ") started with camera parameter file[" << cam_file
			<< "] and marker parameter file[" << mk_file << ']');
}

void
LentiMarkNode::camera_cb(const image_cp& img, const camera_info_cp& cinfo)
{
    const cv::Size2i	img_size(cinfo->width, cinfo->height);
    const cv::Mat	cam_matrix( 3, 3, CV_64FC1, (void*)cinfo->K.data());
    const cv::Mat	dist_coeffs(1, 5, CV_64FC1, (void*)cinfo->D.data());
    _LMT.setCamParams(img_size, cam_matrix, dist_coeffs);

    image_cb(img);
}

void
LentiMarkNode::image_cb(const image_cp& img)
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
	    aist_lenti_mark::markers	markers;
	    markers.header = img->header;
	    markers.detect = m_data.size();

	    for (const auto& data : m_data)
	    {
	      // Broadcast transform from marker frame to camera frame.
		const tf2::Stamped<tf2::Transform>
			transform{tf2::Transform{
				    {data.rot[0], data.rot[1], data.rot[2],
				     data.rot[3], data.rot[4], data.rot[5],
				     data.rot[6], data.rot[7], data.rot[8]},
				    {data.x*0.001, data.y*0.001, data.z*0.001}},
				  img->header.stamp, img->header.frame_id};
		auto	transform_msg = tf2::toMsg(transform);
		transform_msg.child_frame_id = _marker_frame
					     + '_' + std::to_string(data.id);
		_broadcaster.sendTransform(transform_msg);

	      // Construct a marker element and push it into marker array.
		geometry_msgs::Pose	pose_msg;
		tf2::toMsg(transform, pose_msg);

		aist_lenti_mark::marker	marker;
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
	    NODELET_WARN_STREAM('(' << getName() << ") no markers detected.");
    }
    catch (const std::exception& err)
    {
	NODELET_ERROR_STREAM('(' << getName() << ") " << err.what());
    }
}

/************************************************************************
*  class LentiMarkNodelet						*
************************************************************************/
class LentiMarkNodelet : public nodelet::Nodelet
{
  public:
			LentiMarkNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<LentiMarkNode>	_node;
};

void
LentiMarkNodelet::onInit()
{
    NODELET_INFO("aist_lenti_mark::LentiMarkNodelet::onInit()");
    _node.reset(new LentiMarkNode(getPrivateNodeHandle(), getName()));
}

}	// namespace aist_lenti_mark

PLUGINLIB_EXPORT_CLASS(aist_lenti_mark::LentiMarkNodelet, nodelet::Nodelet);
