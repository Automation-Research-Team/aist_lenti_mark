/*!
  \file		get_screen_corners.cpp
  \author	Toshio Ueshiba
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <aist_lenti_mark/markers.h>
#include <aist_lenti_mark/QuadStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <LentiMarkTracker.h>
#include <visualization_msgs/Marker.h>

namespace aist_lenti_mark
{
static geometry_msgs::Point
toMsg(const cv::Point3f& p)
{
    geometry_msgs::Point	point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;

    return point;
}

static geometry_msgs::Pose
identity()
{
    geometry_msgs::Pose	pose;
    pose.position.x    = 0;
    pose.position.y    = 0;
    pose.position.z    = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    return pose;
}

/************************************************************************
*  class GetScreenCornersNode						*
************************************************************************/
class GetScreenCornersNode
{
  private:
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using vis_marker_t	 = visualization_msgs::Marker;
    using quad_t	 = QuadStamped;

  public:
		GetScreenCornersNode(const ros::NodeHandle& nh,
				     const std::string& nodelet_name)	;
		~GetScreenCornersNode()					{}

  private:
    void	camera_cb(const image_cp& img,
			  const camera_info_cp& cinfo)			;
    void	image_cb(const image_cp& img)				;
    std::array<cv::Point3f, 4>
		get_screen_corners(int marker_id,
				   const cv::Point3f& marker_pos,
				   const cv::Matx33f& marker_rot,
				   int image_width, int image_height)	;
    cv::Point3f	get_point_on_screen(int marker_id,
				    const cv::Point3f& marker_pos,
				    const cv::Matx33f& marker_rot,
				    const cv::Point2f& image_point)	;
    const std::string&
		getName()	const	{ return _nodelet_name; }

  private:
    ros::NodeHandle			_nh;
    const std::string			_nodelet_name;
    const std::string			_marker_frame;
    const float				_screen_offset;
    image_transport::ImageTransport	_it;
    image_transport::CameraSubscriber	_camera_sub;
    image_transport::Subscriber		_image_sub;
    const ros::Publisher		_quad_pub;
    const ros::Publisher		_vis_marker_pub;
    tf2_ros::TransformBroadcaster	_broadcaster;
    leag::LentiMarkTracker		_tracker;
};

GetScreenCornersNode::GetScreenCornersNode(const ros::NodeHandle& nh,
					   const std::string& nodelet_name)
    :_nh(nh),
     _nodelet_name(nodelet_name),
     _marker_frame(_nh.param<std::string>("marker_frame", "marker_frame")),
     _screen_offset(_nh.param<float>("screen_offset", 0.025)),
     _it(_nh),
     _camera_sub(_nh.param<bool>("subscribe_camera", false) ?
		 _it.subscribeCamera("/image", 10,
				     &GetScreenCornersNode::camera_cb, this) :
		 image_transport::CameraSubscriber()),
     _image_sub(_nh.param<bool>("subscribe_camera", false) ?
		image_transport::Subscriber() :
		_it.subscribe("/image_raw", 10,
			      &GetScreenCornersNode::image_cb, this)),
     _quad_pub(_nh.advertise<quad_t>("quad", 1)),
     _vis_marker_pub(_nh.advertise<vis_marker_t>("marker", 1)),
     _broadcaster(),
     _tracker()
{
    const auto	cam_file = _nh.param<std::string>("cam_param_file", "");
    if (cam_file != "" && _tracker.setCamParams(cam_file))
	throw std::runtime_error("Failed to set camera parameters from file["
				 + cam_file + ']');

    const auto	mk_file = _nh.param<std::string>("mk_param_file", "");
    if (mk_file == "")
	throw std::runtime_error("Marker parameter file is not specified!");
    else if (_tracker.setMarkerParams(mk_file))
	throw std::runtime_error("Failed to set marker parameters from file["
				 + mk_file + ']');

    NODELET_INFO_STREAM('(' << getName()
			<< ") started with camera parameter file[" << cam_file
			<< "] and marker parameter file[" << mk_file << ']');
}

void
GetScreenCornersNode::camera_cb(const image_cp& img,
				const camera_info_cp& cinfo)
{
    const cv::Size2i	img_size(cinfo->width, cinfo->height);
    const cv::Mat	cam_matrix( 3, 3, CV_64FC1, (void*)cinfo->K.data());
    const cv::Mat	dist_coeffs(1, 5, CV_64FC1, (void*)cinfo->D.data());
    _tracker.setCamParams(img_size, cam_matrix, dist_coeffs);

    image_cb(img);
}

void
GetScreenCornersNode::image_cb(const image_cp& img)
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
	if (_tracker.detect(image) < 0)
	    throw std::runtime_error("LentiMarkTracker::detect() failed.");

      // Get marker detection results.
	std::vector<leag::LentiMarkTracker::ResultData>	m_data;
	if (_tracker.getResult(m_data) == 0)
	    throw std::runtime_error("No markers detected.");

      // Get marker position and rotation.
	const auto&	  data = m_data[0];
	const cv::Point3f marker_pos(0.001*data.x, 0.001*data.y, 0.001*data.z);
	const cv::Matx33f marker_rot(data.rot[0], data.rot[1], data.rot[2],
				     data.rot[3], data.rot[4], data.rot[5],
				     data.rot[6], data.rot[7], data.rot[8]);

      // Get screen corner points.
	const auto	corners = get_screen_corners(data.id,
						     marker_pos, marker_rot,
						     image.cols, image.rows);

      // Broadcast transform from camera frame to marker frame.
	const tf2::Stamped<tf2::Transform>
	    transform{tf2::Transform{
			{marker_rot(0, 0), marker_rot(0, 1), marker_rot(0, 2),
			 marker_rot(1, 0), marker_rot(1, 1), marker_rot(1, 2),
			 marker_rot(2, 0), marker_rot(2, 1), marker_rot(2, 2)},
			{marker_pos.x, marker_pos.y, marker_pos.z}}.inverse(),
			img->header.stamp,
			_marker_frame + '_' + std::to_string(data.id)};
	auto transform_msg = tf2::toMsg(transform);
	transform_msg.child_frame_id = img->header.frame_id;
	_broadcaster.sendTransform(transform_msg);

      // Create and publish quad
	quad_t	quad;
	quad.header	  = transform_msg.header;
	quad.top_left	  = toMsg(corners[0]);
	quad.bottom_left  = toMsg(corners[1]);
	quad.bottom_right = toMsg(corners[2]);
	quad.top_right	  = toMsg(corners[3]);
	_quad_pub.publish(quad);

      // Create and publish visualization marker.
	vis_marker_t	vis_marker;
	vis_marker.header	= transform_msg.header;
	vis_marker.ns		= "vis_marker";
	vis_marker.id		= 0;
	vis_marker.type		= vis_marker_t::LINE_STRIP;
	vis_marker.action	= vis_marker_t::ADD;
	vis_marker.pose		= identity();
	vis_marker.scale.x	= 0.005;
	vis_marker.color.r	= 1.0;
	vis_marker.color.g	= 1.0;
	vis_marker.color.b	= 0.0;
	vis_marker.color.a	= 1.0;
	vis_marker.lifetime	= ros::Duration(0.0);
	vis_marker.frame_locked	= false;

	for (const auto& corner : corners)
	    vis_marker.points.push_back(toMsg(corner));
	vis_marker.points.push_back(toMsg(corners[0]));

	_vis_marker_pub.publish(vis_marker);
    }
    catch (const std::exception& err)
    {
	NODELET_ERROR_STREAM('(' << getName() << ") " << err.what());
    }
}

std::array<cv::Point3f, 4>
GetScreenCornersNode::get_screen_corners(int marker_id,
					 const cv::Point3f& marker_pos,
					 const cv::Matx33f& marker_rot,
					 int image_width, int image_height)
{
    return {get_point_on_screen(marker_id, marker_pos, marker_rot,
				cv::Point2f(0, 0)),
	    get_point_on_screen(marker_id, marker_pos, marker_rot,
				cv::Point2f(0, image_height)),
	    get_point_on_screen(marker_id, marker_pos, marker_rot,
				cv::Point2f(image_width, image_height)),
	    get_point_on_screen(marker_id, marker_pos, marker_rot,
				cv::Point2f(image_width, 0))};
}

cv::Point3f
GetScreenCornersNode::get_point_on_screen(int marker_id,
					  const cv::Point3f& marker_pos,
					  const cv::Matx33f& marker_rot,
					  const cv::Point2f& image_point)
{
  // Get 3D point on the marker plane correnponding to the image point.
    cv::Point3f	screen_point;
    _tracker.getPositionOnMarkerPlane(marker_id, image_point, screen_point);

  // Scale the 3D point so that it lies on the screen.
    screen_point *= 0.001;	// milimeters => meters
    screen_point *= (1.0f + _screen_offset/marker_rot.col(2).dot(screen_point));

  // Transform the screen point to the marker corrdinate frame.
    return marker_rot.t() * (screen_point - marker_pos);
}

/************************************************************************
*  class GetScreenCornersNodelet					*
************************************************************************/
class GetScreenCornersNodelet : public nodelet::Nodelet
{
  public:
			GetScreenCornersNodelet()			{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<GetScreenCornersNode>	_node;
};

void
GetScreenCornersNodelet::onInit()
{
    NODELET_INFO("aist_lenti_mark::GetScreenCornersNodelet::onInit()");
    _node.reset(new GetScreenCornersNode(getPrivateNodeHandle(), getName()));
}

}	// namespace aist_lenti_mark

PLUGINLIB_EXPORT_CLASS(aist_lenti_mark::GetScreenCornersNodelet,
		       nodelet::Nodelet);
