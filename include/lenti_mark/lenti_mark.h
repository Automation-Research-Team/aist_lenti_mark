//STD
#include <cstdio>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

// OpenCV
#include <opencv2/opencv.hpp>

// lenti_mark
#include "LentiMarkTracker.h"

using namespace std;

namespace lenti_mark
{
  class LentiMarkNode
  {
  public:
    LentiMarkNode();
    ~LentiMarkNode(){};

    void run(void);

  private:
    int setParams(void);
    int detect(void);
  
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg,  
    const sensor_msgs::CameraInfoConstPtr& info);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  private:
    leag::LentiMarkTracker LMT;
    vector<leag::LentiMarkTracker::ResultData> m_data;

    int camparam_type;
    string cam_file;
    string mk_file;

    cv::Mat image;
    ros::NodeHandle nh;
    ros::Publisher data_pub;
    image_transport::CameraSubscriber cam_sub;
    image_transport::Subscriber img_sub;
    tf::TransformBroadcaster tf_br;

    cv::Size2i img_size;
    cv::Mat cam_matrix;
    cv::Mat dist_coeffs;

    // debug ----------------
    bool tf_world_flg;
    double cam_pos_x;
    double cam_pos_y;
    double cam_pos_z;
    // debug ----------------
  };
}


