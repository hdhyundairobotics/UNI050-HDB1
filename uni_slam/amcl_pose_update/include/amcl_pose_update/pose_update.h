#ifndef _POSE_UPDATE_H_
#define _POSE_UPDATE_H_

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"
#include "ros/package.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h" // add
#include "visualization_msgs/Marker.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "message_filters/subscriber.h"


// add for json save
#include <iostream>
#include <fstream>
#include "json/json.h"
#include "json/json-forwards.h"

#pragma  comment(lib,"jsoncpp.lib")

//#define M_PI 3.14159265359

using namespace std;

typedef struct Point2D
{
	double x;
	double y;

	Point2D()
	{
		init();
	}
	~Point2D()
	{

	}

	void init()
	{
		x = 0;
		y = 0;
	}
}Point2D;

typedef struct TAG_INFO
{
  int id;
	double position_x;
	double position_y;
  double position_theta;

  bool save_flag;

	TAG_INFO()
	{
		init();
	}
	~TAG_INFO()
	{

	}

	void init()
	{
	  int id = 100000;
	  double position_x = 0.;
	  double position_y = 0.;
    double position_theta = 0.;

    bool save_flag = false;
	}
}TAG_INFO;

class PoseUpdate
{
  public:
    PoseUpdate();
    ~PoseUpdate();
    void sbl236_signalhandler(int sig);

    void load_param();

    void tagCenterCallback(const std_msgs::Bool& msg); // add
    void tagIDCallback(const std_msgs::Int32& msg); // add
    void tagDetectCallback(const std_msgs::Bool& msg);
    void tagInfoCallback(const geometry_msgs::Pose2D& msg);
    void AMCLPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void checkUpdateSaveTagInfo( int &check_cnt, TAG_INFO c_tag, TAG_INFO jSaveTag_Info[] ); // add
    void poseUpdater( TAG_INFO src_tag ); // add
    TAG_INFO calc_AMCLPosefromTagPose( TAG_INFO src_tag ); // change
    void readJson_TagPose(std::string filepath, TAG_INFO jTag[] ); // add
    void saveJson_TagPose( std::string filepath, int nTotal_tag, TAG_INFO jTag[]) ;  // add
    void initialPosePublish( TAG_INFO c_tag, TAG_INFO jTag[] ); // add

    int run();

  protected:

    ros::NodeHandle nh;

    std::string n_space;

    std::shared_ptr<tf2_ros::Buffer> tf_;

    ros::Subscriber tag_detect_sub;
    std::string tag_detect_sub_topic;

    ros::Subscriber tag_center_sub; // add
    std::string tag_center_sub_topic; // add

    ros::Subscriber tag_info_sub;
    std::string tag_info_sub_topic;

    ros::Subscriber tag_id_sub; // add
    std::string tag_id_sub_topic; // add

    ros::Subscriber amcl_pose_sub;
    std::string amcl_pose_sub_topic;

    ros::Publisher initialpose_pub;
    std::string initialpose_pub_topic;

    ros::Publisher marker_pub;
    std::string marker_pub_topic;

    geometry_msgs::PoseWithCovarianceStamped sub_pose;
    geometry_msgs::PoseWithCovarianceStamped tag_pose;

    bool isTag_detect; // change
    bool isTag_center; // add
    double tag_angle;
    double tag_x;
    double tag_y;

    double tag_map_angle;
    double tag_map_yaw;
    double tag_map_x;
    double tag_map_y;

    double cam_offset;
    double tag_position_x;
    double tag_position_y;
    int tag_id; // add
    int nTotal_tag_num;
    // double sub_pose_degree;


    // running param
    bool running_flag; // add

    // Json Path
    string jsonFilePath;

    // Tag Info Save
    TAG_INFO* jReadTag_Info;
    TAG_INFO* jSaveTag_Info;

    int check_cnt;
    float tag_margin;

    int update_flag;
    int b_tagID;

    // tf2::Quaternion sub_pose;
};

#endif  // _POSE_UPDATE_H_