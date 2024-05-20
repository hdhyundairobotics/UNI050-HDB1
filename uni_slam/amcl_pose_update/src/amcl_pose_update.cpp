#include "amcl_pose_update/pose_update.h"

PoseUpdate::PoseUpdate():
  isTag_detect(false),
  isTag_center(false),
  tag_angle(0.0),
  tag_map_angle(0.0),
  cam_offset(0.0), //260.0
  nTotal_tag_num(0),
  check_cnt(0),
  tag_margin(0.0),
  update_flag(0),
  b_tagID(100000)
{
  ros::NodeHandle nhLocal("~");

  nhLocal.param<std::string>("tag_detect_sub_topic", tag_detect_sub_topic, "/tag_detect");
  nhLocal.param<std::string>("tag_info_sub_topic", tag_info_sub_topic, "/tag_position");
  nhLocal.param<std::string>("tag_center_sub_topic", tag_center_sub_topic, "/tag_center"); // add
  nhLocal.param<std::string>("tag_id_sub_topic", tag_id_sub_topic, "/tag_id"); // add
  nhLocal.param<std::string>("amcl_pose_sub_topic", amcl_pose_sub_topic, "/amcl_pose");

  nhLocal.param<std::string>("initialpose_pub_topic", initialpose_pub_topic, "/initialpose");

  tag_detect_sub = nh.subscribe(tag_detect_sub_topic, 1, &PoseUpdate::tagDetectCallback, this);
  tag_info_sub = nh.subscribe(tag_info_sub_topic, 1, &PoseUpdate::tagInfoCallback, this);
  tag_center_sub = nh.subscribe(tag_center_sub_topic, 1, &PoseUpdate::tagCenterCallback, this); // add
  tag_id_sub = nh.subscribe(tag_id_sub_topic, 1, &PoseUpdate::tagIDCallback, this); // add

  amcl_pose_sub = nh.subscribe(amcl_pose_sub_topic, 1, &PoseUpdate::AMCLPoseCallback, this);

  initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_pub_topic, 1);

  auto package_path = ros::package::getPath("amcl_pose_update");
  jsonFilePath = package_path + "/config/TagInfo.json";

  load_param();
  tag_margin = tag_margin*0.001;

  jReadTag_Info = new TAG_INFO[nTotal_tag_num];
  jSaveTag_Info = new TAG_INFO[nTotal_tag_num];

  for(int i=0; i<nTotal_tag_num; i++)
  {
    jReadTag_Info[i].init();
    jSaveTag_Info[i].init();
  }

  b_tagID = 100000;
  tag_id = 100000;

  //saveJson_TagPose(jsonFilePath);
  //readJson_TagPose(jsonFilePath, 0);
}

PoseUpdate::~PoseUpdate()
{
  delete jReadTag_Info;
  delete jSaveTag_Info;

  // delete tag_detect_sub;
  // delete tag_info_sub;
  // delete amcl_pose_sub;
  // TODO: delete everything allocated in constructor
}

void PoseUpdate::load_param()
{
  // Param load
  n_space = ros::this_node::getName();
  ROS_INFO_STREAM("name_space: " << n_space);

  nh.param<int>("total_tag_num", nTotal_tag_num, 0);
  nh.getParam(n_space + "/total_tag_num", nTotal_tag_num);
  ROS_INFO_STREAM("total_tag_num: " << nTotal_tag_num);

  nh.param<float>("tag_center_margin", tag_margin, 50.0);
  nh.getParam(n_space + "/tag_center_margin", tag_margin);
  ROS_INFO_STREAM("tag_center_margin: " << tag_margin);

  nh.param<bool>("amcl_pose_update_running_flag", running_flag, true);
  nh.getParam(n_space + "/amcl_pose_update_running_flag", running_flag);
  ROS_INFO_STREAM("amcl_pose_update_running_flag: " << running_flag);
}

void PoseUpdate::tagCenterCallback(const std_msgs::Bool& msg) // add
{
  isTag_center = msg.data;
}

void PoseUpdate::tagIDCallback(const std_msgs::Int32& msg) // add
{
  tag_id = msg.data;
}

void PoseUpdate::tagDetectCallback(const std_msgs::Bool& msg)
{
  isTag_detect = msg.data;

  // Tag가 detect 되었을대 updater 수행
  if(isTag_detect)
  {
    TAG_INFO detect_tag;
    detect_tag.init();

    detect_tag.id = tag_id;
    detect_tag.position_x = tag_x;
    detect_tag.position_y = tag_y;
    detect_tag.position_theta = tag_angle;

    poseUpdater( detect_tag );
  }
}

void PoseUpdate::tagInfoCallback(const geometry_msgs::Pose2D& msg)
{
  tag_x = (double)msg.x * 0.001 * -1.0;  //offset
  tag_y = (double)msg.y * 0.001;
  tag_angle = (double)msg.theta;
}

void PoseUpdate::AMCLPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  sub_pose = msg;

  // ROS_INFO("AMCL POSE x: %f y: %f z: %f w: %f", msg.pose.pose.position.x,
  // msg.pose.pose.position.y, msg.pose.pose.orientation.z,
  // msg.pose.pose.orientation.w);

  // ROS_INFO("sub_pose POSE x: %f y: %f z: %f w: %f", sub_pose.pose.pose.position.x,
  // sub_pose.pose.pose.position.y, sub_pose.pose.pose.orientation.z,
  // sub_pose.pose.pose.orientation.w);
}

void PoseUpdate::checkUpdateSaveTagInfo( int &check_cnt, TAG_INFO c_tag, TAG_INFO jSTag_Info[] )
{
  int id;
	double position_x;
	double position_y;
  double position_theta;

  int save_cnt = 0;
  bool check_flag = false;

  if(check_cnt < nTotal_tag_num)
  {
    if(check_cnt == 0)
    {
      jSTag_Info[check_cnt].id = c_tag.id;
      jSTag_Info[check_cnt].position_x = c_tag.position_x;
      jSTag_Info[check_cnt].position_y = c_tag.position_y;
      jSTag_Info[check_cnt].position_theta = c_tag.position_theta;
      jSTag_Info[check_cnt].save_flag = true;

      check_cnt++;
    }
    else
    {
      int _idx = 0;

      for(int cnt = 0; cnt < nTotal_tag_num; cnt++ )
      {
        if( jSTag_Info[cnt].id == c_tag.id )
        {
          check_flag = true;
        }
        if(jSTag_Info[cnt].save_flag == false)
        {
          _idx = cnt;
        }
      }


      if(!check_flag)
      {
        jSTag_Info[_idx].id = c_tag.id;
        jSTag_Info[_idx].position_x = c_tag.position_x;
        jSTag_Info[_idx].position_y = c_tag.position_y;
        jSTag_Info[_idx].position_theta = c_tag.position_theta;
        jSTag_Info[_idx].save_flag = true;

        check_cnt = _idx++;
      }
    }
  }
  else if(check_cnt == nTotal_tag_num)  check_cnt = 0;
}


void PoseUpdate::poseUpdater( TAG_INFO src_tag )
{
  if(running_flag)
  {
    // read json param
    readJson_TagPose(jsonFilePath, jReadTag_Info);
    //ROS_INFO("X: %f, Y: %f, Margin : %f", src_tag.position_x, src_tag.position_y, tag_margin);

    if( (-1.0*tag_margin) < src_tag.position_x
       && src_tag.position_x < tag_margin
       && (-1.0*tag_margin) < src_tag.position_y
       && src_tag.position_y < tag_margin )
       {
         //ROS_INFO("X: %f, Y: %f, Margin : %f", src_tag.position_x, src_tag.position_y, tag_margin);
	 if( src_tag.id == 0 || src_tag.id == 20 )
	 {
           if( b_tagID != src_tag.id )
       	   {
          	 b_tagID = src_tag.id;

	         initialPosePublish( src_tag, jReadTag_Info);
	   }
         }
       }
  }
  else
  {
    readJson_TagPose(jsonFilePath, jSaveTag_Info);

    if(isTag_center)
    {
      TAG_INFO _tag;
      _tag.init();

      _tag = calc_AMCLPosefromTagPose(src_tag);
      checkUpdateSaveTagInfo( check_cnt, _tag, jSaveTag_Info );

      //void PoseUpdate::saveJson_TagPose(std::string filepath, int nTotal_tag, TAG_INFO jTag[])
      saveJson_TagPose(jsonFilePath, nTotal_tag_num, jSaveTag_Info);
    }
  }
}

TAG_INFO PoseUpdate::calc_AMCLPosefromTagPose( TAG_INFO src_tag )
{
  TAG_INFO dst_tag;
  dst_tag.init();

  tag_pose.pose.pose.position.x = sub_pose.pose.pose.position.x;
  tag_pose.pose.pose.position.y = sub_pose.pose.pose.position.y;

  tag_pose.pose.pose.orientation.z = sub_pose.pose.pose.orientation.z;
  tag_pose.pose.pose.orientation.w = sub_pose.pose.pose.orientation.w;

  // ROS_INFO("sub_pose x: %f y: %f yaw: %f", sub_pose.pose.pose.position.x,
  // sub_pose.pose.pose.position.y, tf2::getYaw(tag_pose.pose.pose.orientation));

  tag_map_yaw = src_tag.position_theta * M_PI / 180.0;
  tag_map_yaw = tf2::getYaw(tag_pose.pose.pose.orientation) - tag_map_yaw;

  tag_map_angle = tag_map_yaw / M_PI * 180.0;

  // tag_position_x = tag_x;
  // tag_position_y = tag_y;
  //tag_position_x = src_tag.position_x;
  tag_position_x = src_tag.position_x - (cam_offset * 0.001); // offset
  tag_position_y = src_tag.position_y;

  // tag_map_x = tag_pose.pose.pose.position.x - (cos(tag_map_angle) * tag_x);
  // tag_map_y = tag_pose.pose.pose.position.y - (sin(tag_map_angle) * tag_y);
  tag_map_x = tag_pose.pose.pose.position.x - (cos(tag_map_angle) * tag_position_x);
  tag_map_y = tag_pose.pose.pose.position.y - (sin(tag_map_angle) * tag_position_y);

  // ROS_INFO("tag_map_x: %f tag_map_y: %f tag_map_yaw: %f", tag_map_x,
  // tag_map_y, tag_map_yaw);


  // dst에 업데이트
  dst_tag.id = src_tag.id;
  dst_tag.position_x = tag_map_x;
  dst_tag.position_y = tag_map_y;
  dst_tag.position_theta = tag_map_angle;

  dst_tag.save_flag = false;

  //ROS_INFO("dst_tag : %f", dst_tag.position_theta);

  return dst_tag;
}

void PoseUpdate::initialPosePublish( TAG_INFO c_tag, TAG_INFO jTag[] )
{
  TAG_INFO tag_data;
  tag_data.init();

  //ROS_INFO("Detect Angle : %f", c_tag.position_theta);

  // boost::recursive_mutex::scoped_lock pl(configuration_mutex_);
  for(int i=0; i < nTotal_tag_num; i++)
  {
    if( jTag[i].id == c_tag.id )
    {
      tag_data.id = jTag[i].id;
      tag_data.position_x = jTag[i].position_x;
      tag_data.position_y = jTag[i].position_y;
      tag_data.position_theta = jTag[i].position_theta;
    }
  }

  geometry_msgs::PoseWithCovarianceStamped int_pose_msg;

  int_pose_msg.header.frame_id = "map";

  // double tag_yaw = tag_angle * M_PI / 180.0;
  // double current_yaw = tag_map_yaw + tag_yaw;
  // double current_angle = tag_angle + tag_map_angle;
  double tag_yaw = c_tag.position_theta * M_PI / 180.0;
  double tag_map_yaw = tag_data.position_theta * M_PI / 180.0;
  double current_yaw = tag_map_yaw + tag_yaw;
  double current_angle = c_tag.position_theta + tag_data.position_theta;

  tf2::Quaternion qr;
  qr.setRPY(0, 0, current_yaw);
  tf2::convert(qr, int_pose_msg.pose.pose.orientation);

  // double current_x = tag_map_x - (cos(current_angle) * (tag_position_x - tag_x));
  // double current_y = tag_map_y - (sin(current_angle) * (tag_position_y - tag_y));

  // double current_x = tag_map_x + (cos(current_angle) * (tag_position_x - tag_x)) + (cos(tag_map_angle) * tag_position_x);
  // double current_y = tag_map_y + (sin(current_angle) * (tag_position_x - tag_x)) + (sin(tag_map_angle) * tag_position_y);

  // double current_x = tag_data.position_x + (cos(current_angle) * (tag_data.position_x - c_tag.position_x)) + (cos(tag_data.position_theta) * c_tag.position_x);
  // double current_y = tag_data.position_y + (sin(current_angle) * (tag_data.position_x - c_tag.position_x)) + (sin(tag_data.position_theta) * c_tag.position_y);

  c_tag.position_x = c_tag.position_x - (cam_offset * 0.001); // offset
  double current_x = tag_data.position_x + (cos(current_angle) * (tag_position_x - c_tag.position_x)) + (cos(c_tag.position_theta) * tag_position_x);
  double current_y = tag_data.position_y + (sin(current_angle) * (tag_position_x - c_tag.position_x)) + (sin(c_tag.position_theta) * tag_position_x);

  int_pose_msg.pose.pose.position.x = current_x;
  int_pose_msg.pose.pose.position.y = current_y;

  // ROS_INFO("int_pose_msg x: %f y: %f yaw: %f", int_pose_msg.pose.pose.position.x,
  // int_pose_msg.pose.pose.position.y, current_yaw);

  int_pose_msg.pose.covariance = {
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
  };

  sleep(1.0);

  initialpose_pub.publish(int_pose_msg);
  ROS_INFO("ID: %d, PositionX: %f, PositionY: %f, PositionYaw: %f", tag_data.id, int_pose_msg.pose.pose.position.x, int_pose_msg.pose.pose.position.y, current_yaw);
  ROS_INFO("AMCL Pose Updated!");

}

void PoseUpdate::readJson_TagPose(std::string filepath, TAG_INFO jTag[] )
{
  ifstream json_dir(filepath);
	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value value;

	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, json_dir, &value, &errs);

	if (ok == true)
	{
    int nTotalNum = value["Total_num"].asInt();
    //cout << "Total_num : " << nTotalNum << endl;

    if(nTotalNum > 0)
    {
      int jTag_ID = 0;
      float fjTag_positionX = 0.;
      float fjTag_positionY = 0.;
      float fjTag_positionAngle = 0.;

      for(int cnt = 0; cnt < nTotalNum; cnt++)
      {
        string sjTag_info = "null";

        Json::Value jTagList = value["Tag_Sub_Info"][cnt];

        sjTag_info = jTagList["TagID"].asString();
        jTag_ID = atoi(sjTag_info.c_str());
        //cout << "Tag_ID : " << jTag_ID << endl;

        sjTag_info = jTagList["PositionX"].asString();
        fjTag_positionX = atof(sjTag_info.c_str());
        //cout << "PositionX : " << fjTag_positionX << endl;

        sjTag_info = jTagList["PositionY"].asString();
        fjTag_positionY = atof(sjTag_info.c_str());
        //cout << "PositionY : " << fjTag_positionY << endl;

        sjTag_info = jTagList["RotationAngle"].asString();
        fjTag_positionAngle = atof(sjTag_info.c_str());
        //cout << "RotationAngle : " << fjTag_positionAngle << endl;

        jTag[cnt].id = jTag_ID;
        jTag[cnt].position_x = fjTag_positionX;
        jTag[cnt].position_y = fjTag_positionY;
        jTag[cnt].position_theta = fjTag_positionAngle;
      }
    }
  }
}

void PoseUpdate::saveJson_TagPose(std::string filepath, int nTotal_tag, TAG_INFO jTag[])
{
  Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "    ";  // Tab

  unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

  ofstream json_file;
	json_file.open(filepath);

  Json::Value Info;
  Info["Total_num"] = nTotal_tag;

  Json::Value Sub_Info;
  Json::Value Tag_;

  for(int i=0; i<nTotal_tag; i++)
  {
    //Json::Value &tag_sub_name[0];
    Tag_["TagID"] = jTag[i].id;
    Tag_["PositionX"] = jTag[i].position_x;
    Tag_["PositionY"] = jTag[i].position_y;
    Tag_["RotationAngle"] = jTag[i].position_theta;

    Sub_Info.append(Tag_);
  }

  Info["Tag_Sub_Info"] = Sub_Info;

  writer->write(Info, &json_file);
	json_file.close();

  ROS_INFO("Complete SaveJson TagPose");
}
