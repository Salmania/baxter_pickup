#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorProperties.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <shape_tools/solid_primitive_dims.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <iostream>

class VisualServo
{
private:
  ros::NodeHandle node_;
  image_transport::ImageTransport transporter_;
  image_transport::Subscriber camera_image_sub_;
  image_transport::Publisher filter_pub_;
  ros::Publisher collision_object_pub_;
  ros::Publisher gripper_pub_;
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
  std::string arm_;
  std::string planning_group_name_;
  std::string hand_camera_;
  std::string camera_image_;
  moveit_msgs::CollisionObject collision_object_;
  baxter_core_msgs::EndEffectorCommand gripper_cmd_;

  double error_y_;
  double error_x_;

  tf::StampedTransform target_;
  std::vector<double> last_rpy_;
  std::vector<double> next_rpy_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_transform_;
  geometry_msgs::PoseStamped pose;

public:
  VisualServo() : transporter_(node_),  
		  arm_("right"),
		  planning_group_name_(arm_+"_arm"),
		  hand_camera_(arm_+"_hand_camera"),
		  camera_image_("/cameras/"+hand_camera_+"/image")
  {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(20.0);
    move_group_->setPlannerId("KPIECEkConfigDefault");
    filter_pub_ = transporter_.advertise("Filter", 1);
    camera_image_sub_ = transporter_.subscribe(camera_image_, 1, &VisualServo::tracker, this);

    collision_object_pub_ = node_.advertise<moveit_msgs::CollisionObject>("collision_object_", 10);
    collision_object_.operation = moveit_msgs::CollisionObject::ADD;
    collision_object_.primitives.resize(1);
    collision_object_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_object_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.8382;
    collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.8382;
    collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.7112;
    collision_object_.primitive_poses.resize(1);
    collision_object_.primitive_poses[0].position.x = 0.7;
    collision_object_.primitive_poses[0].position.y = -0.3556;
    collision_object_.primitive_poses[0].position.z = 0.7112;
    collision_object_.primitive_poses[0].orientation.w = 1.0;
    collision_object_pub_.publish(collision_object_);

    gripper_pub_ = node_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10);
    gripper_cmd_.id = 65664;
    gripper_cmd_.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    gripper_pub_.publish(gripper_cmd_);
    ros::Duration(2).sleep();

    //at bottom:
    //Translation: [0.656, -0.082, -0.056]
    //Rotation: in Quaternion [0.765, 0.644, 0.017, 0.019]
    //          in RPY [3.091, -0.001, 1.400]

    //at top:
    // Translation: [0.548, -0.080, 0.504]
    // Rotation: in Quaternion [0.733, 0.679, 0.024, 0.022]
    //        in RPY [3.077, -0.006, 1.494]


    goto_home();
    pick_up();
    while(ros::ok()){
    }
  }

  void goto_home()
  {
    move_group_->setPlanningTime(10.0);
    pose.header.frame_id = "base";  	
    pose.pose.position.x = .656;
    pose.pose.position.y = -.082;
    pose.pose.position.z = .504;
    pose.pose.orientation.x = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0.0274;
    move_group_->setPoseTarget(pose,"right_wrist");
    int tries = 0;
    while (!move_group_->move() && (tries < 6))
      tries += 1;

  }

  void pick_up()
  {
    open_gripper();
    move_group_->setPlanningTime(10.0);
    pose.header.frame_id = "base";  	
    pose.pose.position.x = 0.656-error_y_;
    pose.pose.position.y = -0.082-error_x_;
    pose.pose.position.z = 0.05;
    pose.pose.orientation.x = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0.0274;
    move_group_->setPoseTarget(pose,"right_wrist");
    int tries = 0;
    while (!move_group_->move() && (tries < 6))
      tries += 1;
    close_gripper();
    goto_home();
  }

  void close_gripper()
  {
	gripper_cmd_.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
	gripper_pub_.publish(gripper_cmd_);
  }
  
  void open_gripper()
  {
	gripper_cmd_.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
	gripper_pub_.publish(gripper_cmd_);
  }

  void tracker(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr camera_image;
    try
      {
  	camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
  	ROS_ERROR("cv_bridge exception: %s", e.what());
  	return;
      }

    // threshold image in green range for tracking
    // values taken for light and dark areas on the cube face using GIMP
    // white limits - lower:r = 217, g = 229, b=216 upper: g=g=b=255
    // red limits - lower:r = 148, g = 62, b=61 upper: r = 239, b = 122, g = 140
    const int cap_red_upper = 255;
    const int cap_green_upper = 140;
    const int cap_blue_upper = 120;
    const int cap_red_lower = 130;
    const int cap_green_lower = 50;
    const int cap_blue_lower = 50;
    const int tube_red_upper = 255;
    const int tube_green_upper = 255;
    const int tube_blue_upper = 255;
    const int tube_red_lower = 200;
    const int tube_green_lower = 200;
    const int tube_blue_lower = 200;

    cv::Mat threshold_cap_mat, threshold_tube_mat, threshold_mat;
    cv::inRange(camera_image->image,cv::Scalar(cap_blue_lower,cap_green_lower,cap_red_lower),cv::Scalar(cap_blue_upper,cap_green_upper,cap_red_upper),threshold_cap_mat);

    cv::inRange(camera_image->image,cv::Scalar(tube_blue_lower,tube_green_lower,tube_red_lower),cv::Scalar(tube_blue_upper,tube_green_upper,tube_red_upper),threshold_tube_mat);

    // cv::imshow("Cap Image",threshold_cap_mat);
    // cv::imshow("Tube Image",threshold_tube_mat);
    // cv::waitKey(3);

    std::vector<cv::Mat> image_channels;
    cv::split(camera_image->image,image_channels);

    threshold_mat = threshold_tube_mat + threshold_cap_mat;
    image_channels[0] -= threshold_mat;
    image_channels[1] -= threshold_mat;
    image_channels[2] -= threshold_mat;    

    image_channels[1] += threshold_cap_mat*.5;
    image_channels[1] += threshold_tube_mat;

    cv::merge(image_channels,camera_image->image);
    
    filter_pub_.publish(camera_image->toImageMsg());

    cv::Moments cap_moment,tube_moment;
    cap_moment = moments(threshold_cap_mat);
    tube_moment = moments(threshold_tube_mat);
    double cap_x,cap_y,tube_x,tube_y,length_x,length_y,theta;
    cap_x = cap_moment.m01/cap_moment.m00;
    cap_y = cap_moment.m10/cap_moment.m00;
    tube_x = tube_moment.m01/tube_moment.m00;
    tube_y = tube_moment.m10/tube_moment.m00;
    length_x = threshold_tube_mat.rows/2;
    length_y = threshold_tube_mat.cols/2;
    theta = atan2(tube_y-cap_y,tube_x-cap_x);    
    const double dpp = .00128005616804887;
    error_y_ = dpp*(length_y-tube_y);
    error_x_ = dpp*(length_x-tube_x);

    ROS_INFO("x = %f, y =%f \n", error_x_, error_y_);
    // double cap_mass,tube_mass;
    // cap_mass = cap_moment.m00;
    // tube_mass = tube_moment.m00;
  }


};
    
  int main(int argc, char **argv)
  {
    ros::init (argc, argv, "viz_servo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    VisualServo vizual_servo;
    ros::shutdown();
    return 0;
  }


