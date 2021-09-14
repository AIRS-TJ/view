#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Matrix3d r_drift_odom;
Matrix3d r_drift_bluerov;
double marker_scale=1.0;

ros::Subscriber odom_sub;
ros::Subscriber pose_sub;
ros::Publisher odom_pub;
ros::Publisher pose_pub;
ros::Publisher pub_odom_pose_visual;
ros::Publisher pub_bluerov_pose_visual;
nav_msgs::Path odom_path_msg;
nav_msgs::Path pose_path_msg;

void Odom_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom_path_msg.header=odom_msg->header;
	geometry_msgs::PoseStamped pose;
	pose.header=odom_msg->header;
	pose.pose=odom_msg->pose.pose;
	odom_path_msg.poses.push_back(pose);
    odom_pub.publish(odom_path_msg);
	//marker visual
	Vector3d tic;
    Matrix3d qic;
	tic = Vector3d(odom_msg->pose.pose.position.x,
								  odom_msg->pose.pose.position.y,
								  odom_msg->pose.pose.position.z);

	qic = Quaterniond(odom_msg->pose.pose.orientation.w,
										   odom_msg->pose.pose.orientation.x,
										   odom_msg->pose.pose.orientation.y,
										   odom_msg->pose.pose.orientation.z).toRotationMatrix();

	qic=qic*r_drift_odom;
    Quaterniond qua(qic);
	Vector3d vio_t_cam;
	Quaterniond vio_q_cam;
	vio_t_cam = tic;
	vio_q_cam = qua;         
	cameraposevisual.reset();
	cameraposevisual.setScale(marker_scale);
	cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
	cameraposevisual.publish_by(pub_odom_pose_visual, odom_msg->header);
}

void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
	pose_path_msg.header=pose_msg->header;
	geometry_msgs::PoseStamped pose;
	pose.header=pose_msg->header;
	pose.pose=pose_msg->pose;
	pose_path_msg.poses.push_back(pose);
    pose_pub.publish(pose_path_msg);

	//marker visual
	Vector3d tic;
    Matrix3d qic;
	tic = Vector3d(pose_msg->pose.position.x,
								  pose_msg->pose.position.y,
								  pose_msg->pose.position.z);

	qic = Quaterniond(pose_msg->pose.orientation.w,
										   pose_msg->pose.orientation.x,
										   pose_msg->pose.orientation.y,
										   pose_msg->pose.orientation.z).toRotationMatrix();

	qic=qic*r_drift_bluerov;
    Quaterniond qua(qic);
	Vector3d vio_t_cam;
	Quaterniond vio_q_cam;
	vio_t_cam = tic;
	vio_q_cam = qua;         
	cameraposevisual.reset();
	cameraposevisual.setScale(marker_scale);
	cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
	cameraposevisual.publish_by(pub_bluerov_pose_visual, pose_msg->header);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "view_node");
  	ros::NodeHandle n("~");  
	n.param("marker_scale",marker_scale,double(1.0));
    ROS_INFO_STREAM("marker_scale:"<<marker_scale);

	AngleAxisd rv1_odom (M_PI/2, Eigen::Vector3d(0, 1, 0)); 
	AngleAxisd rv2_odom (M_PI/2, Eigen::Vector3d(0,0, 1)); 
	Matrix3d rm1_odom = Matrix3d::Identity();
	Matrix3d rm2_odom = Matrix3d::Identity();
	rm1_odom = rv1_odom.matrix();
	rm2_odom = rv2_odom.matrix();
	r_drift_odom=rm1_odom*rm2_odom;

	AngleAxisd rv1_bluerov (M_PI/2, Eigen::Vector3d(0, 1, 0)); 
	AngleAxisd rv2_bluerov (M_PI/2, Eigen::Vector3d(0,0, -1)); 
	Matrix3d rm1_bluerov = Matrix3d::Identity();
	Matrix3d rm2_bluerov = Matrix3d::Identity();
	rm1_bluerov = rv1_bluerov.matrix();
	rm2_bluerov = rv2_bluerov.matrix();
	r_drift_bluerov=rm1_bluerov*rm2_bluerov;

	odom_sub = n.subscribe("/odometry/filtered",5000, &Odom_Callback);
	pose_sub = n.subscribe("/vrpn_client_node/bluerov/pose",5000, &Pose_Callback);

  	odom_pub = n.advertise<nav_msgs::Path>("/odometry/filtered_path", 100);
	pose_pub = n.advertise<nav_msgs::Path>("/vrpn_client_node/bluerov/pose_path", 100);
    pub_odom_pose_visual = n.advertise<visualization_msgs::MarkerArray>("odom_pose_visual", 1000);
    pub_bluerov_pose_visual = n.advertise<visualization_msgs::MarkerArray>("bluerov_pose_visual", 1000);
  	ros::spin();
  	return 0;

}