/*
Documentation:
version 0.1 - point cloud preprocessing + wall/corridor direction following
						- yet to add people avoidance / walking people following
version 0.2 - people avoidance
						- added package (dependencies):
								- ros-indigo-easy-markers
								- ros-indigo-kalman-filter
								- ros-indigo-people
								- ros-indigo-navigation-layers
version 0.3 - added our own cache of point cloud
								- refreshes itself with a radius
								- clears itself after turning at a corner
						- fixed some incorrect geometry calculation at corners
						- smoothed out calculation between two walls
version 0.4 - added stopping at the end of corridor (3 walls)
								- sending goals to nav stack instead
*/
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cmath>
#define _USE_MATH_DEFINES
ros::Publisher pub_downsample;
//ros::Publisher pub_no_ground;
ros::Publisher pub_plane1;
ros::Publisher pub_plane2;
ros::Publisher pub_plane3;
ros::Publisher pub_midstrip_left;
ros::Publisher pub_midstrip_right;
ros::Publisher pub_frame;
ros::Publisher pub_new_goal;
ros::Publisher pub_cloud_buffer;
ros::Publisher pub_user_control;
ros::Publisher pub_cmd_vel;
using namespace std;
// PCL specific libraries
#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl/pcl_base.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
// filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
// additional filters for detecting the frame
#include <pcl/common/common.h> // has: getMinMax3D
#include <pcl/kdtree/kdtree.h> // for: EuclideanClusterExtraction
#include <pcl/segmentation/extract_clusters.h> // has: EuclideanClusterExtraction
// for all sorts of geometries
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
//#include <pcl/Vector.h>

// for rotating the chair
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* driver;

static const float DEGREE_TOLERANCE = 20;
bool AT_CORNER = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBuffer_(new pcl::PointCloud<pcl::PointXYZ>);
tf::TransformListener* tf_listener;


float corridorRadius_ = 3.00;
int goal_count = 1;
ros::Time lastSentGoal_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane3 (new pcl::PointCloud<pcl::PointXYZ>);


bool CONTROLLED_ = true;

Eigen::Vector4f i_hat(1,0,0,0); // unit vector in x direction
// i_hat.x() = 1.0;
// i_hat.y() = 0.0;
// i_hat.z() = 0.0;
// i_hat.w() = 0.0;

Eigen::Vector4f j_hat(0,1,0,0); // unit vector in y direction
// j_hat.x() = 0.0;
// j_hat.y() = 1.0;
// j_hat.z() = 0.0;
// j_hat.w() = 0.0;

Eigen::Vector4f k_hat(0,0,1,0); // unit vector in z direction
// k_hat.x() = 0.0;
// k_hat.y() = 0.0;
// k_hat.z() = 1.0;
// k_hat.w() = 0.0;

void
assis_control(std_msgs::Bool assis_activate)
{
	CONTROLLED_ = assis_activate.data;
	return;
}



void
publish_all_clouds()
{
	// std::cout << "Publishing all clouds to visualize." << std::endl;

	sensor_msgs::PointCloud2 plane1;
	pcl::PCLPointCloud2 cloud_plane1_ros1;
	pcl::toPCLPointCloud2(*cloud_plane1, cloud_plane1_ros1);
	pcl_conversions::fromPCL(cloud_plane1_ros1, plane1);
	pub_plane1.publish (plane1);

	sensor_msgs::PointCloud2 plane2;
	pcl::PCLPointCloud2 cloud_plane2_ros;
	pcl::toPCLPointCloud2(*cloud_plane2, cloud_plane2_ros);
	pcl_conversions::fromPCL(cloud_plane2_ros, plane2);
	pub_plane2.publish (plane2);

	sensor_msgs::PointCloud2 plane3;
	pcl::PCLPointCloud2 cloud_plane3_ros;
	pcl::toPCLPointCloud2(*cloud_plane3, cloud_plane3_ros);
	pcl_conversions::fromPCL(cloud_plane3_ros, plane3);
	pub_plane3.publish (plane3);

	sensor_msgs::PointCloud2 cloud_buffer;
	pcl::PCLPointCloud2 cloud_buffer_ros;
	pcl::toPCLPointCloud2(*cloudBuffer_, cloud_buffer_ros);
	pcl_conversions::fromPCL(cloud_buffer_ros, cloud_buffer);
	pub_cloud_buffer.publish(cloud_buffer);

	std::cout << "# points in plane 1 = " << cloud_plane1->points.size() << std::endl;
	std::cout << "# points in plane 2 = " << cloud_plane2->points.size() << std::endl;

	std::cout << "...Published all clouds." << std::endl;

	return;
}

void
stop_and_give_control_back_to_user()
{
	publish_all_clouds();
	driver->cancelGoal();

	geometry_msgs::Twist stop_cmd_vel;
	stop_cmd_vel.linear.x = stop_cmd_vel.linear.y = stop_cmd_vel.linear.z = 0;
	stop_cmd_vel.angular.x = stop_cmd_vel.angular.y = stop_cmd_vel.angular.z = 0;
	pub_cmd_vel.publish(stop_cmd_vel);

	CONTROLLED_ = false;

	std_msgs::Bool control_to_user;
	control_to_user.data = true;
	pub_user_control.publish(control_to_user);
	cloudBuffer_->clear();
	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	std::cout << "At the end of a corridor / Empty cloudBuffer_. Handing control back to user." << std::endl;
	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	return;
}


bool
refresh_cloud_buffer(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, float min, float max)
{


	pcl::PCLPointCloud2* cloud_kinect = new pcl::PCLPointCloud2;
	//pcl::PCLPointCloud2ConstPtr cloudKinectPtr(cloud_kinect);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud_kinect);
	pcl::fromPCLPointCloud2(*cloud_kinect, *cloud_in);

	if (cloud_in->points.size() == 0)
	{
		return false;
	}

	std::cout << "Refreshing cloudBuffer..." << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);

	// Down-Sampling x8
	pcl::VoxelGrid<pcl::PointXYZ> downsampling;
	downsampling.setInputCloud (cloud_in);
	downsampling.setLeafSize (0.1f, 0.1f, 0.1f);
	downsampling.filter (*temp);

	// Transform cloud buffer from old pose to current pose
	tf::StampedTransform transform;

	if (cloudBuffer_->points.size() == 0)
	{
		cloudBuffer_->header.frame_id = "base_footprint";
	}
	else
	{
		try
		{
			tf_listener->waitForTransform (cloud_in->header.frame_id, pcl_conversions::fromPCL(cloud_in->header.stamp), cloudBuffer_->header.frame_id, pcl_conversions::fromPCL(cloudBuffer_->header.stamp), "odom", ros::Duration(1.0));
	    tf_listener->lookupTransform (cloud_in->header.frame_id, pcl_conversions::fromPCL(cloud_in->header.stamp), cloudBuffer_->header.frame_id, pcl_conversions::fromPCL(cloudBuffer_->header.stamp), "odom", transform);
	  }
	  catch (tf::LookupException &e)
	  {
	    ROS_ERROR ("%s", e.what ());
	    return false;
	  }
	  catch (tf::ExtrapolationException &e)
	  {
	    ROS_ERROR ("%s", e.what ());
	    return false;
	  }

	 	pcl_ros::transformPointCloud (*cloudBuffer_, *cloudBuffer_, transform);
	}

	// Concatenate clouds
	*cloudBuffer_ += *temp;
	//std::cout << "new cloudBuffer_ header:\n" << cloudBuffer_->header << std::endl;
	cloudBuffer_->header.stamp = cloud_in->header.stamp;

	//Remove points outta radius
	pcl::PassThrough<pcl::PointXYZ> remove_points_outta_range;
	remove_points_outta_range.setInputCloud(cloudBuffer_);
	remove_points_outta_range.setFilterFieldName("x");
	remove_points_outta_range.setFilterLimits(min, max);
	remove_points_outta_range.filter(*temp);
	cloudBuffer_ = temp;

	if (cloudBuffer_->points.size() == 0)
	{
			stop_and_give_control_back_to_user();
	}
	//publish_all_clouds();

	return true;
}



// void
// clear_cloud_buffer_at_corner(std_msgs::Bool goal_reached)
// {
// 	if (AT_CORNER && goal_reached.data)
// 	{
//
// 		//cloudBuffer_->clear();
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_empty_cloud (new pcl::PointCloud<pcl::PointXYZ>);
// 		dummy_empty_cloud->header.frame_id = "base_footprint";
// 		dummy_empty_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
//
// 		refresh_cloud_buffer(dummy_empty_cloud, -corridorRadius_, corridorRadius_);
//
// 		AT_CORNER = false;
// 			cout << "-------------------------------------------------------------------------" << endl;
// 		std::cout << "cloud CLEARED" << std::endl;
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1 (new pcl::PointCloud<pcl::PointXYZ>);
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2 (new pcl::PointCloud<pcl::PointXYZ>);
// 		publish_all_clouds(cloud_plane1, cloud_plane2);
// 	}
//
// 	return;
// }

Eigen::Vector4f
cross4(Eigen::Vector4f A, Eigen::Vector4f B)
{
	Eigen::Vector3f A3;
	Eigen::Vector3f B3;

	float wA = A.w();
	A3.x() = A.x();
	A3.y() = A.y();
	A3.z() = A.z();

	float wB = B.w();
	B3.x() = B.x();
	B3.y() = B.y();
	B3.z() = B.z();

	Eigen::Vector3f AcrossB = A3.cross(B3);

	Eigen::Vector4f AcrossB4;
	AcrossB4.x() = AcrossB.x();
	AcrossB4.y() = AcrossB.y();
	AcrossB4.z() = AcrossB.z();
	AcrossB4.w() = 0.0;

	return AcrossB4;
}

inline float
distance_to_wall(float a, float b, float c, float d)
{
	return d/sqrt(a*a+b*b+c*c);
}


void
wall_following(pcl::ModelCoefficients::Ptr coefficients_plane, float max_distance_forward)
{
	std::cout << "max distance forward = " << max_distance_forward << std::endl;

	if  (max_distance_forward < 0.3)
	{
		stop_and_give_control_back_to_user();
		return;
	}

	if ((ros::Time::now() - lastSentGoal_) < ros::Duration(3.0))
	{
		return;
	}



	std::cout << "Following a single wall..." << std::endl;



	Eigen::Vector4f surface_norm;
	surface_norm.x() = coefficients_plane->values[0];
	surface_norm.y() = coefficients_plane->values[1];
	surface_norm.z() = coefficients_plane->values[2];
	surface_norm.w() = 0.0; //coefficients_plane->values[3];
	//std::cout << "surface norm = " << surface_norm << std::endl;
	float D_to_wall = distance_to_wall(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3]);


	Eigen::Vector4f direction = cross4(surface_norm, k_hat); //surface_norm operator^ k_hat;

	double theta = pcl::getAngle3D( i_hat, direction );

	if (abs(theta) > DEG2RAD(90))//90)
	{
		std::cout << "oh, gotta flip direction since theta = " << theta/M_PI*180 << std::endl;
		direction = cross4(k_hat, surface_norm);
		theta = pcl::getAngle3D( i_hat, direction );

	}
	theta = theta * ( -1 *(direction.y() < 0) + +1*(direction.y() > 0 ));

	//std::cout << "direction = " << direction    << std::endl;
	std::cout << "theta = " << theta/M_PI*180 << std::endl;

	move_base_msgs::MoveBaseGoal wall_ahead;
	//move_base_msgs::MoveBaseGoal wall_ahead_pose;
	//MoveBaseClient driver("move_base", true);
	wall_ahead.target_pose.header.frame_id = "base_footprint";

	/* rotate in place to be along the direction of the wall first */
	wall_ahead.target_pose.header.stamp = pcl_conversions::fromPCL(cloudBuffer_->header.stamp);
	wall_ahead.target_pose.pose.position.x = max_distance_forward/3*2;///2;
	std::cout << "goal x = " << wall_ahead.target_pose.pose.position.x << std::endl;
	std::cout << "distance to wall = " << D_to_wall << std::endl;

/////////////////////////need to fix this!!
	float delta_D = abs(abs(D_to_wall)-1.000)*((-1)*(D_to_wall > 0) + 1 *(D_to_wall < 0));
	float delta_y = 0;
	if (abs(D_to_wall) <= 1.000)
	{
		delta_y = delta_D * cos(theta);
	}
	wall_ahead.target_pose.pose.position.y = wall_ahead.target_pose.pose.position.x*tan(theta) + delta_y;
	std::cout << "delta d = " << delta_D << std::endl;
	std::cout << "delta y = " << delta_y << std::endl;
	std::cout << "goal y = " << wall_ahead.target_pose.pose.position.y << std::endl;

	wall_ahead.target_pose.pose.position.z = 0.0;
	wall_ahead.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (theta);

	driver->sendGoal(wall_ahead);
	goal_count++;
	//geometry_msgs::PoseStamped new_goal;
	lastSentGoal_ = ros::Time::now();
	// driver->waitForResult();

	return;
}

bool
corridor_frame_detection (move_base_msgs::MoveBaseGoal* corridor_ahead) // (geometry_msgs::PoseStamped* corridor_ahead)
{
	cloud_plane1->clear();
	cloud_plane2->clear();
	cloud_plane3->clear();
	if (cloudBuffer_->points.size() == 0)
	{
		return false;
	}
	std::cout << "-------------------------------------------------------------------------" << std::endl;
	std::cout << "Detecting corridor! goal # " << goal_count  << std::endl;


	/************************************************************************************
	RANSAC initialization
	************************************************************************************/
	// Initialize containers
	pcl::ModelCoefficients::Ptr coefficients_plane1 (new pcl::ModelCoefficients());
	pcl::ModelCoefficients::Ptr coefficients_plane2 (new pcl::ModelCoefficients());

	pcl::PointIndices::Ptr inliers_plane1 (new pcl::PointIndices());
	pcl::PointIndices::Ptr inliers_plane2 (new pcl::PointIndices());
	pcl::PointXYZ min_pt1, max_pt1;
	pcl::PointXYZ min_pt2, max_pt2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remained (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remained_temp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients_plane3 (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers_plane3 (new pcl::PointIndices());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
	// Optional
	plane_segmentation.setOptimizeCoefficients (true);
	// Mandatory
	plane_segmentation.setModelType (pcl::SACMODEL_PLANE);
	plane_segmentation.setMethodType (pcl::SAC_RANSAC);
	plane_segmentation.setMaxIterations (1000);
	plane_segmentation.setDistanceThreshold (0.02);

	// Create the extraction object
	pcl::ExtractIndices<pcl::PointXYZ> plane_cloud_extraction;
	//pcl::ExtractIndices<pcl::PointXYZ> remove_wall(true);
	//remove_wall.setInputCloud(cloudBuffer_);

	/************************************************************************************
	RANSAC to detect Wall 1
	************************************************************************************/
	//plane_segmentation.setInputCloud (cloud_without_ground);
	//********************************************************
	plane_segmentation.setInputCloud(cloudBuffer_);
	plane_segmentation.segment (*inliers_plane1, *coefficients_plane1);

	float plane_point_ratio = (float) inliers_plane1->indices.size() / (float)(cloudBuffer_->points.size());

	if (inliers_plane1->indices.size() == 0 || plane_point_ratio <= 0.2)
	{
		std::cerr << "\nCould not estimate a planar model for the given dataset." << std::endl;
		return false;
	}

	std::cout << "plane_point_ratio = " << plane_point_ratio << std::endl;

	plane_cloud_extraction.setInputCloud(cloudBuffer_);
	plane_cloud_extraction.setIndices(inliers_plane1);
	plane_cloud_extraction.setNegative(false);
	plane_cloud_extraction.filter(*cloud_plane1);

	/************************************************************************************
	Remove Wall 1 so RANSAC can detect Wall 2
	Detailed Documentation on the filter can be found at:
	docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
	************************************************************************************/
	//pcl::PointIndices::Ptr points_remained (new pcl::PointIndices ());
	//pcl::IndicesPtr points_remained_indices (new std::vector<int>(points_remained->indices)); // some library inconsistencies lol
	plane_cloud_extraction.setNegative(true);
	//plane_cloud_extraction.filter(points_remained->indices);
	plane_cloud_extraction.filter(*cloud_remained);

	//*points_remained_indices = std::vector<int>(points_remained->indices);

	/************************************************************************************
	If only a single wall is detected, start wall following routine.
	************************************************************************************/

	pcl::getMinMax3D(*cloud_plane1, min_pt1, max_pt1);

	if (plane_point_ratio >= 0.8) // not enough points for plane 2
	{
		// pcl is really annoying in terms of vector/dimension compatibilities...
		Eigen::Vector4f plane1_norm;

		plane1_norm.x() = coefficients_plane1->values[0]/ coefficients_plane1->values[3];
		plane1_norm.y() = coefficients_plane1->values[1]/ coefficients_plane1->values[3];
		plane1_norm.z() = coefficients_plane1->values[2]/ coefficients_plane1->values[3];
		plane1_norm.w() = 0;// coefficients_plane1->values[3];
		double angle1 = pcl::getAngle3D( i_hat, plane1_norm );

		if (angle1 > DEG2RAD(170))
		{
			stop_and_give_control_back_to_user();
			return false;
		}

		wall_following(coefficients_plane1, max_pt1.x);
		return false;
		/* with coefficients, we can figure out direction & distance to wall
		so just keep driving forward along the direction of the wall
		for max_pt1.x * cos(theta) distance */
	}

	/************************************************************************************
	RANSAC to detect Wall 2
	************************************************************************************/
	std::cout << "cloud remained size = " << cloud_remained->points.size() << std::endl;

	plane_segmentation.setInputCloud(cloud_remained);
	//plane_segmentation.setIndices (points_remained_indices);
	plane_segmentation.segment (*inliers_plane2, *coefficients_plane2);

	float plane_point_ratio2 = ((float) inliers_plane1->indices.size() + (float) inliers_plane2->indices.size())/ (float)(cloudBuffer_->points.size());
	std::cout << "plane_point_ratio after 2 = " << plane_point_ratio << std::endl;


	if (inliers_plane2->indices.size() == 0 || (plane_point_ratio2 - plane_point_ratio) < 0.3)
	{
		std::cerr << "\nCoud not estimate a 2nd planar model for the given dataset. Following a single wall..." << std::endl;
		wall_following(coefficients_plane1, max_pt1.x);
		return false;
	}

	Eigen::VectorXf line;

	Eigen::Vector4f plane1_norm;
	plane1_norm.x() = coefficients_plane1->values[0];
	plane1_norm.y() = coefficients_plane1->values[1];
	plane1_norm.z() = coefficients_plane1->values[2];
	plane1_norm.w() = coefficients_plane1->values[3];

	// pcl is really annoying in terms of vector/dimension compatibilities...
	Eigen::Vector4f plane2_norm;
	plane2_norm.x() = coefficients_plane2->values[0];
	plane2_norm.y() = coefficients_plane2->values[1];
	plane2_norm.z() = coefficients_plane2->values[2];
	plane2_norm.w() = coefficients_plane2->values[3];
	bool plane_1_2_parallel = !pcl::planeWithPlaneIntersection(plane1_norm, plane2_norm, line, cos(DEG2RAD(DEGREE_TOLERANCE)));
	float D_to_wall1 = distance_to_wall(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2], coefficients_plane1->values[3]);
	float D_to_wall2 = distance_to_wall(coefficients_plane2->values[0], coefficients_plane2->values[1], coefficients_plane2->values[2], coefficients_plane2->values[3]);
	std::cout << "\nd to wall 1 =" << D_to_wall1 << std::endl;
	std::cout << "\nd to wall 2 =" << D_to_wall2 << std::endl;

	// plane_cloud_extraction.setInputCloud(cloudBuffer_);
	// plane_cloud_extraction.setIndices(inliers_plane2);
	// plane_cloud_extraction.setNegative(false);
	// plane_cloud_extraction.filter(*cloud_plane2);
	//
	// plane_cloud_extraction.setNegative(true); // get outliers
	// plane_cloud_extraction.filter(points_remained->indices);

	//pcl::ExtractIndices<pcl::PointXYZ> remove_wall_indices(true); // Initializing with true will allow us to extract the removed indices

	plane_cloud_extraction.setInputCloud(cloud_remained);
	plane_cloud_extraction.setIndices (inliers_plane2);
	plane_cloud_extraction.setNegative(false);
	plane_cloud_extraction.filter(*cloud_plane2);
	plane_cloud_extraction.setNegative(true);
	// plane_cloud_extraction.filter(points_remained->indices);
	plane_cloud_extraction.filter(*cloud_remained_temp);
	cloud_remained.swap(cloud_remained_temp);

	// *points_remained_indices = std::vector<int>(points_remained->indices);
	// *points_remained_indices = plane_cloud_extraction.getRemovedIndices ();

	std::cout << "cloud remained size = " << cloud_remained->points.size() << std::endl;

	while (
		plane_1_2_parallel &&
		(	abs(D_to_wall1 - D_to_wall2) < 0.2 )
	)
	// if RANSAC incoreectly detect planes by identifying the same wall as two overlapping walls
	{
		std::cout << "*** Putting some points into cloud 1" << std::endl;



		// put this into plane1, and re-search for plane 2

		*cloud_plane1 += *cloud_plane2;


		// RANSAC to detect Wall 2
		// pcl::IndicesPtr points_remained_indices (new std::vector<int>(points_remained->indices)); // some library inconsistencies lol
		plane_segmentation.setInputCloud(cloud_remained);
		//plane_segmentation.setIndices (points_remained_indices);
		plane_segmentation.segment (*inliers_plane2, *coefficients_plane2);

		cloud_plane2->clear();

		plane_cloud_extraction.setInputCloud(cloud_remained);
		plane_cloud_extraction.setIndices (inliers_plane2);
		plane_cloud_extraction.setNegative(false);
		plane_cloud_extraction.filter(*cloud_plane2);
		plane_cloud_extraction.setNegative(true);
		plane_cloud_extraction.filter(*cloud_remained_temp);
		cloud_remained.swap(cloud_remained_temp);
		// plane_cloud_extraction.filter(points_remained->indices);
		// *points_remained_indices = std::vector<int>(points_remained->indices);


		plane2_norm.x() = coefficients_plane2->values[0];
		plane2_norm.y() = coefficients_plane2->values[1];
		plane2_norm.z() = coefficients_plane2->values[2];
		plane2_norm.w() = coefficients_plane2->values[3];

		plane_1_2_parallel = !pcl::planeWithPlaneIntersection(plane1_norm, plane2_norm, line, cos(DEG2RAD(DEGREE_TOLERANCE)));
		D_to_wall1 = distance_to_wall(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2], coefficients_plane1->values[3]);
		D_to_wall2 = distance_to_wall(coefficients_plane2->values[0], coefficients_plane2->values[1], coefficients_plane2->values[2], coefficients_plane2->values[3]);
		std::cout << "\nd to wall 1 = " << D_to_wall1 << std::endl;
		std::cout << "d to wall 2 = " << D_to_wall2 << std::endl;

	}

	publish_all_clouds();

	/************************************************************************************
	Find the frame
	************************************************************************************/

	// find edges

	pcl::getMinMax3D(*cloud_plane1, min_pt1, max_pt1);
	pcl::getMinMax3D(*cloud_plane2, min_pt2, max_pt2);

	// determine which points represent the corridor frame
	pcl::PointXYZ edge1, edge2;

	plane1_norm.x() = coefficients_plane1->values[0]/ coefficients_plane1->values[3];
	plane1_norm.y() = coefficients_plane1->values[1]/ coefficients_plane1->values[3];
	plane1_norm.z() = coefficients_plane1->values[2]/ coefficients_plane1->values[3];
	plane1_norm.w() = 0;// coefficients_plane1->values[3];
	double angle_left = pcl::getAngle3D( i_hat, plane1_norm );

	plane2_norm.x() = coefficients_plane2->values[0]/ coefficients_plane2->values[3];
	plane2_norm.y() = coefficients_plane2->values[1]/ coefficients_plane2->values[3];
	plane2_norm.z() = coefficients_plane2->values[2]/ coefficients_plane2->values[3];
	plane2_norm.w() = 0;//coefficients_plane2->values[3];
	double angle_right = pcl::getAngle3D( i_hat, plane2_norm );

	bool plane1_on_left = false;
	if ((min_pt1.y+max_pt1.y)/2 > (min_pt2.y+max_pt2.y)/2)
	// then plane 1 is on the left, then angle between plane 1 and x should be on right, so switch them
	{
		double temp = angle_left;
		angle_left = angle_right;
		angle_right = temp;
		plane1_on_left = true;
	}

	double angle_1_2 = pcl::getAngle3D( plane1_norm, plane2_norm );
	//std::cout << "angle_1_2? = " << RAD2DEG(angle_1_2) << std::endl;
	bool plane_1_2_perpendicular = (angle_1_2 < DEG2RAD(95)) && (angle_1_2 > DEG2RAD(85));
	double angle_sum = angle_left + angle_right;

	// end of corridor?
	/*
	pcl::ExtractIndices<pcl::PointXYZ> remove_wall_indices(true); // Initializing with true will allow us to extract the removed indices
	plane_cloud_extraction.setInputCloud(cloudBuffer_);
	plane_cloud_extraction.setIndices (inliers_plane1);
	plane_cloud_extraction.filter(*cloud_plane1);
	pcl::PointIndices::Ptr points_remained (new pcl::PointIndices ());
	plane_cloud_extraction.setNegative(true); // get outliers
	plane_cloud_extraction.filter(points_remained->indices);

	pcl::IndicesPtr points_remained_indices (new std::vector<int>(points_remained->indices)); // some library inconsistencies lol
	plane_segmentation.setIndices (points_remained_indices);
	plane_segmentation.segment (*inliers_plane2, *coefficients_plane2);

	plane_cloud_extraction.setInputCloud(cloudBuffer_);
	plane_cloud_extraction.setIndices (inliers_plane2);
	plane_cloud_extraction.filter(*cloud_plane2);

	if (inliers_plane2->indices.size() == 0)
	{
		std::cerr << "\nCoud not estimate a 2nd planar model for the given dataset. Following a single wall..." << std::endl;

		plane_cloud_extraction.setInputCloud(cloudBuffer_);
		plane_cloud_extraction.setIndices(inliers_plane1);
		plane_cloud_extraction.setNegative(false);
		plane_cloud_extraction.filter(*cloud_plane1);
	*/

	if (plane_point_ratio2 < 0.8)
	/************************************************************************************
	RANSAC to detect Wall 3
	************************************************************************************/
	{
		std::cout << "dectecting wall 3!" << std::endl;

		// pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(points_remained));
		// pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
		// ransac.setDistanceThreshold(0.01);
		// ransac.computeModel();
		// ransac.getInliers(inliers_plane3);

		// plane_segmentation.setIndices (points_remained_indices);
		plane_segmentation.setInputCloud(cloud_remained);
		plane_segmentation.segment (*inliers_plane3, *coefficients_plane3);
		float D_to_wall3 = distance_to_wall(coefficients_plane3->values[0], coefficients_plane3->values[1], coefficients_plane3->values[2], coefficients_plane3->values[3]);
		std::cout << "\nd to wall 3 =" << D_to_wall3 << std::endl;

		plane_cloud_extraction.setInputCloud(cloud_remained);
		plane_cloud_extraction.setIndices (inliers_plane3);
		plane_cloud_extraction.setNegative(false);
		plane_cloud_extraction.filter(*cloud_plane3);
		plane_cloud_extraction.setNegative(true);
		plane_cloud_extraction.filter(*cloud_remained_temp);
		cloud_remained.swap(cloud_remained_temp);
		// plane_cloud_extraction.filter(points_remained->indices);
		// *points_remained_indices = std::vector<int>(points_remained->indices);
		std::cout << "# points in plane 3 = " << inliers_plane3->indices.size() << std::endl;

		Eigen::Vector4f plane3_norm;
		plane3_norm.x() = coefficients_plane3->values[0]/ coefficients_plane3->values[3];
		plane3_norm.y() = coefficients_plane3->values[1]/ coefficients_plane3->values[3];
		plane3_norm.z() = coefficients_plane3->values[2]/ coefficients_plane3->values[3];
		plane3_norm.w() = 0;

		double angle_3_1 = pcl::getAngle3D(plane1_norm, plane3_norm);
		bool plane_3_1_perpendicular = (angle_3_1 < DEG2RAD(95)) && (angle_3_1 > DEG2RAD(85));
		bool plane_3_1_parallel = (angle_3_1 < DEG2RAD(180)) && (angle_3_1 > DEG2RAD(170));
		double angle_3_2 = pcl::getAngle3D(plane2_norm, plane3_norm);
		bool plane_3_2_perpendicular = (angle_3_2 < DEG2RAD(95)) && (angle_3_2 > DEG2RAD(85));
		bool plane_3_2_parallel = (angle_3_2 < DEG2RAD(180)) && (angle_3_2 > DEG2RAD(170));
		std::cout << "angle_3_1 = " << RAD2DEG(angle_3_1) << std::endl;
		std::cout << "angle_3_2 = " << RAD2DEG(angle_3_2) << std::endl;
		std::cout << "angle_1_2 = " << RAD2DEG(angle_1_2) << std::endl;

		if (plane_1_2_perpendicular && plane_3_1_perpendicular && plane_3_2_parallel
			|| plane_1_2_perpendicular && plane_3_1_parallel && plane_3_2_perpendicular
			|| plane_1_2_parallel && plane_3_1_perpendicular && plane_3_2_perpendicular)
		{


			stop_and_give_control_back_to_user();

			// corridor_ahead->target_pose.header.frame_id = "base_footprint";
			// corridor_ahead->target_pose.header.stamp = pcl_conversions::fromPCL(cloudBuffer_->header.stamp);
			// corridor_ahead->target_pose.pose.position.x = 0.;
			// corridor_ahead->target_pose.pose.position.y = 0;
			// corridor_ahead->target_pose.pose.position.z = 0;
			// corridor_ahead->target_pose.pose.orientation.x = 0;
			// corridor_ahead->target_pose.pose.orientation.y = 0;
			// corridor_ahead->target_pose.pose.orientation.z = 0;
			// corridor_ahead->target_pose.pose.orientation.w = 1.0;

			return false;
		}
	}




	/////////////////////////////

	float turn_angle_rad;

	/*
	CORNER
	*/

	if (plane_1_2_perpendicular)
	/* at a corner here
	go forward half of the distance between here and the facing corering wall
	and turn */
	{
		std::cerr << "AT A COOOOOOOOOOOOOOOOOOOOOOOORNEEEEEEEEEEEEEEEEEEEEEER" << std::endl;
		float x0 = line[0];
		float y0 = line[1];
		float z0 = line[2];
		float a = line[3];
		float b = line[4];
		float c = line[5];

		float z = 0;
		float x_corner = a*(z - z0)/c + x0;
		float y_corner = b*(z - z0)/c + y0;

		double theta;

		//x = x/3*2;
		//y = - y/3*2;

		//x = x-1;
		//y = -(y + (-1)*(y>1) + (+1)*(y<-1));

		// clear the other wall to not interfere with future detection
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToBe_Kept____AfterCorner;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToBe_Removed_AfterCorner;
		std::cout << "pi - angle_left = " << M_PI-angle_left << std::endl;
		std::cout << "pi - angle_right = " << M_PI-angle_right << std::endl;

		if (plane1_on_left && (angle_left < angle_right))
		{
			std::cout << "plane 1 on left and left < right" << std::endl;
			cloudToBe_Kept____AfterCorner = cloud_plane1;
			cloudToBe_Removed_AfterCorner = cloud_plane2;
			// x = (abs(D_to_wall1) - 1.000)/cos(M_PI-angle_left);
			// y = x*tan(M_PI-angle_left);
			// y = ( (D_to_wall2 > 0)*(-1) + (D_to_wall2 < 0)*(+1) )*y;
			theta = M_PI - angle_right;
		}
		else if (plane1_on_left && (angle_left > angle_right))
		{
			std::cout << "plane 1 on left and left > right" << std::endl;
			cloudToBe_Kept____AfterCorner = cloud_plane2;
			cloudToBe_Removed_AfterCorner = cloud_plane1;
			// x = (abs(D_to_wall1) - 1.000)/cos(M_PI-angle_right);
			// y = x*tan(M_PI-angle_right);
			// y = ( (D_to_wall2 > 0)*(-1) + (D_to_wall2 < 0)*(+1) )*y;
			theta = M_PI - angle_left;
		}
		else if (angle_left < angle_right)
		{
			std::cout << "plane 2 on left and left < right" << std::endl;
			cloudToBe_Kept____AfterCorner = cloud_plane2;
			cloudToBe_Removed_AfterCorner = cloud_plane1;
			// x = (abs(D_to_wall2) - 1.000)/cos(M_PI-angle_left);
			// y = x*tan(M_PI-angle_left);
			// y = ( (D_to_wall1 > 0)*(-1) + (D_to_wall1 < 0)*(+1) )*y;
			theta = M_PI - angle_right;
		}
		else
		{
			std::cout << "plane 2 on left and left > right" << std::endl;
			cloudToBe_Kept____AfterCorner = cloud_plane1;
			cloudToBe_Removed_AfterCorner = cloud_plane2;
			// x = (abs(D_to_wall2) - 1.000)/cos(M_PI-angle_right);
			// y = x*tan(M_PI-angle_right);
			// y = ( (D_to_wall1 > 0)*(-1) + (D_to_wall1 < 0)*(+1) )*y;
			theta = M_PI - angle_left;
		}
		std::cout << "theta = " << theta << std::endl;

		float x_center = (min_pt1.x+max_pt1.x+min_pt2.x+max_pt2.x)/4;
		float y_center = (min_pt1.y+max_pt1.y+min_pt2.y+max_pt2.y)/4;
		std::cout << "x_center = " << x_center << std::endl;
		std::cout << "y_center = " << y_center << std::endl;
		std::cout << "x_corner = " << x_corner << std::endl;
		std::cout << "y_corner = " << y_corner << std::endl;


		float slope = (x_center - x_corner)/(y_center -  y_corner);
		// float intercept = y_corner - slope * x_corner;
		std::cout << "slope = " << slope << std::endl;

		corridor_ahead->target_pose.header.frame_id = "base_footprint";
		corridor_ahead->target_pose.header.stamp = pcl_conversions::fromPCL(cloudBuffer_->header.stamp);

		float D_to_new_wall = (abs(D_to_wall1) > abs(D_to_wall2))*abs(D_to_wall2) + (abs(D_to_wall1) < abs(D_to_wall2))*abs(D_to_wall1);
		corridor_ahead->target_pose.pose.position.x = y_corner * slope + x_corner;
		if (x_corner - corridor_ahead->target_pose.pose.position.x < 1.000)
		{
			std::cout << "new corner goal x = \n" << std::endl;
			std::cout << corridor_ahead->target_pose.pose.position.x << std::endl;
			std::cout << "too close to wall -- have to move backwards" << std::endl;
			corridor_ahead->target_pose.pose.position.x = x_corner - 1.000; // - y_corner/slope + x_corner; //- D_to_new_wall;///cos(theta);
			// corridor_ahead->target_pose.pose.position.x = corridor_ahead->target_pose.pose.position.x -
		}
		//if (x_corner - corridor_ahead->target_pose.pose.position.x < 3.0)
		//std::cout << "x corner - D = " << x_corner - D_to_new_wall << std::endl;
		//std::cout << "x corner - D/cos = " << x_corner - D_to_new_wall/cos(theta) << std::endl;

		corridor_ahead->target_pose.pose.position.y = y_corner + (y_center - y_corner)*3;; //0;//y_corner + D_to_new_wall*((y_corner < 0)*1 + (y_corner > 0)*(-1));///cos(theta);

		corridor_ahead->target_pose.pose.position.z = 0;
		std::cout << "new corner goal = \n" << std::endl;
		std::cout << corridor_ahead->target_pose.pose.position << std::endl;
		turn_angle_rad = (angle_1_2 - theta)*1*((y_corner < 0)*1 + (y_corner > 0)*(-1));
		std::cout << "turn angle = " << RAD2DEG(turn_angle_rad) << std::endl;
		// 0.8? - since ROS isnt very accurate about orientation, rotate by a small angle to not lose the wall
		//(y > 0)*angle_left/2 - (y < 0)*angle_right/2;
		AT_CORNER = true;

		corridor_ahead->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (turn_angle_rad);

		driver->sendGoal(*corridor_ahead);
		driver->waitForResult();

		if (driver->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Corner goal finished. CLOUD CLEARED");
			AT_CORNER = false;
			cloudBuffer_->clear();
			*cloudBuffer_ += *cloudToBe_Kept____AfterCorner;
			cloudToBe_Removed_AfterCorner->clear();
			publish_all_clouds();
		//lastSentGoal_ = lastSentGoal_ + ros::Duration(3.0);
				// delay this a bit more so move_base finish reaching the goal before new cloud readings
		}
		else
			std::cerr << "The base failed to turn at a corner for some reason" << std::endl;

		return false;

	}

	/*
	CORRIDOR WITH TWO WALLS TO FOLLOW
	*/

	else
	{

		//std::cout << "\nangle_left = " << RAD2DEG(angle_left) << std::endl;
		//std::cout << "angle_right = " << RAD2DEG(angle_right) << std::endl;
		std::cout << "angle_sum = " << RAD2DEG(angle_sum) << std::endl;

		edge1.x = max_pt1.x;
		edge2.x = max_pt2.x;
		turn_angle_rad = (angle_left - angle_right) / 2;
		// if ( plane_1_2_parallel ) // two parallel walls
		// {
		// 	//std::cout << "coefficients plane1:\n" << *coefficients_plane1 << std::endl;
		// 	//std::cout << "coefficients_plane2:\n" << *coefficients_plane2 << std::endl;
		// 	std::cout << "two planes perfectly parallel" << std::endl;
		//
		// 	// if (angle_left <= 90 && angle_right >= 90)
		// 	// {
		// 	//
		// 	//
		// 	// 	edge1.y = min_pt1.y;
		// 	// 	edge2.y = min_pt2.y;
		// 	// }
		// 	// else
		// 	// {
		// 	// 	edge1.y = max_pt1.y;
		// 	// 	edge2.y = max_pt2.y;
		// 	// }
		// 	edge1.y = (min_pt1.y + max_pt1.y)/2;
		// 	edge2.y = (min_pt2.y + max_pt2.y)/2;
		// }
		// else // narrow down / wider up walls
		// {
		// 	std::cout << "two planes diverge" << std::endl;
		// 	if (angle_sum > 180) // narrow down
		// 	{
		// 		edge1.y = plane1_on_left * min_pt1.y + (!plane1_on_left)*min_pt2.y;
		// 		edge2.y = plane1_on_left * max_pt2.y + (!plane1_on_left)*max_pt1.y;
		// 	}
		// 	else // wider up
		// 	{
		// 		edge1.y = plane1_on_left * max_pt1.y + (!plane1_on_left)*max_pt2.y;
		// 		edge2.y = plane1_on_left * min_pt2.y + (!plane1_on_left)*min_pt1.y;
		// 	}
		// }
		corridor_ahead->target_pose.header.frame_id = "base_footprint";
		corridor_ahead->target_pose.header.stamp = pcl_conversions::fromPCL(cloudBuffer_->header.stamp);
	  corridor_ahead->target_pose.pose.position.x = ((edge1.x >=0)*edge1.x + (edge2.x >=0)*edge2.x)/ ((edge1.x >= 0) + (edge2.x >= 0));
		if (corridor_ahead->target_pose.pose.position.x <= 0.05)
		// typically happens when the Kinect loses the wall in its FOV :(
		{
			std::cout << "EMPTY CLOUD IN FOV (but not empty cloudBuffer_) ! so switched back to single wall following" << std::endl;
			// corridor_ahead->target_pose.pose.position.x = 0.5;
			if (edge1.x >= -0.05)
				wall_following(coefficients_plane1, 0.5);
			else
				wall_following(coefficients_plane2, 0.5);
			return false;

		}
		//corridor_ahead->target_pose.pose.position.y = (edge1.y + edge2.y)/2;
		corridor_ahead->target_pose.pose.position.z = 0;

		// y = (-d-ax)/b if z = 0
		float y1 = (-coefficients_plane1->values[3] - coefficients_plane1->values[0]*corridor_ahead->target_pose.pose.position.x)/coefficients_plane1->values[1];
		float y2 = (-coefficients_plane2->values[3] - coefficients_plane2->values[0]*corridor_ahead->target_pose.pose.position.x)/coefficients_plane2->values[1];

		corridor_ahead->target_pose.pose.position.y = (y1+y2)/2;//corridor_ahead->target_pose.pose.position.x*tan(turn_angle_rad);// (edge1.y + edge2.y)/2
		std::cout << "y1 = " << y1 << std::endl;
		std::cout << "y2 = " << y2 << std::endl;
	}
	//std::cout << "NEW GOAL = " << std::endl;
	//std::cout << corridor_ahead->target_pose.pose.position << std::endl;
	std::cout << "\nturn_angle = " << RAD2DEG(turn_angle_rad) << " degrees" << std::endl;
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw (turn_angle_rad);
	corridor_ahead->target_pose.pose.orientation.x = odom_quaternion.x;
	corridor_ahead->target_pose.pose.orientation.y = odom_quaternion.y;
	corridor_ahead->target_pose.pose.orientation.z = odom_quaternion.z;
	corridor_ahead->target_pose.pose.orientation.w = odom_quaternion.w;

	if (RAD2DEG(turn_angle_rad) > 160)
	{
		std::cout << "yoooooo avoided a UUUUUUUUUUUUUUUUUUUUUUUUU turn" << std::endl;
		return false;
	}

	return true;
}

void
corridor_following( const sensor_msgs::PointCloud2ConstPtr& cloud_msg )
{
	// geometry_msgs::PoseStamped corridor_ahead;
	// corridor_ahead.header.frame_id = "base_footprint";
	move_base_msgs::MoveBaseGoal corridor_ahead;


	if (refresh_cloud_buffer(cloud_msg, -corridorRadius_, corridorRadius_))
	{
		//std::cout << "Since last sent goal: " << (lastSentGoal_ - ros::Time::now()) << std::endl;
		if ( !AT_CORNER)
		{
			if (corridor_frame_detection(&corridor_ahead) && ((ros::Time::now() - lastSentGoal_) > ros::Duration(3.0)))
			{
				std::cout << "Following two walls..." << std::endl;
				goal_count++;
				//geometry_msgs::PoseStamped new_goal;
				lastSentGoal_ = ros::Time::now();
				//pub_new_goal.publish(corridor_ahead);
				driver->sendGoal(corridor_ahead);
			}
		}
	}

	return;
}


int
main (int argc, char** argv)
{
	cout << "Press ENTER key to detect corridor frame." << endl;

	// Initialize ROS
	ros::init (argc, argv, "corridor_following");
	tf_listener = new tf::TransformListener(ros::Duration(30));
	driver = new MoveBaseClient("move_base", true);

	if (cin.get() == '\n')
	{


//		ros::Time now = ros::Time::now();
//		tf_listener->waitForTransform ("base_footprint", "base_footprint", ros::Time(0), ros::Duration(1.0));


		ros::NodeHandle input_node_handle;
		// Create a ROS subscriber for the input point cloud and odometry
		ros::Subscriber sub_in = input_node_handle.subscribe ("input", 1, corridor_following);
		// ros::Subscriber sub_user = input_node_handle.subscribe ("hall_trans", 1, assis_control);


		// Create a ROS publisher for the output point cloud
		pub_downsample = input_node_handle.advertise<sensor_msgs::PointCloud2> ("downsampled", 1);
		pub_plane1 = input_node_handle.advertise<sensor_msgs::PointCloud2> ("plane1", 1);
		pub_plane2 = input_node_handle.advertise<sensor_msgs::PointCloud2> ("plane2", 1);
		pub_plane3 = input_node_handle.advertise<sensor_msgs::PointCloud2> ("plane3", 1);
		pub_frame = input_node_handle.advertise<sensor_msgs::PointCloud2> ("frame", 1);


		// Create a ROS subscriber for the rtapmap goal
		ros::NodeHandle goal_node_handle;
		//ros::Subscriber sub_out = goal_node_handle.subscribe ("rtapmap/goal", 1, corridor_following);
		//ros::Subscriber sub_goal_checker = goal_node_handle.subscribe ("rtabmap/goal_reached", 1, clear_corner);
		// ros::Subscriber sub_goal_checker = goal_node_handle.subscribe ("rtabmap/goal_reached", 1, clear_corner);
		pub_cmd_vel = goal_node_handle.advertise<geometry_msgs::Twist> ("nav/cmd_vel", 1);


		// Create a ROS publisher for the goals in assistive mode
		// pub_new_goal = goal_node_handle.advertise<geometry_msgs::PoseStamped> ("rtabmap/goal", 1);
		pub_cloud_buffer = goal_node_handle.advertise<sensor_msgs::PointCloud2> ("cloud_buffer", 1);
		pub_user_control = goal_node_handle.advertise<std_msgs::Bool> ("user_control", 1);

		lastSentGoal_ = ros::Time::now();
		cloud_plane1->header.frame_id = "base_footprint";
		cloud_plane2->header.frame_id = "base_footprint";
		cloud_plane3->header.frame_id = "base_footprint";

		// Spin
		//ros::spin();
		ros::Rate corridor_dis(3); // 1 hz
		while (ros::ok() && CONTROLLED_)
		{
		 ros::spinOnce(); // Callback will be processed
		 corridor_dis.sleep();
		}
	}

	delete driver;
	delete tf_listener;

	return 0;
}
