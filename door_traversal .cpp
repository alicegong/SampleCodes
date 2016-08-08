/*
Documentation:
version 1.0 - detects door on a single plane
version 1.1 - wrapped single plane door detection into a function: find_door_given_clusters_on_same_plane(...)
version 2.0 - added case 2! - door between 2 different planes
version 2.1 - adaptive rotation degree!
version 2.2 - examine door (simulation could only give gaps)
*/

#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cmath>
ros::Publisher pub_downsample;
//ros::Publisher pub_no_ground;
ros::Publisher pub_plane1;
ros::Publisher pub_plane2;
ros::Publisher pub_midstrip;
ros::Publisher pub_midstrip2;
ros::Publisher pub_door;
using namespace std;
// PCL specific libraries

#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>

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
// additional filters for detecting the door
#include <pcl/common/common.h> // has: getMinMax3D
#include <pcl/kdtree/kdtree.h> // for: EuclideanClusterExtraction
#include <pcl/segmentation/extract_clusters.h> // has: EuclideanClusterExtraction
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
// for rotating the chair
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static const float MIN_DOOR_WIDTH = 0.81;
static const float MAX_DOOR_WIDTH = 1.22;
static const float DEGREE_TOLERANCE = 20;

inline float
distance_to_wall(float a, float b, float c, float d)
{
	return d/sqrt(a*a+b*b+c*c);
}

inline float
calculate_gap_width(pcl::PointXYZ left_edge, pcl::PointXYZ right_edge)
{
	// pow(base, power)
	// sqrt(num)
	return  sqrt (pow ( (right_edge.x - left_edge.x), 2) + pow ( (right_edge.y - left_edge.y), 2) );
}

void rotate_chair(float dir)
{
	cerr << "*** Case 2 failed: too wide between 2 parallel walls. Will rotate the vehicle in direction " << dir << " for " << dir*30 << "degrees and restart search..." << endl;
	MoveBaseClient driver("move_base", true);
	//wait for the action server to come up
  while(!driver.waitForServer(ros::Duration(5.0)))
	{
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.0;
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw (DEG2RAD(dir*30));
	goal.target_pose.pose.orientation.x = odom_quaternion.x;
	goal.target_pose.pose.orientation.y = odom_quaternion.y;
	goal.target_pose.pose.orientation.z = odom_quaternion.z;
  goal.target_pose.pose.orientation.w = odom_quaternion.w;

  ROS_INFO("Rotating chair...");
  driver.sendGoal(goal);

  driver.waitForResult();

  if (driver.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Rotated");
  else
    ROS_INFO("Failed to rotate for some reason");

  return;
}


bool
find_door_given_clusters_on_same_plane ( std::vector<pcl::PointIndices>* cluster_indices,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_midstrip,
	pcl::ModelCoefficients::Ptr coefficients_plane,
	pcl::PointXYZ* left_edge, pcl::PointXYZ* right_edge )
	{
		int j = 1;
		pcl::PointXYZ min_pt, max_pt;
		pcl::PointXYZ test_point_left, test_point_right;
		test_point_left.x = 0; test_point_left.y = -10; test_point_left.z = 0;
		test_point_right.x = 0; test_point_right.y = +10; test_point_right.z = 0;

		// figure out which direction the vehicle is facing
		bool tilted_left = pcl::pointToPlaneDistanceSigned(test_point_left, coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3])
		> pcl::pointToPlaneDistanceSigned(test_point_right, coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3]);


		while ( j <= 3)
		{
			for (std::vector<pcl::PointIndices>::const_iterator iterator = cluster_indices->begin(); iterator != cluster_indices->end(); ++iterator)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator point_iterator = iterator->indices.begin(); point_iterator != iterator->indices.end(); ++point_iterator)
				{
					cloud_cluster->points.push_back(cloud_plane_midstrip->points[*point_iterator]);
					// cloud_door->points.push_back(cloud_plane_midstrip->points[*point_iterator]);
				}
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				// find edges of each cluster
				pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);


				if ( j == 1 )
				{
					*left_edge = min_pt; // if cluster 1 is on the left side, this stays
					*right_edge = max_pt; // if cluster 1 is on the right side, this stays
				}
				else if ( j == 2)
				{
					if ( ! tilted_left)
					{
						if ( min_pt.y > left_edge->y || max_pt.y  > right_edge->y )
						// cluster 2 is on the left side
						{
							right_edge->x = left_edge->x;
							left_edge->x = max_pt.x;
							left_edge->y = min_pt.y;

						}
						else
						// cluster 2 is on the right side
						{
							left_edge->x = right_edge->x;
							right_edge->x = min_pt.x;
							right_edge->y = max_pt.y;
						}
					}
					else
					{
						if ( min_pt.y > left_edge->y || max_pt.y  > right_edge->y )
						// cluster 2 is on the left side
						{
							*left_edge = min_pt;
						}
						else
						// cluster 2 is on the right side
						{
							*right_edge = max_pt;
						}
					}
				}
				else // if (j == 3), in case there's an actual door such that wall is partially behind the door
				{
					if ( min_pt.y < left_edge->y )
					{
						left_edge->y = min_pt.y;
						left_edge->x = max_pt.x;
					}
					else if ( max_pt.y > right_edge->y )
					{
						*right_edge = max_pt;
					}
				}
				j++;
			}
		}



		float gap_width = calculate_gap_width(*left_edge, *right_edge);
		cout << "\n gap width = "  << gap_width << endl;
		if (gap_width  >= MIN_DOOR_WIDTH && gap_width <= MAX_DOOR_WIDTH )
		{
			return true;
		}

		return false;
	}

	bool
	find_door_given_clusters_on_diff_plane ( /*std::vector<pcl::PointIndices>* cluster_indices,
		std::vector<pcl::PointIndices>* cluster_indices2,*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1_midstrip,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2_midstrip,
		pcl::ModelCoefficients::Ptr coefficients_plane1,
		pcl::ModelCoefficients::Ptr coefficients_plane2,
		pcl::PointXYZ* left_edge, pcl::PointXYZ* right_edge )
{
	/*

	std::vector<pcl::PointIndices>::const_iterator iterator;

	iterator = cluster_indices->begin();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<int>::const_iterator point_iterator = iterator->indices.begin(); point_iterator != iterator->indices.end(); ++point_iterator)
	{
		cloud_cluster->points.push_back(cloud_plane_midstrip->points[*point_iterator]);
	}
	cloud_cluster->width = cloud_cluster->points.size();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	iterator = cluster_indices2->begin();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<int>::const_iterator point_iterator = iterator->indices.begin(); point_iterator != iterator->indices.end(); ++point_iterator)
	{
		cloud_cluster2->points.push_back(cloud_plane2_midstrip->points[*point_iterator]);
	}
	cloud_cluster2->width = cloud_cluster2->points.size();
	cloud_cluster2->height = 1;
	cloud_cluster2->is_dense = true;

	pcl::PointXYZ min_pt, max_pt;
	pcl::PointXYZ min_pt2, max_pt2;


	pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
	pcl::getMinMax3D(*cloud_cluster2, min_pt2, max_pt2);
	// std::cout << "edge points are: " << std::endl;
	// std::cout << "min1: " << min_pt << std::endl;
	// std::cout << "max1: " << max_pt << std::endl;
	// std::cout << "min2: " << min_pt2 << std::endl;
	// std::cout << "max2: " << max_pt2 << std::endl;
	*/
	pcl::getMinMax3D(*cloud_plane1_midstrip, min_pt, max_pt);
	pcl::getMinMax3D(*cloud_plane2_midstrip, min_pt2, max_pt2);



	Eigen::Vector4f plane1;
	plane1.x() = coefficients_plane1->values[0];
	plane1.y() = coefficients_plane1->values[1];
	plane1.z() = coefficients_plane1->values[2];
	plane1.w() = coefficients_plane1->values[3];

	Eigen::Vector4f plane2;
	plane2.x() = coefficients_plane2->values[0];
	plane2.y() = coefficients_plane2->values[1];
	plane2.z() = coefficients_plane2->values[2];
	plane2.w() = coefficients_plane2->values[3];

	float offset = abs(coefficients_plane1->values[3] - coefficients_plane2->values[3]);

	Eigen::VectorXf line;

	bool planes_parallel = !pcl::planeWithPlaneIntersection(plane1, plane2, line, cos(DEGREE_TOLERANCE/180*3.14));

	if ( planes_parallel )
	// https://searchcode.com/codesearch/view/37450483/ Note that the angular tolerance is cos(theta) where theta is tolerated degrees
	// if two planes do not have an intersection, i.e. they are parallel, then same routine as before
	{
		// std::cout << "2 PARALLEL planes..." << std::endl;
		pcl::PointXYZ test_point_left, test_point_right;
		test_point_left.x = 0; test_point_left.y = -10; test_point_left.z = 0;
		test_point_right.x = 0; test_point_right.y = +10; test_point_right.z = 0;

		// figure out which direction the vehicle is facing
		bool tilted_left = pcl::pointToPlaneDistanceSigned(test_point_left, coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2], coefficients_plane1->values[3])
		> pcl::pointToPlaneDistanceSigned(test_point_right, coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2], coefficients_plane1->values[3]);


		*left_edge = min_pt; // if cluster 1 is on the left side, this stays
		*right_edge = max_pt; // if cluster 1 is on the right side, this stays

		if ( ! tilted_left)
		{
			if ( min_pt.y > left_edge->y || max_pt.y  > right_edge->y )
			// cluster 2 is on the left side
			{
				right_edge->x = left_edge->x;
				left_edge->x = max_pt2.x;
				left_edge->y = min_pt2.y;

			}
			else
			// cluster 2 is on the right side
			{
				left_edge->x = right_edge->x;
				right_edge->x = min_pt2.x;
				right_edge->y = max_pt2.y;
			}
		}
		else
		{
			if ( min_pt.y > left_edge->y || max_pt.y  > right_edge->y )
			// cluster 2 is on the left side
			{
				*left_edge = min_pt2;
			}
			else
			// cluster 2 is on the right side
			{
				*right_edge = max_pt2;
			}
		}
	}

	else
	// find the intersection line of two plane
	// then find the two points closest to the intersection line
	{
		// std::cout << "2 INTERSECTING planes..." << std::endl;
		float x0 = line[0];
		float y0 = line[1];
		float z0 = line[2];
		float a = line[3];
		float b = line[4];
		float c = line[5];

		float z = (min_pt.z + max_pt.z + min_pt2.z + max_pt2.z) / 4;
		float x = a*(z - z0)/c + x0;
		float y = b*(z - z0)/c + y0;
		// std::cout << "x = "<<x << std::endl;
		// std::cout << "y = "<<y << std::endl;

		if ( min_pt.y > min_pt2.y)
		// cluster 1 on left
		{
			// std::cout << "plane 1 on left" << std::endl;
			left_edge->y = min_pt.y;
			right_edge->y = max_pt2.y;

			if (abs(min_pt.x - x) > abs(max_pt.x - x))
			{
				left_edge->x = max_pt.x;
				right_edge->x = max_pt2.x;
			}
			else
			{
				left_edge->x = min_pt.x;
				right_edge->x = min_pt2.x;
			}
		}
		else
		{
			// std::cout << "plane 2 on left" << std::endl;
			left_edge->y = min_pt2.y;
			right_edge->y = max_pt.y;

			if (abs(min_pt.x - x) > abs(max_pt.x - x))
			{
				left_edge->x = max_pt2.x;
				right_edge->x = max_pt.x;
			}
			else
			{
				left_edge->x = min_pt2.x;
				right_edge->x = min_pt.x;
			}
		}
		left_edge->z = z;
		right_edge->z = z;
		std::cout << "intersecting planes... z = " << z << std::endl;
	}

	float gap_width = calculate_gap_width(*left_edge, *right_edge);
	// cout << "\n gap width = "  << gap_width << endl;
	if (gap_width  >= MIN_DOOR_WIDTH && gap_width <= MAX_DOOR_WIDTH )
	{
		return true;
	}
	else if (planes_parallel)
	{
		std::cout << "too wide: gap width = " << gap_width << std::endl;
		float dir = 1; // default: turn right
		dir = dir * (abs(left_edge->y) - abs(right_edge->y));
		// turn right when tilted left, i.e., difference > 0
		// turn left = +ve, right = -ve

		rotate_chair(dir);
	}

	return false;
}


bool
door_detection (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 	move_base_msgs::MoveBaseGoal* door_pose)
{


	bool search4nextPlane = true;
	bool door_found = false;

	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud_kinect = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudKinectPtr(cloud_kinect);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud_kinect);
	pcl::fromPCLPointCloud2(*cloud_kinect, *cloud);

	if (cloud->points.size() == 0)
	{
		return false;
	}

	std::cout << "Door detection routine starts." << std::endl;


	/************************************************************************************
	Down-Sampling x8
	************************************************************************************/
	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PointXYZ> downsampling;
	downsampling.setInputCloud (cloud);
	downsampling.setLeafSize (0.1, 0.1, 0.1);
	downsampling.filter (*cloud_downsampled);


	/************************************************************************************
	RANSAC initialization
	************************************************************************************/
	// Initialize
	pcl::ModelCoefficients::Ptr coefficients_plane1 (new pcl::ModelCoefficients());
	pcl::ModelCoefficients::Ptr coefficients_plane2 (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers_plane1 (new pcl::PointIndices());
	pcl::PointIndices::Ptr inliers_plane2 (new pcl::PointIndices());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the extraction object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	/************************************************************************************
	RANSAC to detect Wall 1
	************************************************************************************/
	//seg.setInputCloud (cloud_without_ground);
	//********************************************************
	seg.setInputCloud (cloud_downsampled);
	seg.segment (*inliers_plane1, *coefficients_plane1);
	if (inliers_plane1->indices.size() == 0)
	{
		std::cerr << "\nCoud not estimate a planar model for the given dataset." << std::endl;
	}
	else
	{
		// cout << "plane 1 # of points:" << inliers_plane1->indices.size() << endl;
		// cout << "plane 1 coefficients: " << *coefficients_plane1 << endl;
		float plane_point_ratio = (float) inliers_plane1->indices.size() / (float)(cloud_downsampled->height * cloud_downsampled->width);
		// cout << "plane point ratio: " << plane_point_ratio << endl;

		if (plane_point_ratio >= 0.8)
		{
			search4nextPlane = false;
			// cout << " !! - Not enough points left for plane 2. Skipped" << endl;
		}


	}


	// Extract inliers into another point cloud to visualize
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
	extract.setInputCloud(cloud_downsampled);
	extract.setIndices(inliers_plane1);
	extract.setNegative(false);
	extract.filter(*cloud_plane);

	/************************************************************************************
	Find the door
	************************************************************************************/

	// find left & right most
	pcl::PointXYZ min_pt, max_pt;
	pcl::PointXYZ min_pt2, max_pt2;
	// min_pt2.x = 0;
	// min_pt2.y = 0;
	// min_pt2.z = 0;
	// max_pt2.x = 0;
	// max_pt2.y = 0;
	// max_pt2.z = 0;

	pcl::getMinMax3D(*cloud_plane, min_pt, max_pt);
	float mid_height_lower = ( min_pt.z + max_pt.z )/3;
	float mid_height_upper = mid_height_lower*2;
	float mid_height = ( min_pt.z + max_pt.z )/2;

	// filter out a strip at mid height
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_midstrip (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> midstrip_filter;
	midstrip_filter.setInputCloud(cloud_plane);
	midstrip_filter.setFilterFieldName("z");
	midstrip_filter.setFilterLimits(mid_height_lower, mid_height_upper);
	midstrip_filter.filter(*cloud_plane_midstrip);

	// try to find 2 clusters
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_plane_midstrip);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
	cluster_extraction.setClusterTolerance(0.2); // 20cm
	cluster_extraction.setMinClusterSize(5);
	cluster_extraction.setMaxClusterSize(25000);
	cluster_extraction.setSearchMethod(tree);
	cluster_extraction.setInputCloud(cloud_plane_midstrip);
	cluster_extraction.extract(cluster_indices);

	// cout << "** Find clusters? " << cluster_indices.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_door (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ left_edge, right_edge;


	if ( cluster_indices.size() == 2 )
	{

		if (find_door_given_clusters_on_same_plane(&cluster_indices, cloud_plane_midstrip, coefficients_plane1, &left_edge, &right_edge) ) //&& gap_width <= 1.64 )
		{
			search4nextPlane = false;
			door_found = true;
			cout << "** Case 1A: FIND DOOR!\n  left_edge = " << left_edge << "\n  right_edge = " << right_edge << endl;

		}
	}



	/************************************************************************************
	If no door find in one plane
	search for plane 2
	************************************************************************************/

	if (search4nextPlane)
	{
		cerr << "*** Case 1A failed: Didn't find a wide/narrow enough gap in plane 1. Continue to search..." << endl;
		//cout<<"\nfinished";

		/************************************************************************************
		Remove Wall 1 so RANSAC can detect Wall 2
		Detailed Documentation on the filter can be found at:
		docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
		************************************************************************************/
		///cl::PointCloud<pcl::PointXYZ>::Ptr cloud_remained (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ExtractIndices<pcl::PointXYZ> remove_wall (true); // Initializing with true will allow us to extract the removed indices
		//remove_wall.setInputCloud(cloud_without_ground);
		//********************************************************
		remove_wall.setInputCloud(cloud_downsampled);
		remove_wall.setIndices (inliers_plane1);
		remove_wall.filter(*cloud_plane);
		pcl::PointIndices::Ptr cloud_remained (new pcl::PointIndices ());
		remove_wall.setNegative(true); // get outliers
		remove_wall.filter(cloud_remained->indices);

		float D_to_wall1 = distance_to_wall(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2], coefficients_plane1->values[3]);


		pcl::IndicesPtr cloud_remained_indices (new std::vector<int>(cloud_remained->indices)); // some library inconsistencies lol
		pcl::PassThrough<pcl::PointXYZ> limit_forward;
		// without this, the algorithm might detect the other side of the corridor (outside the door) and the other wall
		limit_forward.setInputCloud(cloud_downsampled);
		limit_forward.setIndices(cloud_remained_indices);
		limit_forward.setFilterFieldName("x");
		limit_forward.setFilterLimits(0.0, D_to_wall1 + 1.0);
		limit_forward.filter(cloud_remained->indices);
		*cloud_remained_indices = cloud_remained->indices;

		/************************************************************************************
		RANSAC to detect Wall 2
		************************************************************************************/

		seg.setIndices (cloud_remained_indices);
		seg.segment (*inliers_plane2, *coefficients_plane2);
		if (inliers_plane2->indices.size() == 0)
		{
			std::cerr << "\nCoud not estimate a 2nd planar model for the given dataset." << std::endl;
		}
		else
		{
			// cout << "\nplane 2 # of points:" << inliers_plane2->indices.size() << endl;
			// cout << "plane 2 coefficients: " << *coefficients_plane2 << endl;

		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2 (new pcl::PointCloud<pcl::PointXYZ>);
		extract.setInputCloud(cloud_downsampled);
		extract.setIndices(inliers_plane2);
		extract.setNegative(false);
		extract.filter(*cloud_plane2);

		//cout << "let's examine the cloud! plane 2 \n" << *cloud_plane2 <<endl;
		//cout << "x = "<< cloud_plane2->points[1].x << " y = " << cloud_plane2->points[1].y << " z = " << cloud_plane2->points[1].z << endl;
		//cout << "x = "<< cloud_plane2->points[2].x << " y = " << cloud_plane2->points[2].y << " z = " << cloud_plane2->points[2].z << endl;
		// below gives the 3 coordinates in 1 bracket
		//cout << "point 2 = " << cloud_plane2->points[2] << endl;



		/************************************************************************************
		Publish planes to ROS
		************************************************************************************/
		// Convert to ROS data type

		sensor_msgs::PointCloud2 plane2;
		pcl::PCLPointCloud2 cloud_plane2_ros;
		pcl::toPCLPointCloud2(*cloud_plane2, cloud_plane2_ros);
		pcl_conversions::fromPCL(cloud_plane2_ros, plane2);
		pub_plane2.publish (plane2);

		/************************************************************************************
		Extract clusters with same routine as before
		************************************************************************************/
		pcl::getMinMax3D(*cloud_plane2, min_pt2, max_pt2);
		float mid_height_lower = ( min_pt2.z + max_pt2.z )/3;
		float mid_height_upper = mid_height_lower*2;
		float mid_height = ( min_pt2.z + max_pt2.z )/2;

		// filter out a strip at mid height
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2_midstrip (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PassThrough<pcl::PointXYZ> midstrip_filter;
		midstrip_filter.setInputCloud(cloud_plane2);
		midstrip_filter.setFilterFieldName("z");
		midstrip_filter.setFilterLimits(mid_height_lower, mid_height_upper);
		midstrip_filter.filter(*cloud_plane2_midstrip);

		sensor_msgs::PointCloud2 midstrip2;
		pcl::PCLPointCloud2 cloud_plane2_midstrip_ros;
		pcl::toPCLPointCloud2(*cloud_plane2_midstrip, cloud_plane2_midstrip_ros);
		pcl_conversions::fromPCL(cloud_plane2_midstrip_ros, midstrip2);
		pub_midstrip2.publish (midstrip2);

		// try to find 2 clusters

		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_plane2_midstrip);
		std::vector<pcl::PointIndices> cluster_indices2;
		//pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
		//cluster_extraction.setClusterTolerance(0.2); // 2cm
		//cluster_extraction.setMinClusterSize(15);
		//cluster_extraction.setMaxClusterSize(25000);
		//cluster_extraction.setSearchMethod(tree);
		cluster_extraction.setInputCloud(cloud_plane2_midstrip);
		cluster_extraction.extract(cluster_indices2);

		// cout << "** Find clusters? " << cluster_indices2.size() << endl;

		/************************************************************************************
		Case 1B: 2nd plane // 1st plane but didnt find a door in 1st plane (was a wall behind the door)
		so use the same routine as 1A
		************************************************************************************/

		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_door (new pcl::PointCloud<pcl::PointXYZ>);


		if ( cluster_indices2.size() == 2 )
		{

			//find_door_given_clusters_on_same_plane(&cluster_indices2, cloud_plane2_midstrip, coefficients_plane2, &left_edge, &right_edge );
			/*find_door_given_clusters_on_same_plane ( std::vector<pcl::PointIndices>::Ptr cluster_indices2,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2_midstrip,
			pcl::ModelCoefficients::Ptr coefficients_plane,
			pcl::PointXYZ* left_edge, pcl::PointXYZ* right_edge )*/

			//float gap_width = calculate_gap_width(left_edge, right_edge);
			if (find_door_given_clusters_on_same_plane(&cluster_indices2, cloud_plane2_midstrip, coefficients_plane2, &left_edge, &right_edge ) ) //&& gap_width <= 1.64 )
			{
				search4nextPlane = false;
				door_found = true;
				cout << "** Case 1B: FIND DOOR!\n  left_edge = " << left_edge << "\n  right_edge = " << right_edge << endl;
				// left_edge.z = mid_height;
				// right_edge.z = mid_height;
			}
		}

		if (search4nextPlane && cluster_indices.size() >= 1 && cluster_indices2.size() >= 1)
		{
			cerr << "*** Case 1B failed: Didn't find a wide/narrow enough gap in plane 2. Continue to search..." << endl;
			/************************************************************************************
			Case 2: door between 2 different planes
			************************************************************************************/
			/*find_door_given_clusters_on_diff_plane(//&cluster_indices, &cluster_indices2,
				cloud_plane_midstrip, cloud_plane2_midstrip,
				coefficients_plane1, coefficients_plane2,
				&left_edge, &right_edge );*/

			//float gap_width = calculate_gap_width(left_edge, right_edge);
			if (find_door_given_clusters_on_diff_plane(//&cluster_indices, &cluster_indices2,
				cloud_plane_midstrip, cloud_plane2_midstrip,
				coefficients_plane1, coefficients_plane2,
				&left_edge, &right_edge )) //&& gap_width <= 1.64 )
			{
				search4nextPlane = false;
				door_found = true;
				cout << "** Case 2: FIND DOOR!\n  left_edge = " << left_edge << "\n  right_edge = " << right_edge << endl;
				// left_edge.z = mid_height;
				// right_edge.z = mid_height;
			}
		}


	}

	if (!door_found && cluster_indices.size() == 1)
	{
		float dir = -1;
		if (max_pt.y - min_pt.y > max_pt2.y - min_pt2.y)
		{
			dir = dir + (abs(min_pt.y) > abs(max_pt.y))*2;
			dir = dir * 1/(max_pt.y - min_pt.y);
		}
		else
		{
			dir = dir + (abs(min_pt2.y) > abs(max_pt2.y))*2;
			dir = dir * 1/(max_pt2.y - min_pt2.y);
		}
		// so change it to 1
		cerr << "*** Case 2 failed: Didn't find any door. Will rotate the vehicle in " << dir << " direction and restart search..." << endl;
		// turn left = +ve, right = -ve
		// turn left when right (min.y) > left (max.y)
		rotate_chair(dir);

	}

	/************************************************************************************
	Publish everything else to ROS
	************************************************************************************/
	sensor_msgs::PointCloud2 plane1;
	pcl::PCLPointCloud2 cloud_plane_ros1;
	pcl::toPCLPointCloud2(*cloud_plane, cloud_plane_ros1);
	pcl_conversions::fromPCL(cloud_plane_ros1, plane1);
	pub_plane1.publish (plane1);

	sensor_msgs::PointCloud2 downsampled;
	pcl::PCLPointCloud2 downsampled_ros;
	pcl::toPCLPointCloud2(*cloud_downsampled, downsampled_ros);
	pcl_conversions::fromPCL(downsampled_ros, downsampled);
	pub_downsample.publish(downsampled);

	sensor_msgs::PointCloud2 midstrip;
	pcl::PCLPointCloud2 cloud_plane_midstrip_ros;
	pcl::toPCLPointCloud2(*cloud_plane_midstrip, cloud_plane_midstrip_ros);
	pcl_conversions::fromPCL(cloud_plane_midstrip_ros, midstrip);
	pub_midstrip.publish (midstrip);

	cout << "-------------------------------------------------------------------------" << endl;


	if (door_found)
	{
		door_pose->target_pose.header.frame_id = "base_footprint";
		door_pose->target_pose.header.stamp = ros::Time::now();
	  door_pose->target_pose.pose.position.x = (left_edge.x + right_edge.x)/2;
		door_pose->target_pose.pose.position.y = (left_edge.y + right_edge.y)/2;
		door_pose->target_pose.pose.position.z = 0;
		float turn_angle_rad = atan(door_pose->target_pose.pose.position.y/door_pose->target_pose.pose.position.x);
		geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw (turn_angle_rad);
		door_pose->target_pose.pose.orientation.x = odom_quaternion.x;
		door_pose->target_pose.pose.orientation.y = odom_quaternion.y;
		door_pose->target_pose.pose.orientation.z = odom_quaternion.z;
	  door_pose->target_pose.pose.orientation.w = odom_quaternion.w;

		pcl::PointXYZ door_coord;
		door_coord.x = door_pose->target_pose.pose.position.x;
		door_coord.y = door_pose->target_pose.pose.position.y;
		door_coord.z = door_pose->target_pose.pose.position.z;

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_door (new pcl::PointCloud<pcl::PointXYZ>);
		cloud_door->points.push_back(left_edge);
		cloud_door->points.push_back(right_edge);
		cloud_door->width = cloud_door->points.size();
		cloud_door->height = 1;
		cloud_door->is_dense = true;

		sensor_msgs::PointCloud2 door;
		pcl::PCLPointCloud2 cloud_door_ros;
		pcl::toPCLPointCloud2(*cloud_door, cloud_door_ros);
		pcl_conversions::fromPCL(cloud_door_ros, door);
		door.header.frame_id = cloud_plane2_midstrip->header.frame_id;
		// door.header.stamp = cloud_plane2_midstrip->header.stamp;
		pub_door.publish(door);

		return true;
	}
	return false;

}


void door_traversal ( const sensor_msgs::PointCloud2ConstPtr& cloud_msg )
{
	move_base_msgs::MoveBaseGoal door_pose;
	MoveBaseClient driver("move_base", true);

	if (door_detection(cloud_msg, &door_pose))
	{

		//wait for the action server to come up
	  while(!driver.waitForServer(ros::Duration(5.0)))
		{
	    ROS_INFO("Waiting for the move_base action server to come up");
	  }

	  ROS_INFO("Sending door_pose goal");
	  driver.sendGoal(door_pose);

	  driver.waitForResult();

	  if (driver.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Hooray, the base is at the door.");
			move_base_msgs::MoveBaseGoal forward_extra;
			forward_extra.target_pose.header.frame_id = "base_footprint";
			forward_extra.target_pose.header.stamp = ros::Time::now();
		  forward_extra.target_pose.pose.position.x = 0.5;
			forward_extra.target_pose.pose.orientation.w = 1.0;

			ROS_INFO("Sending forward_extra goal");
		  driver.sendGoal(forward_extra);

		  driver.waitForResult();
			if (driver.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("YEEEEEEEES the base got through the door!!");
			else
		    ROS_INFO("The base is stuck for some reason AFTER at the door");
		}
	  else
		{
			ROS_INFO("The base is stuck BEFORE the door for some reason");
			ROS_INFO("Sending door_pose goal AGAIN");
			driver.sendGoal(door_pose);
			driver.waitForResult();
			if (driver.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Hooray, the base is at the door.");
				move_base_msgs::MoveBaseGoal forward_extra;
				forward_extra.target_pose.header.frame_id = "base_footprint";
				forward_extra.target_pose.header.stamp = ros::Time::now();
			  forward_extra.target_pose.pose.position.x = 1.0;
				forward_extra.target_pose.pose.orientation.w = 1.0;

				ROS_INFO("Sending forward_extra goal");
			  driver.sendGoal(forward_extra);

			  driver.waitForResult();
				if (driver.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					ROS_INFO("YEEEEEEEES the base got through the door!!");
				else
			    ROS_INFO("The base is stuck for some reason AFTER at the door");
			}
		  else
			{
				ROS_INFO("The base is stuck BEFORE the door for some reason");
			}
		}
	}

	// modify the routine:
	// when the vehicle has reach the goal (center of door),
	// move forward 0.5m more to have a clear view
	return;
}

int
main (int argc, char** argv)
{
	cout << "Press ENTER key to detect door." << endl;

	if (cin.get() == '\n')
	{
		// Initialize ROS
		ros::init (argc, argv, "door_traversal");
		ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
		ros::Subscriber sub = nh.subscribe ("obstacles_cloud", 1, door_traversal);

	// Create a ROS publisher for the output point cloud
		pub_downsample = nh.advertise<sensor_msgs::PointCloud2> ("downsampled", 1);
		pub_plane1 = nh.advertise<sensor_msgs::PointCloud2> ("plane1", 1);
		pub_plane2 = nh.advertise<sensor_msgs::PointCloud2> ("plane2", 1);
		pub_midstrip = nh.advertise<sensor_msgs::PointCloud2> ("midstrip", 1);
		pub_door = nh.advertise<sensor_msgs::PointCloud2> ("door", 1);
		pub_midstrip2 = nh.advertise<sensor_msgs::PointCloud2> ("midstrip2", 1);

		// Spin
		ros::spin();
	}

	return 0;


}
