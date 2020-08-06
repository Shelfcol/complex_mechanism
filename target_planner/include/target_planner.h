#ifndef TARGET_PLANNER_H
#define TARGET_PLANNER_H

#include <algorithm>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <map>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <tf/transform_listener.h>
#include <vector>

using namespace std;

class TargetPlanner
{
public:
	struct position
	{
		float fx;
		float fy;
	};

	struct map_point
	{
		int nx;
		int ny;
	};

	struct state
	{
		float fx;
		float fy;
		float fyaw;
	};

public:
	TargetPlanner(ros::NodeHandle &node_handle, string &frame_id);

private:
	/**
    * @brief 定义车辆运动参数
    */

	float disinfect_radius; //机器人某个位置能消毒液能覆盖的半径
	float disinfect_reso;	// 规划点之间的间距
	float x_min;			// 确定全局删格地图中的矩形消毒区域
	float x_max;
	float y_min;
	float y_max;

	vector<geometry_msgs::PoseStamped> targets;
	/**
     * @brief ROS相关
     */
	ros::NodeHandle nh;
	string msFrame_id;

	ros::Subscriber map_sub;
	ros::Subscriber sub_amcl;
	ros::Publisher pub_target;
	ros::Publisher bound_box_pub;
	ros::Publisher target_point_pub_;
	/**
     * @brief 地图信息相关
     */
	nav_msgs::OccupancyGridConstPtr map_;
	bool get_map;					//是否获得地图
	bool if_calc_targets;			//表示是否计算了targets
	float mfResolution;				//地图分辨率
	int mfWidth;					//地图宽度
	int mfHeight;					// 地图高度
	position mpOrigin;				// 地图原点
	int max_index;					//地图的最大索引值
	geometry_msgs::Point32 car_pos; //车辆此时的位置
	float min_dist2target;			// 车辆距离目标点小于这个距离，则表示到达这个目标点，准备发下一个目标点
	int targets_size;				//保存目标点的数量
	int targets_number_now;			//表示下一个需要发送给小车的点的序号
	float max_waiting_time;
	ros::Time last; //保存开始从当前目标点出发的时间

private:
	//订阅全局代价地图
	void getMap(const nav_msgs::OccupancyGridConstPtr &map);

	//发布规定的消毒区域的位置
	void pubBoundBox();

	//根据接收到的全局膨胀代价地图，计算目标点,整个规划是弓型路径规划
	void calc_targets();
	int check_collision(float x, float y); //检验此点是否在全局代价地图中的危险位置

	//判断车辆是否到达目标点，这里可以有一个计数，如果到达目标点时间间隔超过60s，则自动发送下一个目标点，表示这个目标点被临时占据，无法到达
	void if_arrive_target();

	float calc_dist(float dx, float dy);

	//每收到一次amcl发过来的全局位置，则计算发送的target序号，然后将target发送出去
	void getCarPos(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose);
};

//-------------------------------------------------------------
TargetPlanner::TargetPlanner(ros::NodeHandle &node_handle, string &frame_id)
{
	msFrame_id = frame_id;
	nh = node_handle;

	nh.param("disinfect_radius", disinfect_radius, 1.0f);
	nh.param("disinfect_reso", disinfect_reso, 1.0f);
	nh.param("x_min", x_min, 0.0f);
	nh.param("x_max", x_max, 3.0f);
	nh.param("y_min", y_min, 0.0f);
	nh.param("y_max", y_max, 5.0f);
	nh.param("min_dist2target", min_dist2target, 0.5f);
	nh.param("max_waiting_time", max_waiting_time, 120.0f); //到达下一个点的最大时间，超过这个时间，表示此点被占据，无法到达，则继续到下一个点
	targets_number_now = 1;

	map_sub = nh.subscribe("/move_base/global_costmap/costmap", 10, &TargetPlanner::getMap, this);
	sub_amcl = nh.subscribe("/amcl_pose", 100, &TargetPlanner::getCarPos, this);
	pub_target = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
	bound_box_pub = nh.advertise<geometry_msgs::PolygonStamped>("/bound_box", 1, true);
	last = ros::Time::now();
	get_map = false;
	if_calc_targets = false;
}

//订阅全局代价地图
void TargetPlanner::getMap(const nav_msgs::OccupancyGridConstPtr &map)
{
	ROS_INFO("get map");
	get_map = true;
	map_ = map;
	mfResolution = map_->info.resolution;
	mfWidth = map_->info.width;
	mfHeight = map_->info.height;
	mpOrigin.fx = map_->info.origin.position.x;
	mpOrigin.fy = map_->info.origin.position.y;
	max_index = mfWidth * mfHeight - 1;
	if (!if_calc_targets)
	{
		if_calc_targets = true;
		calc_targets();
	}
}

//发布规定的消毒区域的位置
void TargetPlanner::pubBoundBox()
{
	geometry_msgs::PolygonStamped bound_box_msg;
	bound_box_msg.header.frame_id = msFrame_id;
	bound_box_msg.header.stamp = ros::Time::now();

	geometry_msgs::Point32 A;
	geometry_msgs::Point32 B;
	geometry_msgs::Point32 C;
	geometry_msgs::Point32 D;

	A.x = x_max;
	A.y = y_max;
	B.x = x_min;
	B.y = y_max;
	C.x = x_min;
	C.y = y_min;
	D.x = x_max;
	D.y = y_min;

	bound_box_msg.polygon.points.push_back(A);
	bound_box_msg.polygon.points.push_back(B);
	bound_box_msg.polygon.points.push_back(C);
	bound_box_msg.polygon.points.push_back(D);

	bound_box_pub.publish(bound_box_msg);
}

//根据接收到的全局膨胀代价地图，计算目标点,整个规划是弓型路径规划
void TargetPlanner::calc_targets()
{
	int count = 1;
	for (float y = y_min; y < y_max; y += disinfect_reso)
	{
		++count;
		if (count % 2 == 0) //偶数规划从左到右
		{
			for (float x = x_min; x < x_max; x += disinfect_reso)
			{
				if (check_collision(x, y) == 0)
				{
					geometry_msgs::PoseStamped target_pose;

					target_pose.header.stamp = ros::Time::now();
					target_pose.header.frame_id = msFrame_id;

					target_pose.pose.position.x = x;
					target_pose.pose.position.y = y;
					target_pose.pose.position.z = 0.0;
					target_pose.pose.orientation.x = 0.0;
					target_pose.pose.orientation.y = 0.0;
					target_pose.pose.orientation.z = 0.0;
					target_pose.pose.orientation.w = 1.0;
					targets.push_back(target_pose);
				}
			}
		}

		else
		{
			for (float x = x_max; x > x_min; x -= disinfect_reso)
			{
				if (check_collision(x, y) == 0)
				{
					geometry_msgs::PoseStamped target_pose;
					target_pose.header.stamp = ros::Time::now();
					target_pose.header.frame_id = msFrame_id;
					target_pose.pose.position.x = x;
					target_pose.pose.position.y = y;
					target_pose.pose.position.z = 0.0;
					target_pose.pose.orientation.x = 0.0;
					target_pose.pose.orientation.y = 0.0;
					target_pose.pose.orientation.z = 0.0;
					target_pose.pose.orientation.w = 1.0;
					targets.push_back(target_pose);
				}
			}
		}
	}
	targets_size = targets.size();
	std::cout << "targets size= " << targets.size() << std::endl;
	if (targets_size > 1)
	{
		//计算targets的朝向角，第一个点的朝向为随机，后面每个点的朝向为其前一个点指向此点的方向
		for (int i = 1; i < targets.size(); ++i)
		{
			double yaw = atan2(targets[i].pose.position.y - targets[i - 1].pose.position.y, targets[i].pose.position.x - targets[i - 1].pose.position.x);
			//std::cout << "yaw=" << yaw << std::endl;

			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw); //只通过y即绕z的旋转角度计算四元数，用于平面
			targets[i].pose.orientation.x = q.x;
			targets[i].pose.orientation.y = q.y;
			targets[i].pose.orientation.z = q.z;
			targets[i].pose.orientation.w = q.w;
		}
	}

	ROS_INFO("targets calc finished");

	if (targets_size > 1)
		pub_target.publish(targets[targets_number_now]); //发送当前需要到达的target
	else
	{
		ROS_WARN("NO TARGET ");
	}
}

int TargetPlanner::check_collision(float x, float y) //检验此点是否在全局代价地图中的危险位置
{
	int x_ = (x - mpOrigin.fx) / mfResolution;
	int y_ = (y - mpOrigin.fy) / mfResolution;
	int index = y_ * mfWidth + x_;
	if (index < max_index)
	{
		if (map_->data[index] == 0) //只有点在这个地图区域，且被探索过，且无障碍物，才将其选入
			return 0;
	}
	return 1;
}

//判断车辆是否到达目标点，这里可以有一个计数，如果到达目标点时间间隔超过60s，则自动发送下一个目标点，表示这个目标点被临时占据，无法到达
void TargetPlanner::if_arrive_target()
{
	if (targets_number_now >= targets_size - 1)
	{
		ROS_INFO("arrived last position");
		return;
	}
	float dist = calc_dist((targets[targets_number_now].pose.position.x - car_pos.x), (targets[targets_number_now].pose.position.y - car_pos.y));
	ros::Time begin = ros::Time::now(); //此时的时间
	if ((begin - last).toSec() > max_waiting_time)
	{
		++targets_number_now;
		last = begin;
	}
	else if (dist < min_dist2target)
	{
		++targets_number_now;
	}
	pub_target.publish(targets[targets_number_now]); //发送当前需要到达的target
	ROS_INFO("pub target");
}

float TargetPlanner::calc_dist(float dx, float dy)
{
	return sqrt(dx * dx + dy * dy);
}

//每收到一次amcl发过来的全局位置，则计算发送的target序号，然后将target发送出去
void TargetPlanner::getCarPos(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose)
{

	geometry_msgs::Pose Amclpose; //订阅机器人的姿态信息
	car_pos.x = Pose->pose.pose.position.x;
	car_pos.y = Pose->pose.pose.position.y;
	if (if_calc_targets) //目标点计算之后才进行判断
	{
		if_arrive_target(); //判断发送的点的序号，并将点发出去
		pubBoundBox();
	}
}

#endif
