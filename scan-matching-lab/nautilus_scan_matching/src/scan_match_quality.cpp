#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include "nautilus_scan_matching/JumpTable.h"
#include "nautilus_scan_matching/Correspondence.h"
#include "nautilus_scan_matching/poly34.hpp"

using namespace std;


class ScanQuality
{
  private:
	ros::NodeHandle nh_;

  std::vector<geometry_msgs::Point> estimated_poses;
  std::vector<geometry_msgs::Point> true_poses;
  vector<double> true_times;
  double prev_time = ros::Time::now().toSec();
  double curr_time;
  double sum = 0.0;
  double num_points = 0;

	//Topic names & CONST
	const string& SCAN_TOPIC  = "/scan";
	const string& POSE_TOPIC = "/scan_match_pose";
	const string& ODOM_TOPIC = "/odom";
	const string& FAKE_SCAN_TOPIC = "/fake_scan_match";
	const string& DRIVE_TOPIC = "/nav";
	const int MAX_ITERATIONS = 100;
	const double MAX_CORRESPONDENCE_DIST = 3.0;
	const int WINDOW_SIZE = 100;

	//Publishers
	ros::Publisher error_pub_;

	//Subscribers
	ros::Subscriber odom_sub_;
	ros::Subscriber pose_sub_;

  public:
	ScanQuality()
	{
		cout<<"Quality node started..\n";
		// Publish the error - TBD
		// error_pub_ = nh_.advertise<double>("/match_error", 1);

		//Subscriber for odometry and laserscan
		odom_sub_ = nh_.subscribe(ODOM_TOPIC, 10, &ScanQuality::odom_callback, this);
		pose_sub_ = nh_.subscribe(POSE_TOPIC, 10, &ScanQuality::match_quality_callback, this);
	}

	void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
	{
		geometry_msgs::Point pose_msg = odom_msg->pose.pose.position;
		double time_msg = odom_msg->header.stamp.sec;

		true_poses.push_back(pose_msg);
		true_times.push_back(time_msg);

    //ROS_INFO("pose_msg: %f, %f, %f\n", pose_msg.x,pose_msg.y,pose_msg.z);
	}

	// Returns element closest to target (estimated time) in true times
    geometry_msgs::Point findClosest(vector<double> times, std::vector<geometry_msgs::Point> true_poses, int n, double target)
    {
        cout.precision(10);
        // Edge cases
        if (target <= times[0]){
            cout <<"(estimate) time: "<< times[0]<<endl;
            return true_poses[0];
        }
        if (target >= times[n - 1]){
            cout <<"(estimate) time: "<< times[n - 1]<<endl;
            return true_poses[n - 1];
        }

        // Binary search
        int i = 0, j = n, mid = 0;
        while (i < j) {
            mid = (i + j) / 2;

            if (times[mid] == target){
                cout <<"(estimate) time: "<< times[mid]<<endl;
                return true_poses[mid];
            }

            /* If target is less than timesay element,
                then search in left */
            if (target < times[mid]) {

                // If target is greater than previous
                // to mid, return closest of two
                if (mid > 0 && target > times[mid - 1]){
                    int ret = getClosest(times[mid - 1],
                                      times[mid], target);
                    if(ret == 1){
                        cout <<"(estimate) time: "<< times[mid - 1]<<endl;
                        return true_poses[mid - 1];
                    }else{
                        cout <<"(estimate) time: "<< times[mid]<<endl;
                        return true_poses[mid];
                    }
                }

                /* Repeat for left half */
                j = mid;
            }

            // If target is greater than mid
            else {
                if (mid < n - 1 && target < times[mid + 1]){
                    int ret = getClosest(times[mid],
                                      times[mid + 1], target);
                    if(ret == 1){
                        cout <<"(estimate) time: "<< times[mid]<<endl;
                        return true_poses[mid];
                    }else{
                        cout <<"(estimate) time: "<< times[mid+1]<<endl;
                        return true_poses[mid + 1];
                    }
                }
                // update i
                i = mid + 1;
            }
        }

        // Only single element left after search
        cout <<"(estimate) time: "<< times[mid]<<endl;
        return true_poses[mid];
    }

    // Function to compare which time value is the closer to the target time
    int getClosest(double val1, double val2, double target)
    {
        if (target - val1 >= val2 - target)
            return 2;  // Value 2
        else
            return 1;  // Value 1
    }

    double get_pose_difference(geometry_msgs::Point estimated_pose,double curr_time){

        // determine closest true l[t] to estimated l[t]
        int n = true_times.size();
        double target = curr_time;
        geometry_msgs::Point true_pos = findClosest(true_times, true_poses, n, target);
        cout << "true_pos: "<<true_pos <<"estimated_pose: "<<estimated_pose;

        // calculate difference in position to true position
        double x_diff = estimated_pose.x - true_pos.x;
        double y_diff = estimated_pose.y - true_pos.y;
        double z_diff = estimated_pose.z - true_pos.z;
        double total_diff = x_diff*x_diff + y_diff*y_diff + z_diff*z_diff;

        return total_diff;
    }

	void do_scan_match_quality(geometry_msgs::Point estimated_pose)
	{
        double curr_time = ros::Time::now().toSec();
        cout << "******************************************"<<endl;
        cout.precision(10);
        cout << "(true) time: "<<curr_time <<endl;
        // Determine the l[t] and estimated l[t] that line up
        // Find the squared difference between them
        double pose_diff = get_pose_difference(estimated_pose,curr_time);

        cout<<"pose_diff: "<<pose_diff<<"\n\n";
        // Add the difference to the sum
        sum += pose_diff;
        num_points += 1;

        // If WINDOW_SIZE has been reached, divide by the number of points to get the average
        if (curr_time - prev_time >= WINDOW_SIZE){
            double avg = sum / num_points;
            true_poses.clear();
            true_times.clear();
            sum = 0.0;
            num_points = 0.0;
            prev_time = ros::Time::now().toSec(); //curr_time;
            cout << "$$$$ Window (w="<<WINDOW_SIZE<<") Error: "<<avg<<" $$$$"<<endl;
        }
	}

	void match_quality_callback(const geometry_msgs::PoseStamped::ConstPtr &match_msg)
	{
		// Get the latest pose estimate
		geometry_msgs::Point estimated_pose = match_msg->pose.position;

		// do scan quality
		do_scan_match_quality(estimated_pose);
	}

	~ScanQuality() {}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_match_quality");
	ScanQuality scan_match_quality;
	ros::spin();
	return 0;
}
