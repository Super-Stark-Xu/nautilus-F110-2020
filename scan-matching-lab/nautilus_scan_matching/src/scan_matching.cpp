#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

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

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include "nautilus_scan_matching/JumpTable.h"

using namespace std;

class ScanMatching 
{
  private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_lis;

    vector<geometry_msgs::Point> prev_points_;
    vector<geometry_msgs::Point> curr_points_;
    vector<geometry_msgs::Point> transformed_points_;

    tf::Transform tr_;
    tf::StampedTransform tf_base_laser;
    sensor_msgs::LaserScan prev_scan_;
    geometry_msgs::PoseStamped estimated_pose_;
    
    //Topic names for publishers and subscribers
    const string& SCAN_TOPIC  = "/scan";
    const string& POSE_TOPIC = "/scan_match_pose";
    const string& ODOM_TOPIC = "/odom";
    const string& FAKE_SCAN_TOPIC = "/fake_scan_match";
    const string& DRIVE_TOPIC = "/nav";
    const int MAX_ITERATIONS = 100;

    //Publishers
    ros::Publisher pose_pub_;
    ros::Publisher fake_scan_pub_;
    ros::Publisher drive_pub_;

    //Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

  public:
    ScanMatching()
    {
        cout<<"Node started..\n";
        //Publishers : Add others as per your need
        drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 1);
        fake_scan_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(FAKE_SCAN_TOPIC, 1);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);

        //Subscriber for odometry and laserscan
        odom_sub_ = nh_.subscribe(ODOM_TOPIC, 10, &ScanMatching::odom_callback, this);
        scan_sub_ = nh_.subscribe(SCAN_TOPIC, 1, &ScanMatching::scan_callback, this);

        // Initialize prev_scan_ and tr_
        prev_scan_ = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>(SCAN_TOPIC, nh_, ros::Duration(5.0)));
        prev_points_ = convert_LaserScan_toPCL(prev_scan_);
        tr_.setIdentity();
        tf_lis.lookupTransform("/base_link", "/laser", ros::Time(0), tf_base_laser);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        geometry_msgs::Pose pose_msg = odom_msg->pose.pose;
        Eigen::Affine3d map_base_eigen;
        Eigen::Affine3d base_laser_eigen;
        Eigen::Affine3d map_laser_eigen;
        tf::poseMsgToEigen(pose_msg, map_base_eigen);
        tf::transformTFToEigen(tf_base_laser, base_laser_eigen);
        
        map_laser_eigen = map_base_eigen*base_laser_eigen;
        tf::transformEigenToTF(map_laser_eigen, tr_);
    }

    nautilus_scan_matching::JumpTable update_jump_table(const sensor_msgs::LaserScan prev_scan)
    {
        int i,j;
        nautilus_scan_matching::JumpTable prev_scan_jt;

        for(i=0; i<prev_scan.ranges.size(); i++)
        {
            j = i+1;
            while(j<prev_scan.ranges.size() && prev_scan.ranges[j] <= prev_scan.ranges[i]) j++;
            prev_scan_jt.up_bigger.push_back(j-i);

            j = i+1;
            while(j<prev_scan.ranges.size() && prev_scan.ranges[j] >= prev_scan.ranges[i]) j++;
            prev_scan_jt.up_smaller.push_back(j-i);

            j = i-1;
            while(j>=0 && prev_scan.ranges[j] <= prev_scan.ranges[i]) j--;
            prev_scan_jt.down_bigger.push_back(j-i);

            j = i-1;
            while(j>=0 && prev_scan.ranges[j] >= prev_scan.ranges[i]) j--;
            prev_scan_jt.down_smaller.push_back(j-i);            
        }
        return prev_scan_jt;
    }

    std::vector<geometry_msgs::Point> convert_LaserScan_toPCL(const sensor_msgs::LaserScan scan_msg)
    {
        double rad, ang;
        geometry_msgs::Point temp_pt;
        std::vector<geometry_msgs::Point> points_;
        for(int i=0; i<scan_msg.ranges.size(); i++)
        {
            rad = scan_msg.ranges[i];
            ang = scan_msg.angle_min + i*scan_msg.angle_increment;
            temp_pt.x = rad*cos(ang);
            temp_pt.y = rad*sin(ang);
            temp_pt.z = 0.00;
            points_.push_back(temp_pt);
        }
        return points_;
    }

    std::vector<geometry_msgs::Point> get_roto_translation(const std::vector<geometry_msgs::Point> points, const tf::Transform tr)
    {
        geometry_msgs::Point temp_pt;
        Eigen::Vector3d temp_eigen;
        Eigen::Vector3d pt_eigen;
        Eigen::Affine3d tr_eigen;
        std::vector<geometry_msgs::Point> trans_points_;
        for(int i=0; i<points.size(); i++)
        {
            tf::pointMsgToEigen(points[i], pt_eigen);
            tf::transformTFToEigen(tr, tr_eigen);
            temp_eigen = tr_eigen*pt_eigen;
            tf::pointEigenToMsg(temp_eigen, temp_pt);
            trans_points_.push_back(temp_pt);
        }
        return trans_points_;
    }

    void do_scan_matching(const sensor_msgs::LaserScan curr_scan_) 
    {
        //The following steps are inspired from the Censi's PL-ICP paper. Try to modularize your code for each step to make it more readable and debuggable.
        //Use Eigen math library for linear algebra, matrix and vector operations, geometrical transformations.
        
        /*1.Compute the coordinates of the second scan’s points in the first scan’s frame of reference, 
        according to the roto-translation obtained from odometry update.*/
        curr_points_ = convert_LaserScan_toPCL(curr_scan_);
        transformed_points_ = get_roto_translation(curr_points_, tr_);
        
        /*2.Find correspondence between points of the current and previous frame. You can use naive way of looking 
        through all points in sequence or use radial ordering of laser points to speed up the search.*/
        nautilus_scan_matching::JumpTable prev_scan_jt = update_jump_table(prev_scan_);

        
        //3. Based on the correspondences, find the necessary tranform.
            //3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.

            //3.b. You should get a fourth order polynomial in lamda which you can solve to get value(hint:greatest real root of polynomial equn) of lamda.

            //3.b. Use the calcualted value of lamda to estimate the transform using equation 24 in the Censi's paper.


        //4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.


        /*5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it. Ideally, this should coincide
        with your actual current laserscan message.*/

        // update the prev_scan_ & prev_points_
        prev_scan_ = curr_scan_;
        prev_points_ = curr_points_;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        // get the curr_scan_ 
        sensor_msgs::LaserScan curr_scan_ = *scan_msg;
        
        // do the scan matching
        do_scan_matching(curr_scan_);
    }

    ~ScanMatching() {}
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "scan_matcher");
    ScanMatching scan_matching;
    ros::spin();
    return 0;
}
