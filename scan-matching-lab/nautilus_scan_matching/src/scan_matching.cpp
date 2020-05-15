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
#include "nautilus_scan_matching/Correspondence.h"
#include "nautilus_scan_matching/poly34.hpp"

using namespace std;

inline double SQUARE(double s)
{
	return s*s;
}

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
	tf::Transform prev_tr_;

	tf::StampedTransform tf_base_laser;
	sensor_msgs::LaserScan prev_scan_;
	geometry_msgs::PoseStamped estimated_pose_;

	//Topic names & CONST
	const string& SCAN_TOPIC  = "/scan";
	const string& POSE_TOPIC = "/scan_match_pose";
	const string& ODOM_TOPIC = "/odom";
	const string& FAKE_SCAN_TOPIC = "/fake_scan_match";
	const string& DRIVE_TOPIC = "/nav";
	const int MAX_ITERATIONS = 25;
	const double MAX_CORRESPONDENCE_DIST = 3.0;

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
		pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
		fake_scan_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(FAKE_SCAN_TOPIC, 1);
		
		//Subscriber for odometry and laserscan
		odom_sub_ = nh_.subscribe(ODOM_TOPIC, 10, &ScanMatching::odom_callback, this);
		scan_sub_ = nh_.subscribe(SCAN_TOPIC, 1, &ScanMatching::scan_callback, this);

		// Initialize prev_scan_ and tr_
		prev_scan_ = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>(SCAN_TOPIC, nh_, ros::Duration(5.0)));
		prev_points_ = convert_LaserScan_toPCL(prev_scan_);
		prev_tr_ = update_transform(tr_);
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

	tf::Transform update_transform(tf::Transform tr)
	{
		tf::Transform new_tr = tr;
		return new_tr;
	}

	nautilus_scan_matching::JumpTable update_jump_table(const sensor_msgs::LaserScan scan_msg)
	{
		int i,j;
		nautilus_scan_matching::JumpTable scan_jt;

		for(i=0; i<scan_msg.ranges.size(); i++)
		{
			j = i+1;
			while(j<scan_msg.ranges.size() && scan_msg.ranges[j] <= scan_msg.ranges[i]) j++;
			scan_jt.up_bigger.push_back(j-i);

			j = i+1;
			while(j<scan_msg.ranges.size() && scan_msg.ranges[j] >= scan_msg.ranges[i]) j++;
			scan_jt.up_smaller.push_back(j-i);

			j = i-1;
			while(j>=0 && scan_msg.ranges[j] <= scan_msg.ranges[i]) j--;
			scan_jt.down_bigger.push_back(j-i);

			j = i-1;
			while(j>=0 && scan_msg.ranges[j] >= scan_msg.ranges[i]) j--;
			scan_jt.down_smaller.push_back(j-i);
		}
		return scan_jt;
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

	visualization_msgs::MarkerArray getFakeScan(const std::vector<geometry_msgs::Point> prev_points_, const tf::Transform tr_q)
	{
		visualization_msgs::MarkerArray fake_scan_msg;
		visualization_msgs::Marker temp_marker;
		std::vector<geometry_msgs::Point> transformed_fake_points_;
		transformed_fake_points_ = get_roto_translation(prev_points_, tr_q);

		for(int i=0; i<transformed_fake_points_.size(); i++)
		{
			temp_marker.type = temp_marker.SPHERE;
			temp_marker.header.stamp = ros::Time::now();
			temp_marker.header.frame_id = "laser";
			temp_marker.pose.position = transformed_fake_points_[i];

			temp_marker.color.r = 1.0;
			temp_marker.color.g = 0.0;
			temp_marker.color.b = 0.0;
			temp_marker.color.a = 1.0;

			temp_marker.scale.x = 0.1;
			temp_marker.scale.y = 0.1;
			temp_marker.scale.z = 0.1;

			fake_scan_msg.markers.push_back(temp_marker);
		}
		return fake_scan_msg;
	}

	void Cartesian_toPolar(const geometry_msgs::Point tp, double &norm, double &theta)
	{
		norm = sqrt(tp.x*tp.x + tp.y*tp.y);
		theta = atan2(tp.y, tp.x);
	}

	double calculateDistance(const geometry_msgs::Point P_iw, const geometry_msgs::Point P_up)
	{
		double x = P_iw.x - P_up.x;
		double y = P_iw.y - P_up.y;
		return (x*x + y*y);
	}

	std::vector<nautilus_scan_matching::Correspondence> findCorrespondence(nautilus_scan_matching::JumpTable prev_scan_jt)
	{
		nautilus_scan_matching::Correspondence temp_corr;
		std::vector<nautilus_scan_matching::Correspondence> C_k;

		const int MIN_IDX = 0;
		const int MAX_IDX = transformed_points_.size()-1;

		int last_best = -1;
		double C = (double)prev_scan_.ranges.size()/(prev_scan_.angle_max - prev_scan_.angle_min);

		for(int i=0; i<transformed_points_.size(); i++)
		{
			double p_iw_norm, p_iw_phi;
			Cartesian_toPolar(transformed_points_[i], p_iw_norm, p_iw_phi);
			int start_cell = (int)((p_iw_phi - prev_scan_.angle_min)*C);

			int j1 = -1;
			double best_dist = 2*prev_scan_.range_max;
			int we_start_at;
			if (last_best == -1)
				we_start_at = start_cell;
			else
				we_start_at = last_best + 1;

			if (we_start_at > MAX_IDX) we_start_at = MAX_IDX;
			if (we_start_at < MIN_IDX) we_start_at = MIN_IDX;

			int up = we_start_at+1;
			int down = we_start_at;
			double last_dist_up = 0.00; //doubts set to inf;
			double last_dist_down = -1.00; // same doubts;

			bool up_stopped = false;
			bool down_stopped = false;
			bool now_up;

			while( (!up_stopped) || (!down_stopped))
			{
				if (up_stopped){ now_up = false; }
				else if (down_stopped) { now_up = true; }
				else { now_up = (last_dist_up < last_dist_down); }

				if (now_up)
				{
					if (up > MAX_IDX){up_stopped = true; continue;}

					last_dist_up = calculateDistance(transformed_points_[i], prev_points_[up]);
					if ((last_dist_up < best_dist) || (j1 ==-1)){ j1 = up; best_dist = last_dist_up;}

					if (up > start_cell)
					{
						double up_phi = prev_scan_.angle_min + up*prev_scan_.angle_increment;
						double del_phi = up_phi - p_iw_phi;
						double min_dist_up = sin(del_phi)*p_iw_norm;

						if (SQUARE(min_dist_up) > best_dist){up_stopped = true; continue;}

						if (prev_scan_.ranges[up] < p_iw_norm)
							up += prev_scan_jt.up_bigger[up];
						else
							up += prev_scan_jt.up_smaller[up];
					}
					else
						up++;
				}

				if (!now_up)
				{
					if (down < MIN_IDX){down_stopped = true; continue;}

					last_dist_down = calculateDistance(transformed_points_[i], prev_points_[down]);
					if ((last_dist_down < best_dist) || (j1 == -1)){ j1 = down; best_dist = last_dist_down; }

					if (down < start_cell)
					{

						double down_phi = prev_scan_.angle_min + down*prev_scan_.angle_increment;
						double del_phi = p_iw_phi -  down_phi;
						double min_dist_down = sin(del_phi)*p_iw_norm;

						if (SQUARE(min_dist_down) > best_dist){down_stopped = true; continue;}

						if (prev_scan_.ranges[down] < p_iw_norm)
							down += prev_scan_jt.down_bigger[down];
						else
							down += prev_scan_jt.down_smaller[down];
					}
					else
						down--;
				}
			}

			// outside while
			//if no match point found & accounting for boundary condns.
			if ((j1 <= MIN_IDX) || (j1 >= MAX_IDX) || (j1 == -1) || (best_dist > MAX_CORRESPONDENCE_DIST))
			{
				temp_corr.valid = false;
				temp_corr.j1 = -1;
				temp_corr.j2 = -1;
				temp_corr.dist_j1 = 0.00;
				C_k.push_back(temp_corr);
				continue;
			}

			int j2;
			double j2_dist;

			int j2_up = j1 + 1;
			if (j2_up < MIN_IDX || j2_up > MAX_IDX)
				j2_up = -1;

			int j2_down = j1 - 1;
			if (j2_down < MIN_IDX || j2_down > MAX_IDX)
				j2_down = -1;

			if ((j2_up == -1) && (j2_down == -1))
			{
				temp_corr.valid = false;
				temp_corr.j1 = -1;
				temp_corr.j2 = -1;
				temp_corr.dist_j1 = 0.00;
				C_k.push_back(temp_corr);
				continue;
			}

			// Decide j2 up/down
			if(j2_up == -1)
				j2 = j2_down;

			else if(j2_down == -1)
				j2 = j2_up;

			else
			{
				double j2_up_dist = calculateDistance(transformed_points_[i], prev_points_[j2_up]);
				double j2_down_dist = calculateDistance(transformed_points_[i], prev_points_[j2_down]);
				if (j2_up_dist < j2_down_dist)
				{
					j2 = j2_up;
					j2_dist = j2_up_dist;
				}

				else
				{
					j2 = j2_down;
					j2_dist = j2_down_dist;
				}
			}

			temp_corr.valid = true;
			temp_corr.j1 = j1;
			temp_corr.j2 = j2;
			temp_corr.dist_j1 = best_dist;
			temp_corr.dist_j2 = j2_dist;
			C_k.push_back(temp_corr);

			last_best = j1;
		}

		return C_k;
	}

	void get_MgW_matrices(const std::vector<nautilus_scan_matching::Correspondence> C_k, Eigen::Matrix4d &M, Eigen::RowVector4d &gT, Eigen::Matrix4d &W)
	{
		M = Eigen::MatrixXd::Zero(4, 4);
		gT<<0,0,0,0;
		W<<0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

		Eigen::MatrixXd Mi(2,4);
		Eigen::Vector2d ni;
		Eigen::Matrix2d Ci;
		Eigen::Vector2d PI_i;

		for(int i=0; i<C_k.size(); i++)
		{
			// ignore false match points
			if(C_k[i].valid == false)
				continue;

			int j1 = C_k[i].j1;
			int j2 = C_k[i].j2;

			Mi<<1, 0, curr_points_[i].x, -curr_points_[i].y,
			0, 1, curr_points_[i].y, curr_points_[i].x;

			double dx = prev_points_[j1].x - prev_points_[j2].x;
			double dy = prev_points_[j1].y - prev_points_[j2].y;
			ni<<dy, -dx;
			ni.normalize();
			Ci = ni*ni.transpose();
			PI_i<<prev_points_[j1].x, prev_points_[j1].y;
			Eigen::Matrix4d temp_M = Mi.transpose()*Ci*Mi;
			Eigen::RowVector4d temp_gT = -2.0*PI_i.transpose()*Ci*Mi;
			M += temp_M;
			gT += temp_gT;
		}
	}

	bool find_lambda(double &lambda, const Eigen::Matrix4d M, const Eigen::Vector4d g, const Eigen::Matrix4d W)
	{
		Eigen::Matrix2d I;
		I<<1, 0,
		0, 1;

		Eigen::Matrix2d A;
		Eigen::Matrix2d B;
		Eigen::Matrix2d D;

		A<<M(0,0), M(0,1),
		M(1,0), M(1,1);

		B<<M(0,2), M(0,3),
		M(1,2), M(1,3);

		D<<M(2,2), M(2,3),
		M(3,2), M(3,3);

		//Scaling factor
		// A = 2*A;
		// B = 2*B;
		// D = 2*D;

		Eigen::Matrix2d S;
		S = D - B.transpose()*A.inverse()*B;

		Eigen::Matrix4d F1; //the matrix in the 1st  term of eqn(31);
		Eigen::Matrix4d F2; //the matrix in the 2nd term of eqn(31);
		Eigen::Matrix4d F3; //the matrix in the 3rd term of eqn(31);

		F1<<A.inverse()*B*B.transpose()*A.inverse().transpose(), -A.inverse()*B,
		(-A.inverse()*B).transpose(), I;

		F2<<A.inverse()*B*S.adjoint()*B.transpose()*A.inverse().transpose(), -A.inverse()*B*S.adjoint(),
		(-A.inverse()*B*S.adjoint()).transpose(), S.adjoint();

		F3<<A.inverse()*B*S.adjoint().transpose()*S.adjoint()*B.transpose()*A.inverse().transpose(), -A.inverse()*B*S.adjoint().transpose()*S.adjoint(),
		(-A.inverse()*B*S.adjoint().transpose()*S.adjoint()).transpose(), S.adjoint().transpose()*S.adjoint();

		double t0, t1, t2; // 3rd, 2nd, 1st term of left-side eqn(31)
		t2 = 4*g.transpose()*F1*g;//for lambda^2 term
		t1 = 4*g.transpose()*F2*g;//for lambda term
		t0 = g.transpose()*F3*g;//for const term

		double a, b, c, d; //coefficient of x^4 + a*x^3 + b*x^2 + c*x + d = 0
		a = S(0,0) + S(1,1);
		b = (8*S.determinant() + 4*SQUARE(a) - t2)/16;
		c = (4*a*S.determinant() - t1)/16;
		d = (SQUARE(S.determinant()) - t0)/16;

    	double roots[4];
    	bool real_root;
		int result = SolveP4(roots, a, b, c, d);

    	if (result == 4)
    	{
        		real_root = true;
        		for(int i=0; i<4; i++)
        		{
        	       if (roots[i] > lambda)
            	       lambda = roots[i];
        		}
    	}

    	else if (result == 2)
    	{
        		real_root = true;
        		for(int i=0; i<2; i++)
        		{
            		if (roots[i] > lambda)
                		lambda = roots[i];
        		}
    	}

    	else
        	real_root = false;

		return real_root;
	}

	void do_scan_matching(geometry_msgs::PoseStamped &pose_msg, const sensor_msgs::LaserScan curr_scan_)
	{
		curr_points_ = convert_LaserScan_toPCL(curr_scan_);
		nautilus_scan_matching::JumpTable prev_scan_jt = update_jump_table(prev_scan_);
		tf::Transform cur_tr = update_transform(tr_);
		tf::Transform q = prev_tr_.inverseTimes(cur_tr); //initial guess

		for(int k=0; k<MAX_ITERATIONS; k++)
		{
			/*1.Compute the coordinates of the second scan’s points in the first scan’s frame of reference,
			according to the roto-translation obtained from odometry update.*/
			transformed_points_ = get_roto_translation(curr_points_, q);

			/*2.Find correspondence between points of the current and previous frame. You can use naive way of looking
			through all points in sequence or use radial ordering of laser points to speed up the search.*/
			std::vector<nautilus_scan_matching::Correspondence> C_k;
			C_k = findCorrespondence(prev_scan_jt);

			//3. Based on the correspondences, find the necessary tranform.
			//3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.
			Eigen::Matrix4d M;
			Eigen::Matrix4d W;
			Eigen::RowVector4d gT;
			get_MgW_matrices(C_k, M, gT, W);
			Eigen::Vector4d g = gT.transpose();
			Eigen::Vector4d X_;

			//3.b. You should get a fourth order polynomial in lamda which you can solve to get value(hint:greatest real root of polynomial equn) of lamda
			double lambda;
			bool real_root = find_lambda(lambda, M, g, W);

			// //3.c. Use the calculated value of lamda to estimate the transform using equation 24 in the Censi's paper.
			if (real_root)
			{
				X_ = (-(2*M + 2*lambda*W)).inverse().transpose()*g;
				double yaw = atan2(X_(3), X_(2));

				//update q & cur_tr
				geometry_msgs::Pose temp_pose;
				temp_pose.position.x = X_(0);
				temp_pose.position.y = X_(1);
				temp_pose.position.z = 0.0;
				temp_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

				tf::poseMsgToTF(temp_pose, q);
				cur_tr = prev_tr_ * q;
			}

			else
			{
				break;
			}
		}

		//4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.
		tf_br.sendTransform(tf::StampedTransform(cur_tr, ros::Time::now(), "map", "fake_laser"));
		tf::Stamped<tf::Transform> cur_stamped_tr(cur_tr, ros::Time::now(), "map");
		tf::poseStampedTFToMsg(cur_stamped_tr, pose_msg);
		pose_pub_.publish(pose_msg);

		/*5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it. Ideally, this should
		coincide with your actual current laserscan message.*/
		visualization_msgs::MarkerArray fake_scan;
		fake_scan = getFakeScan(prev_points_, q);
		fake_scan_pub_.publish(fake_scan);

		// update the prev_scan_ & prev_points_
		prev_scan_ = curr_scan_;
		prev_points_ = curr_points_;
		prev_tr_ = cur_tr;
	}

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
	{
		// get the curr_scan_
		sensor_msgs::LaserScan curr_scan_ = *scan_msg;

		// do the scan matching
		do_scan_matching(estimated_pose_, curr_scan_);
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
