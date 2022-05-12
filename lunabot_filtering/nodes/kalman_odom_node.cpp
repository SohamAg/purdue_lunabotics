#include <ros/ros.h>
#include <lunabot_filtering/kalman.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const static nav_msgs::Odometry vec_to_odom(const Vec &vec)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = vec(0);
	odom.pose.pose.position.y = vec(3);
	odom.pose.pose.position.z = vec(6);
	tf2::Quaternion q;
	q.setRPY(0, 0, vec(9));
	odom.pose.pose.orientation.x = q.getX();
	odom.pose.pose.orientation.y = q.getY();
	odom.pose.pose.orientation.z = q.getZ();
	odom.pose.pose.orientation.w = q.getW();
	odom.twist.twist.linear.x = vec(1);
	odom.twist.twist.linear.y = vec(4);
	odom.twist.twist.linear.z = vec(7);
	odom.twist.twist.angular.z = vec(10);
	return odom;
}

const static Vec odom_to_vec(const nav_msgs::Odometry &odom)
{
	Vec vec;
	vec(0) = odom.pose.pose.position.x;
	vec(3) = odom.pose.pose.position.y;
	vec(6) = odom.pose.pose.position.z;
	tf2::Quaternion q(odom.pose.pose.orientation.x,
					  odom.pose.pose.orientation.y,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	vec(9) = yaw;
	vec(1) = odom.twist.twist.linear.x;
	vec(4) = odom.twist.twist.linear.y;
	vec(7) = odom.twist.twist.linear.z;
	vec(10) = odom.twist.twist.angular.z;
	return vec;
}

class KalmanOdomNode
{
private:
	ros::NodeHandle nh_;
	const int dt = 1;
	Kalman k;
	ros::Publisher corrected_pose_pub;
	ros::Subscriber apriltag_pose_sub;
	ros::Subscriber t265_odom_sub;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber imu_sub;

	Vec odom_in_[SENSOR_CNT]; // [apriltag, t265, imu]
	Vec odom_corrected_;
	Vec ctrl_u_;

	void cmd_vel_cb(const geometry_msgs::Twist &msg);
	void pose_apriltag_cb(const nav_msgs::Odometry &msg);
	void odom_t265_cb(const nav_msgs::Odometry &msg);
	void imu_cb(const sensor_msgs::Imu &msg);
	void publish_corrected();

public:
	KalmanOdomNode();
	void update();
};

void KalmanOdomNode::pose_apriltag_cb(const nav_msgs::Odometry &msg)
{
	odom_in_[APRILTAG] = odom_to_vec(msg);
}

void KalmanOdomNode::odom_t265_cb(const nav_msgs::Odometry &msg)
{
	odom_in_[T265] = odom_to_vec(msg);
}

void KalmanOdomNode::imu_cb(const sensor_msgs::Imu &msg)
{
	odom_in_[IMU][2] = msg.linear_acceleration.x;
	odom_in_[IMU][5] = msg.linear_acceleration.y;
	odom_in_[IMU][8] = msg.linear_acceleration.z;
	odom_in_[IMU][10] = msg.angular_velocity.z;
}

void KalmanOdomNode::cmd_vel_cb(const geometry_msgs::Twist &msg)
{
	ctrl_u_(1) = msg.linear.x;
	ctrl_u_(4) = msg.linear.y;
	ctrl_u_(7) = msg.linear.z;
	ctrl_u_(10) = msg.angular.z;
}

void KalmanOdomNode::publish_corrected()
{
	nav_msgs::Odometry odom = vec_to_odom(odom_corrected_);
	corrected_pose_pub.publish(odom);
}

KalmanOdomNode::KalmanOdomNode()
{
	// Matrices for sensor zero - apriltag
	Mat P0, H0, R0;
	P0.setZero();
	P0(0, 0) = 1.0; // x
	P0(3, 3) = 1.0; // y
	P0(6, 6) = 1.0; // z
	H0 = P0;
	R0 = P0;

	// Matrices for sensor one - odometry cam
	Mat P1, H1, R1;
	P1.setIdentity();
	P1(2, 2) = 0.0; // x''
	P1(5, 5) = 0.0; // y''
	P1(8, 8) = 0.0; // z''
	P1(11, 11) = 0.0; // theta''
	H1 = P1;
	R1 = P1;


	// Matrices for sensor two - imu
	Mat P2, H2, R2;
	P2.setZero();
	P2(2, 2) = 1.0; // x''
	P2(5, 5) = 1.0; // y''
	P2(8, 8) = 1.0; // z''
	P2(10, 10) = 1.0; // theta''
	H2 = P2;
	R2 = P2;


	// General values
	Mat Q, F, G;
	Q.setConstant(0.1);
	F << 1.0, dt * 1.0, dt * dt * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0, 1.0, dt * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, dt * 1.0, dt * dt * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0, 1.0, dt * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt * 1.0, dt * dt * 0.5, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1.0, dt * 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt * 1.0, dt * dt * 0.5,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1.0, dt * 1.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 1.0;
	G.setIdentity();

	Vec x0;
	x0.setZero();

	Mat Parray[SENSOR_CNT] = {P0, P1, P2};
	Mat Harray[SENSOR_CNT] = {H0, H1, H2};
	Mat Rarray[SENSOR_CNT] = {R0, R1, R2};

	ctrl_u_.setZero();

	k = Kalman(x0, Q, F, G, Parray, Harray, Rarray);

	std::string corrected_odom_topic;
	std::string apriltag_pose_topic;
	std::string t265_odom_topic;
	std::string imu_topic;
	std::string cmd_vel_topic;
	ros::param::get("corrected_odom_topic", corrected_odom_topic);
	ros::param::get("apriltag_pose_topic", apriltag_pose_topic);
	ros::param::get("t265_odom_topic", t265_odom_topic);
	ros::param::get("imu_topic", imu_topic);
	ros::param::get("/cmd_vel_topic", cmd_vel_topic);
	corrected_pose_pub = nh_.advertise<nav_msgs::Odometry>(corrected_odom_topic, 10);
	apriltag_pose_sub = nh_.subscribe(apriltag_pose_topic, 10, &KalmanOdomNode::pose_apriltag_cb, this);
	t265_odom_sub = nh_.subscribe(t265_odom_topic, 10, &KalmanOdomNode::odom_t265_cb, this);
	imu_sub = nh_.subscribe(imu_topic, 10, &KalmanOdomNode::imu_cb, this);
	cmd_vel_sub = nh_.subscribe(cmd_vel_topic, 10, &KalmanOdomNode::cmd_vel_cb, this);
}

void KalmanOdomNode::update()
{
	for (int i = 0; i < SENSOR_CNT; i++)
	{
		k.predict(i, ctrl_u_);
		k.correct(i, odom_in_[i]);
	}
	odom_corrected_ = k.getX();
	publish_corrected();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kalman_node");
	ros::Rate r(15);
	KalmanOdomNode k;

	while (ros::ok())
	{
		k.update();
	}
}