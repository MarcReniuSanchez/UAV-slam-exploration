#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/tf.h>

#define M_PI  3.14159265358979323846  /* pi */

bool sim_running = false;
static const int64_t kNanoSecondsInSecond = 1000000000;
const float DEG_2_RAD = M_PI / 180.0;

const double radius = 2.0; /* meters */
const double height = 1.5; /* operation height, meters */
const double wait_time = 1.0; /* wait time between waypoints, seconds */
const int num_waypoints = 10; /* number of waypoints */

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

WaypointWithTime generateWaypoint() {
  WaypointWithTime wp;
  double t, x, y, z, yaw;
  double r = radius * sqrt(((double) rand() / (RAND_MAX)));

  t = wait_time;
  z = height;
  yaw = (rand() % 360);
  // Random generation of x & y
  x = r * cos(yaw);
  y = r * sin(yaw);
  // yaw to radians
  yaw = yaw * DEG_2_RAD;

  wp = WaypointWithTime(t, x, y, z, yaw);
  ROS_INFO("Waypoint x: %2.2f y: %2.2f, z: %2.2f, yaw: %2.2f", x, y, z, yaw);
  return wp;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle nh;

	ROS_INFO("Initializing waypoint generator...");

  std::vector<WaypointWithTime> waypoints;

  // Initializing random seed
  srand (time(NULL));

  for (int i = 0; i < num_waypoints; ++i) {
    waypoints.push_back(generateWaypoint());
  }
  ROS_INFO("Generated %d waypoints.", (int )waypoints.size());

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  ros::Duration(2).sleep();

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }
  wp_pub.publish(msg);

  return 0;
}
