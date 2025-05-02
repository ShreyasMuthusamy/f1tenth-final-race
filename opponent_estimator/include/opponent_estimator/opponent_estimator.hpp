#ifndef OPPONENT_ESTIMATOR_H
#define OPPONENT_ESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

typedef sensor_msgs::msg::LaserScan LaserScan;
typedef nav_msgs::msg::Odometry Odometry;
typedef geometry_msgs::msg::PoseStamped PoseStamped;
typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;

class OpponentEstimator : public rclcpp::Node {
    public:
        OpponentEstimator();
    
    private:
        bool is_sim = false;

        rclcpp::QoS qos = rclcpp::QoS(1);

        // Obstacle detection variables
        std::vector<std::vector<std::vector<double>>> costmap;
        std::vector<std::vector<double>> centerline;
        std::vector<std::vector<double>> opponent;

        // State variables
        std::vector<double> ego_global_pose;
        std::vector<double> opp_global_pose;

        // Topic names
        std::string odom_topic;
        std::string scan_topic = "/scan";
        std::string opp_pose_topic = "/opp_hat/pose";

        std::string costmap_topic = "/costmap";
        std::string opp_pose_viz_topic = "/opp_hat/viz/pose";

        // Subscribers
        rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<LaserScan>::SharedPtr scan_sub;

        // Publishers
        rclcpp::Publisher<PoseStamped>::SharedPtr opp_pose_pub;

        rclcpp::Publisher<MarkerArray>::SharedPtr costmap_pub;
        rclcpp::Publisher<Marker>::SharedPtr opp_pose_viz_pub;

        // Callbacks
        void pose_callback(const Odometry::ConstSharedPtr odom_msg);
        void scan_callback(const LaserScan::ConstSharedPtr scan_msg);

        // CSV handler functions
        void read_centerline(const std::string &path);

        // Ego and opponent estimation functions
        void estimate_opp();
        std::vector<std::vector<int>> cluster(const std::vector<std::vector<double>> &points, double tol);
        void connect(std::vector<int>& parents, int i, int j);
        int find(std::vector<int>& parents, int i);

        // // Visualization
        void visualize_costmap();
        void visualize_opp_pose();
};

#endif
