#include <fstream>
#include <cmath>

#include "opponent_estimator/opponent_estimator.hpp"

OpponentEstimator::OpponentEstimator() : Node("opponent_estimator_node") {
    // Declare ROS parameters
    this->declare_parameter("is_sim", true);
    this->declare_parameter("race_mode", false);

    this->declare_parameter("costmap_xmin", -6.0);
    this->declare_parameter("costmap_xmax", 6.0);
    this->declare_parameter("costmap_ymin", -3.0);
    this->declare_parameter("costmap_ymax", 6.0);
    this->declare_parameter("costmap_res", 0.1);
    this->declare_parameter("costmap_tol", 0.1);

    this->declare_parameter("plot_res", 0.1);
    
    this->declare_parameter("centerline_fname", "centerline_deviate");
    this->declare_parameter("cluster_dist_tol", 0.6);
    this->declare_parameter("cluster_size_max", 10);

    // QoS settings
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    is_sim = this->get_parameter("is_sim").as_bool();
    if (is_sim) {
        odom_topic = "/ego_racecar/odom";
    } else {
        odom_topic = "/pf/pose/odom";
    }

    // Get centerline
    std::string centerline_fname = this->get_parameter("centerline_fname").as_string();
    std::string centerline_path = "/home/shreyasm/f1tenth_ws/src/opponent_estimator/csv/" + centerline_fname + ".csv";
    read_centerline(centerline_path);

    // Subscribers
    odom_sub = this->create_subscription<Odometry>(odom_topic, qos, std::bind(&OpponentEstimator::pose_callback, this, std::placeholders::_1));
    scan_sub = this->create_subscription<LaserScan>(scan_topic, qos, std::bind(&OpponentEstimator::scan_callback, this, std::placeholders::_1));

    // Publishers
    opp_pose_pub = this->create_publisher<PoseStamped>(opp_pose_topic, qos);
    costmap_pub = this->create_publisher<MarkerArray>(costmap_topic, qos);
    opp_pose_viz_pub = this->create_publisher<Marker>(opp_pose_viz_topic, qos);
}

void OpponentEstimator::pose_callback(const Odometry::ConstSharedPtr odom_msg) {
    double x = odom_msg->pose.pose.orientation.x;
    double y = odom_msg->pose.pose.orientation.y;
    double z = odom_msg->pose.pose.orientation.z;
    double w = odom_msg->pose.pose.orientation.w;

    double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y *y + z * z));

    ego_global_pose = {
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        yaw
    };

    estimate_opp();

    bool race_mode = this->get_parameter("race_mode").as_bool();
    if (!race_mode) {
        visualize_costmap();
        visualize_opp_pose();
    }
}

void OpponentEstimator::scan_callback(const LaserScan::ConstSharedPtr scan_msg) {
    // Clean ranges
    std::vector<float> ranges(scan_msg->ranges);
    for (auto &range : ranges) {
        if (range < scan_msg->range_min) range = scan_msg->range_min;
        if (range > scan_msg->range_max) range = scan_msg->range_max;
    }

    double xmin = this->get_parameter("costmap_xmin").as_double();
    double xmax = this->get_parameter("costmap_xmax").as_double();
    double ymin = this->get_parameter("costmap_ymin").as_double();
    double ymax = this->get_parameter("costmap_ymax").as_double();
    double res = this->get_parameter("costmap_res").as_double();
    double tol = this->get_parameter("costmap_tol").as_double();

    int nx = int((xmax - xmin) / res) + 1;
    int ny = int((ymax - ymin) / res) + 1;

    double xres = (xmax - xmin) / (nx - 1);
    double yres = (ymax - ymin) / (ny - 1);

    std::vector<double> xs(nx), ys(ny);
    std::vector<double>::iterator ptr;
    double val;
    for (ptr = xs.begin(), val = xmin; ptr != xs.end(); ptr++) {
        *ptr = val;
        val += xres;
    }
    for (ptr = ys.begin(), val = ymin; ptr != ys.end(); ptr++) {
        *ptr = val;
        val += yres;
    }

    if (costmap.empty()) {
        std::vector<std::vector<double>> grid_v(nx, std::vector<double>(ny, -1e8));
        std::vector<std::vector<double>> grid_x(nx, std::vector<double>(ny, -1e8));
        std::vector<std::vector<double>> grid_y(nx, std::vector<double>(ny, -1e8));

        costmap.push_back(grid_v);
        costmap.push_back(grid_x);
        costmap.push_back(grid_y);
    }

    // Create occupancy grid in vehicle local frame
    for (int i = 0; i < nx; i++) {
        double x = xs[i];
        for (int j = 0; j < ny; j++) {
            double y = ys[j];
            double r = sqrt(x * x + y * y);
            double theta = atan2(y, x);

            if (theta >= scan_msg->angle_min && theta <= scan_msg->angle_max) {
                int idx = int((theta - scan_msg->angle_min) / scan_msg->angle_increment);
                costmap[0][i][j] = (abs(r - ranges[idx]) < tol);
                
            } else {
                costmap[0][i][j] = 0;
            }

            costmap[1][i][j] = x;
            costmap[2][i][j] = y;
        }
    }
}

void OpponentEstimator::read_centerline(const std::string &path) {
    std::fstream fin;
    fin.open(path, std::ios::in);
    std::string line, word;

    while (getline(fin, line)) {
        std::stringstream s(line);
        std::vector<std::string> row;
        while (getline(s, word, ';')) {
            row.push_back(word);
        }
        centerline.push_back({std::stod(row[0]), std::stod(row[1]), 1.0});
    }
}

void OpponentEstimator::estimate_opp() {
    if (costmap.empty()) {
        return;
    }

    opp_global_pose.clear();
    opponent.clear();

    // Get occupied costmap points in the global frame
    std::vector<std::vector<double>> occ_points;
    for (int i = 0; i < (int) costmap[0].size(); i++) {
        for (int j = 0; j < (int) costmap[0][0].size(); j++) {
            if (costmap[0][i][j] == 0.0) continue;
            double x_car = costmap[1][i][j];
            double y_car = costmap[2][i][j];

            double x = ego_global_pose[0];
            double y = ego_global_pose[1];
            double yaw = ego_global_pose[2];

            double x_global = cos(yaw) * x_car - sin(yaw) * y_car + x;
            double y_global = sin(yaw) * x_car + cos(yaw) * y_car + y;

            occ_points.push_back({x_global, y_global});
        }
    }

    if (occ_points.empty()) {
        PoseStamped opp_state;
        opp_state.pose.position.x = INFINITY;
        opp_state.pose.position.y = INFINITY;
        opp_pose_pub->publish(opp_state);
        return;
    }

    double cluster_dist_tol = this->get_parameter("cluster_dist_tol").as_double();
    int cluster_max = (int) this->get_parameter("cluster_size_max").as_int();
    std::vector<std::vector<int>> clusters = cluster(occ_points, cluster_dist_tol);
    int max_cluster_size = 0;
    int max_cluster_idx = 0;
    for (int i = 0; i < (int) clusters.size(); ++i) {
        if ((int) clusters[i].size() > max_cluster_size && (int) clusters[i].size() < cluster_max) {
            max_cluster_size = (int) clusters[i].size();
            max_cluster_idx = i;
        }
    }

    std::vector<int> opponent_idx = clusters[max_cluster_idx];
    for (const auto i: opponent_idx) {
        opponent.push_back(occ_points[i]);
    }

    // Use average pose for estimation
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto &pt: opponent) {
        mean_x += pt[0];
        mean_y += pt[1];
    }
    mean_x /= double(opponent.size());
    mean_y /= double(opponent.size());
    // std::cout << "opponent_size" << ": " << opponent.size() << std::endl;

    opp_global_pose = {mean_x, mean_y};

    // Publish result
    PoseStamped opp_state;
    opp_state.pose.position.x = opp_global_pose[0];
    opp_state.pose.position.y = opp_global_pose[1];
    opp_pose_pub->publish(opp_state);
}
