#include <opponent_estimator/opponent_estimator.hpp>

void OpponentEstimator::visualize_costmap() {
    if (costmap.empty()) {
        return;
    }

    double grid_resolution = this->get_parameter("costmap_res").as_double();
    double plot_resolution = this->get_parameter("plot_res").as_double();
    int down_sample = std::max(1, int(plot_resolution / grid_resolution));

    MarkerArray marker_arr;

    int id = 0;

    for (int i = 0; i < (int) costmap[0].size(); i++) {
        if (i % down_sample) continue;
        for (int j = 0; j < (int) costmap[0][0].size(); j++) {
            if (j % down_sample) continue;
            if (costmap[0][i][j] == 0.0) continue;

            // Transform to map frame
            double x = costmap[1][i][j] * cos(ego_global_pose[2]) - costmap[2][i][j] * sin(ego_global_pose[2]) + ego_global_pose[0];
            double y = costmap[1][i][j] * sin(ego_global_pose[2]) + costmap[2][i][j] * cos(ego_global_pose[2]) + ego_global_pose[1];

            // Add marker
            Marker marker;
            marker.header.frame_id = "/map";
            marker.id = i;
            marker.ns = "occupancy_grid_" + std::to_string(id++);
            marker.type = Marker::CUBE;
            marker.action = Marker::ADD;

            marker.pose.position.x = x;
            marker.pose.position.y = y;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.lifetime.nanosec = int(1e8);

            marker_arr.markers.push_back(marker);
        }
    }

    costmap_pub->publish(marker_arr);
}

void OpponentEstimator::visualize_opp_pose() {
    if (opp_global_pose.empty()) {
        return;
    }

    Marker marker;
    marker.header.frame_id = "/map";
    marker.id = 0;
    marker.ns = "opponent_pose";
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;

    marker.pose.position.x = opp_global_pose[0];
    marker.pose.position.y = opp_global_pose[1];

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.lifetime.nanosec = int(1e8);

    opp_pose_viz_pub->publish(marker);
}