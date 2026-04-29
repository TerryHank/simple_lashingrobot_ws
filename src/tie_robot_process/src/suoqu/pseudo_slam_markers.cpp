#include "suoqu_runtime_internal.hpp"

#include <cmath>
#include <fstream>

void clear_pseudo_slam_markers()
{
    if (!pub_pseudo_slam_markers) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_points.clear();
        pseudo_slam_marker_outlier_global_indices.clear();
        pseudo_slam_marker_outlier_secondary_plane_global_indices.clear();
        pseudo_slam_marker_outlier_line_global_indices.clear();
        pseudo_slam_marker_outlier_column_neighbor_global_indices.clear();
        pseudo_slam_marker_execution_state = PseudoSlamMarkerExecutionState{};
    }
    {
        std::lock_guard<std::mutex> path_origin_lock(pseudo_slam_marker_path_origin_mutex);
        pseudo_slam_marker_path_origin = Cabin_Point{};
        pseudo_slam_marker_path_origin_valid = false;
    }

    visualization_msgs::Marker clear_marker;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.header.frame_id = "map";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(clear_marker);
    pub_pseudo_slam_markers.publish(marker_array);
}

void publish_pseudo_slam_markers(const std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    if (!pub_pseudo_slam_markers) {
        return;
    }

    PseudoSlamMarkerExecutionState marker_execution_state_snapshot;
    std::unordered_set<int> marker_outlier_global_indices_snapshot;
    std::unordered_set<int> marker_outlier_secondary_plane_global_indices_snapshot;
    std::unordered_set<int> marker_outlier_line_global_indices_snapshot;
    std::unordered_set<int> marker_outlier_column_neighbor_global_indices_snapshot;
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_points = world_points;
        marker_outlier_global_indices_snapshot = pseudo_slam_marker_outlier_global_indices;
        marker_outlier_secondary_plane_global_indices_snapshot =
            pseudo_slam_marker_outlier_secondary_plane_global_indices;
        marker_outlier_line_global_indices_snapshot = pseudo_slam_marker_outlier_line_global_indices;
        marker_outlier_column_neighbor_global_indices_snapshot =
            pseudo_slam_marker_outlier_column_neighbor_global_indices;
        marker_execution_state_snapshot = pseudo_slam_marker_execution_state;
    }

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker points_marker;
    points_marker.header.stamp = ros::Time::now();
    points_marker.header.frame_id = "map";
    points_marker.ns = "pseudo_slam_points";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.025;
    points_marker.scale.y = 0.025;
    points_marker.scale.z = 0.025;
    points_marker.color.a = 1.0f;

    for (size_t point_pos = 0; point_pos < world_points.size(); ++point_pos) {
        const auto& world_point = world_points[point_pos];
        const int global_idx = world_point.idx > 0 ? world_point.idx : static_cast<int>(point_pos) + 1;
        const bool is_active_dispatch_point =
            marker_execution_state_snapshot.active_dispatch_global_indices.count(global_idx) > 0;
        const bool is_highlighted_area_point =
            marker_execution_state_snapshot.highlighted_area_global_indices.count(global_idx) > 0;
        const bool is_outlier_column_neighbor_blocked =
            marker_outlier_column_neighbor_global_indices_snapshot.count(global_idx) > 0;
        const bool is_outlier_secondary_plane_point =
            marker_outlier_secondary_plane_global_indices_snapshot.count(global_idx) > 0;
        const bool is_outlier_line_point =
            marker_outlier_line_global_indices_snapshot.count(global_idx) > 0;
        const bool is_outlier_point =
            marker_outlier_global_indices_snapshot.count(global_idx) > 0;

        geometry_msgs::Point point_msg;
        point_msg.x = static_cast<double>(world_point.World_coord[0]) / 1000.0;
        point_msg.y = static_cast<double>(world_point.World_coord[1]) / 1000.0;
        point_msg.z = static_cast<double>(world_point.World_coord[2]) / 1000.0;
        points_marker.points.push_back(point_msg);

        std_msgs::ColorRGBA point_color;
        point_color.a = 1.0f;
        if (is_active_dispatch_point) {
            point_color.r = 1.0f;
            point_color.g = 0.20f;
            point_color.b = 0.20f;
        } else if (is_highlighted_area_point) {
            point_color.r = 1.0f;
            point_color.g = 0.92f;
            point_color.b = 0.10f;
        } else if (is_outlier_secondary_plane_point) {
            point_color.r = 0.55f;
            point_color.g = 0.85f;
            point_color.b = 0.30f;
        } else if (is_outlier_column_neighbor_blocked || is_outlier_line_point || is_outlier_point) {
            point_color.r = 0.05f;
            point_color.g = 0.05f;
            point_color.b = 0.05f;
        } else {
            point_color.r = 0.10f;
            point_color.g = 0.95f;
            point_color.b = 0.85f;
        }
        points_marker.colors.push_back(point_color);

        visualization_msgs::Marker label_marker;
        label_marker.header.stamp = points_marker.header.stamp;
        label_marker.header.frame_id = "map";
        label_marker.ns = "pseudo_slam_labels";
        label_marker.id = static_cast<int>(point_pos) + 1;
        label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label_marker.action = visualization_msgs::Marker::ADD;
        label_marker.pose.position = point_msg;
        label_marker.pose.position.z += 0.05;
        label_marker.pose.orientation.w = 1.0;
        label_marker.scale.z = 0.05;
        label_marker.color = point_color;
        label_marker.text = std::to_string(global_idx);
        marker_array.markers.push_back(label_marker);
    }

    marker_array.markers.insert(marker_array.markers.begin(), points_marker);
    pub_pseudo_slam_markers.publish(marker_array);
}

std::unordered_set<int> collect_global_indices_from_point_json_array(const nlohmann::json& points_json)
{
    std::unordered_set<int> global_indices;
    if (!points_json.is_array()) {
        return global_indices;
    }

    for (const auto& point_json : points_json) {
        const int global_idx = point_json.value(
            "global_idx",
            point_json.value("idx", -1)
        );
        if (global_idx > 0) {
            global_indices.insert(global_idx);
        }
    }
    return global_indices;
}

std::unordered_set<int> collect_global_indices_from_group_json(const nlohmann::json& group_json)
{
    if (group_json.is_array()) {
        return collect_global_indices_from_point_json_array(group_json);
    }
    if (!group_json.contains("points")) {
        return {};
    }
    return collect_global_indices_from_point_json_array(group_json["points"]);
}

std::unordered_set<int> collect_global_indices_from_area_json(const nlohmann::json& area_json)
{
    std::unordered_set<int> global_indices;
    if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
        return global_indices;
    }

    for (const auto& group_json : area_json["groups"]) {
        const auto group_global_indices = collect_global_indices_from_group_json(group_json);
        global_indices.insert(group_global_indices.begin(), group_global_indices.end());
    }
    return global_indices;
}

void set_pseudo_slam_marker_execution_state(
    int area_index,
    const std::unordered_set<int>& highlighted_area_global_indices,
    const std::unordered_set<int>& active_dispatch_global_indices
)
{
    std::vector<tie_robot_msgs::PointCoords> marker_points_snapshot;
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_execution_state.current_area_index = area_index;
        pseudo_slam_marker_execution_state.highlighted_area_global_indices =
            highlighted_area_global_indices;
        pseudo_slam_marker_execution_state.active_dispatch_global_indices =
            active_dispatch_global_indices;
        marker_points_snapshot = pseudo_slam_marker_points;
    }

    if (!marker_points_snapshot.empty()) {
        publish_pseudo_slam_markers(marker_points_snapshot);
    }
}

void set_pseudo_slam_marker_path_origin(const Cabin_Point& path_origin)
{
    std::lock_guard<std::mutex> lock(pseudo_slam_marker_path_origin_mutex);
    pseudo_slam_marker_path_origin = path_origin;
    pseudo_slam_marker_path_origin_valid = true;
}

bool recompute_pseudo_slam_marker_outlier_sets(
    const std::vector<tie_robot_msgs::PointCoords>& marker_points_snapshot,
    const Cabin_Point& path_origin,
    std::unordered_set<int>& outlier_global_indices,
    std::unordered_set<int>& outlier_secondary_plane_global_indices,
    std::unordered_set<int>& outlier_line_global_indices,
    std::unordered_set<int>& outlier_column_neighbor_global_indices)
{
    outlier_global_indices.clear();
    outlier_secondary_plane_global_indices.clear();
    outlier_line_global_indices.clear();
    outlier_column_neighbor_global_indices.clear();
    if (marker_points_snapshot.empty()) {
        return true;
    }

    std::unordered_map<int, PseudoSlamCheckerboardInfo> merged_checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(marker_points_snapshot, path_origin);
    const std::vector<tie_robot_msgs::PointCoords> planning_z_outlier_points =
        collect_pseudo_slam_planning_z_outliers(marker_points_snapshot);
    outlier_secondary_plane_global_indices =
        collect_pseudo_slam_outlier_secondary_plane_global_indices(planning_z_outlier_points);
    std::vector<tie_robot_msgs::PointCoords> secondary_plane_outlier_points;
    secondary_plane_outlier_points.reserve(planning_z_outlier_points.size());
    for (const auto& outlier_point : planning_z_outlier_points) {
        if (outlier_secondary_plane_global_indices.count(outlier_point.idx) > 0) {
            secondary_plane_outlier_points.push_back(outlier_point);
        }
    }
    outlier_line_global_indices =
        collect_pseudo_slam_outlier_line_global_indices(planning_z_outlier_points);
    std::vector<tie_robot_msgs::PointCoords> planning_world_points =
        filter_pseudo_slam_planning_outliers(marker_points_snapshot);
    planning_world_points = filter_pseudo_slam_points_near_outlier_secondary_plane_members(
        planning_world_points,
        secondary_plane_outlier_points
    );
    outlier_column_neighbor_global_indices =
        collect_pseudo_slam_outlier_column_neighbor_blocked_global_indices(
            planning_world_points,
            planning_z_outlier_points
        );
    planning_world_points = filter_pseudo_slam_points_near_outlier_columns(
        planning_world_points,
        planning_z_outlier_points
    );
    std::unordered_map<int, PseudoSlamCheckerboardInfo> checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    planning_world_points = filter_pseudo_slam_non_checkerboard_points(
        planning_world_points,
        checkerboard_info_by_idx
    );
    checkerboard_info_by_idx = build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    merged_checkerboard_info_by_idx = sync_merged_checkerboard_membership_with_planning(
        merged_checkerboard_info_by_idx,
        checkerboard_info_by_idx
    );

    for (const auto& world_point : marker_points_snapshot) {
        const auto merged_checkerboard_it = merged_checkerboard_info_by_idx.find(world_point.idx);
        if (merged_checkerboard_it == merged_checkerboard_info_by_idx.end() ||
            !merged_checkerboard_it->second.is_checkerboard_member) {
            if (outlier_column_neighbor_global_indices.count(world_point.idx) == 0) {
                outlier_global_indices.insert(world_point.idx);
            }
        }
    }
    return true;
}

bool refresh_pseudo_slam_marker_outlier_state_from_current_points(bool log_refresh)
{
    std::vector<tie_robot_msgs::PointCoords> marker_points_snapshot;
    Cabin_Point path_origin{};
    bool has_path_origin = false;
    {
        std::lock_guard<std::mutex> marker_lock(pseudo_slam_marker_state_mutex);
        marker_points_snapshot = pseudo_slam_marker_points;
    }
    {
        std::lock_guard<std::mutex> path_origin_lock(pseudo_slam_marker_path_origin_mutex);
        path_origin = pseudo_slam_marker_path_origin;
        has_path_origin = pseudo_slam_marker_path_origin_valid;
    }

    if (marker_points_snapshot.empty() || !has_path_origin) {
        return false;
    }

    std::unordered_set<int> refreshed_outlier_global_indices;
    std::unordered_set<int> refreshed_outlier_secondary_plane_global_indices;
    std::unordered_set<int> refreshed_outlier_line_global_indices;
    std::unordered_set<int> refreshed_outlier_column_neighbor_global_indices;
    recompute_pseudo_slam_marker_outlier_sets(
        marker_points_snapshot,
        path_origin,
        refreshed_outlier_global_indices,
        refreshed_outlier_secondary_plane_global_indices,
        refreshed_outlier_line_global_indices,
        refreshed_outlier_column_neighbor_global_indices
    );
    {
        std::lock_guard<std::mutex> marker_lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_outlier_global_indices = refreshed_outlier_global_indices;
        pseudo_slam_marker_outlier_secondary_plane_global_indices =
            refreshed_outlier_secondary_plane_global_indices;
        pseudo_slam_marker_outlier_line_global_indices = refreshed_outlier_line_global_indices;
        pseudo_slam_marker_outlier_column_neighbor_global_indices =
            refreshed_outlier_column_neighbor_global_indices;
    }

    if (log_refresh) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: pseudo_slam离群阈值热更新后已重算Marker离群状态，当前离群点%d个，离群二次平面成员点%d个，离群线成员点%d个，列邻域屏蔽点%d个。\n",
            static_cast<int>(refreshed_outlier_global_indices.size()),
            static_cast<int>(refreshed_outlier_secondary_plane_global_indices.size()),
            static_cast<int>(refreshed_outlier_line_global_indices.size()),
            static_cast<int>(refreshed_outlier_column_neighbor_global_indices.size())
        );
    }
    publish_pseudo_slam_markers(marker_points_snapshot);
    return true;
}

void maybe_refresh_pseudo_slam_marker_outlier_threshold()
{
    const float current_threshold_mm = load_pseudo_slam_planning_z_outlier_threshold_mm();
    const float previous_threshold_mm =
        pseudo_slam_marker_last_outlier_threshold_mm.exchange(current_threshold_mm);
    const float current_secondary_plane_threshold_mm =
        load_pseudo_slam_outlier_secondary_plane_threshold_mm();
    const float previous_secondary_plane_threshold_mm =
        pseudo_slam_marker_last_outlier_secondary_plane_threshold_mm.exchange(
            current_secondary_plane_threshold_mm
        );
    const float current_secondary_plane_neighbor_tolerance_mm =
        load_pseudo_slam_outlier_secondary_plane_neighbor_tolerance_mm();
    const float previous_secondary_plane_neighbor_tolerance_mm =
        pseudo_slam_marker_last_outlier_secondary_plane_neighbor_tolerance_mm.exchange(
            current_secondary_plane_neighbor_tolerance_mm
        );
    if (std::fabs(current_threshold_mm - previous_threshold_mm) < 1e-3f &&
        std::fabs(current_secondary_plane_threshold_mm - previous_secondary_plane_threshold_mm) < 1e-3f &&
        std::fabs(current_secondary_plane_neighbor_tolerance_mm -
                  previous_secondary_plane_neighbor_tolerance_mm) < 1e-3f) {
        return;
    }

    printCurrentTime();
    ros_log_printf(
        "Cabin_log: pseudo_slam离群阈值热更新：主平面%.2fmm -> %.2fmm，离群二次平面%.2fmm -> %.2fmm，二次平面邻域xy±%.2fmm -> ±%.2fmm，开始刷新RViz离群点显示。\n",
        previous_threshold_mm,
        current_threshold_mm,
        previous_secondary_plane_threshold_mm,
        current_secondary_plane_threshold_mm,
        previous_secondary_plane_neighbor_tolerance_mm,
        current_secondary_plane_neighbor_tolerance_mm
    );
    refresh_pseudo_slam_marker_outlier_state_from_current_points(true);
}

void clear_pseudo_slam_marker_execution_state()
{
    std::vector<tie_robot_msgs::PointCoords> marker_points_snapshot;
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_execution_state = PseudoSlamMarkerExecutionState{};
        marker_points_snapshot = pseudo_slam_marker_points;
    }

    if (!marker_points_snapshot.empty()) {
        publish_pseudo_slam_markers(marker_points_snapshot);
    }
}

bool load_pseudo_slam_marker_points_from_json(
    std::vector<tie_robot_msgs::PointCoords>& restored_marker_points,
    std::unordered_set<int>& restored_outlier_global_indices,
    std::unordered_set<int>& restored_outlier_secondary_plane_global_indices,
    std::unordered_set<int>& restored_outlier_line_global_indices,
    std::unordered_set<int>& restored_outlier_column_neighbor_global_indices,
    std::string& error_message
)
{
    restored_marker_points.clear();
    restored_outlier_global_indices.clear();
    restored_outlier_secondary_plane_global_indices.clear();
    restored_outlier_line_global_indices.clear();
    restored_outlier_column_neighbor_global_indices.clear();
    error_message.clear();

    nlohmann::json points_json;
    std::string points_error;
    if (!load_scan_artifact_json(
            pseudo_slam_points_json_file,
            "pseudo_slam_points.json",
            points_json,
            points_error
        )) {
        error_message = points_error;
        return false;
    }

    try {
        if (!points_json.contains("pseudo_slam_points") ||
            !points_json["pseudo_slam_points"].is_array()) {
            error_message = "pseudo_slam_points.json格式错误";
            return false;
        }

        int fallback_idx = 1;
        for (const auto& point_json : points_json["pseudo_slam_points"]) {
            tie_robot_msgs::PointCoords point;
            point.idx = point_json.value(
                "global_idx",
                point_json.value("idx", fallback_idx)
            );
            point.Pix_coord[0] = 0;
            point.Pix_coord[1] = 0;
            point.World_coord[0] = point_json.value("world_x", point_json.value("x", 0.0f));
            point.World_coord[1] = point_json.value("world_y", point_json.value("y", 0.0f));
            point.World_coord[2] = point_json.value("world_z", point_json.value("z", 0.0f));
            point.Angle = point_json.value("angle", -45.0f);
            point.is_shuiguan = false;
            restored_marker_points.push_back(point);
            if (point_json.value("is_planning_outlier", false)) {
                restored_outlier_global_indices.insert(point.idx);
            }
            if (point_json.value("is_outlier_secondary_plane_member", false)) {
                restored_outlier_secondary_plane_global_indices.insert(point.idx);
            }
            if (point_json.value("is_planning_outlier_line_member", false)) {
                restored_outlier_line_global_indices.insert(point.idx);
            }
            if (point_json.value("is_outlier_column_neighbor_blocked", false)) {
                restored_outlier_column_neighbor_global_indices.insert(point.idx);
            }
            fallback_idx++;
        }
    } catch (const std::exception&) {
        restored_marker_points.clear();
        restored_outlier_global_indices.clear();
        restored_outlier_secondary_plane_global_indices.clear();
        restored_outlier_line_global_indices.clear();
        restored_outlier_column_neighbor_global_indices.clear();
        error_message = "pseudo_slam_points.json读取失败";
        return false;
    }

    return true;
}

bool load_pseudo_slam_marker_path_origin_from_bind_path_json(
    Cabin_Point& restored_path_origin,
    std::string& error_message)
{
    restored_path_origin = Cabin_Point{};
    error_message.clear();

    std::ifstream bind_path_file(pseudo_slam_bind_path_json_file);
    if (!bind_path_file.is_open()) {
        error_message = "pseudo_slam_bind_path.json不存在";
        return false;
    }

    try {
        nlohmann::json bind_path_json;
        bind_path_file >> bind_path_json;
        if (!bind_path_json.contains("path_origin") || !bind_path_json["path_origin"].is_object()) {
            error_message = "pseudo_slam_bind_path.json缺少path_origin";
            return false;
        }
        restored_path_origin.x = bind_path_json["path_origin"].value("x", 0.0f);
        restored_path_origin.y = bind_path_json["path_origin"].value("y", 0.0f);
    } catch (const std::exception&) {
        error_message = "pseudo_slam_bind_path.json读取失败";
        return false;
    }

    return true;
}

void restore_pseudo_slam_markers_from_json_on_startup()
{
    std::vector<tie_robot_msgs::PointCoords> restored_marker_points;
    std::unordered_set<int> restored_outlier_global_indices;
    std::unordered_set<int> restored_outlier_secondary_plane_global_indices;
    std::unordered_set<int> restored_outlier_line_global_indices;
    std::unordered_set<int> restored_outlier_column_neighbor_global_indices;
    Cabin_Point restored_path_origin{};
    std::string restore_error;
    if (!load_pseudo_slam_marker_points_from_json(
            restored_marker_points,
            restored_outlier_global_indices,
            restored_outlier_secondary_plane_global_indices,
            restored_outlier_line_global_indices,
            restored_outlier_column_neighbor_global_indices,
            restore_error
        )) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_Warn: 启动时恢复/cabin/pseudo_slam_markers失败：%s\n",
            restore_error.c_str()
        );
        return;
    }

    if (restored_marker_points.empty()) {
        printCurrentTime();
        ros_log_printf("Cabin_log: pseudo_slam_points.json存在，但没有可恢复的历史扫描点。\n");
        return;
    }
    if (!load_pseudo_slam_marker_path_origin_from_bind_path_json(restored_path_origin, restore_error)) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_Warn: 启动时恢复/cabin/pseudo_slam_markers缺少path_origin：%s\n",
            restore_error.c_str()
        );
    } else {
        set_pseudo_slam_marker_path_origin(restored_path_origin);
    }

    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_outlier_global_indices = restored_outlier_global_indices;
        pseudo_slam_marker_outlier_secondary_plane_global_indices =
            restored_outlier_secondary_plane_global_indices;
        pseudo_slam_marker_outlier_line_global_indices = restored_outlier_line_global_indices;
        pseudo_slam_marker_outlier_column_neighbor_global_indices =
            restored_outlier_column_neighbor_global_indices;
    }
    publish_pseudo_slam_markers(restored_marker_points);
    refresh_pseudo_slam_marker_outlier_state_from_current_points(false);
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 已从pseudo_slam_points.json恢复%d个历史扫描点到/cabin/pseudo_slam_markers。\n",
        static_cast<int>(restored_marker_points.size())
    );
}
