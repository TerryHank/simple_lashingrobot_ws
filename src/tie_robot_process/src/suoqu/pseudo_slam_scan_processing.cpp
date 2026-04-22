#include "suoqu_runtime_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>

double point_distance_mm(const tie_robot_msgs::PointCoords& lhs, const tie_robot_msgs::PointCoords& rhs)
{
    const double dx = static_cast<double>(lhs.World_coord[0]) - static_cast<double>(rhs.World_coord[0]);
    const double dy = static_cast<double>(lhs.World_coord[1]) - static_cast<double>(rhs.World_coord[1]);
    const double dz = static_cast<double>(lhs.World_coord[2]) - static_cast<double>(rhs.World_coord[2]);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool transform_cabin_world_point_to_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    tie_robot_msgs::PointCoords& gripper_point
)
{
    tf2::Transform gripper_from_scepter;
    if (!lookup_gripper_from_scepter_transform(gripper_from_scepter)) {
        return false;
    }

    Cabin_State current_cabin_state{};
    {
        std::lock_guard<std::mutex> lock(cabin_state_mutex);
        current_cabin_state = cabin_state;
    }

    const tf2::Vector3 point_in_scepter_frame(
        static_cast<double>(world_point.World_coord[0] - current_cabin_state.X) / 1000.0,
        static_cast<double>(world_point.World_coord[1] - current_cabin_state.Y) / 1000.0,
        static_cast<double>(current_cabin_state.Z - world_point.World_coord[2]) / 1000.0 -
            gripper_from_scepter.getOrigin().z()
    );
    const tf2::Vector3 point_in_gripper_frame = gripper_from_scepter * point_in_scepter_frame;

    gripper_point = world_point;
    gripper_point.World_coord[0] = static_cast<float>(point_in_gripper_frame.x() * 1000.0);
    gripper_point.World_coord[1] = static_cast<float>(point_in_gripper_frame.y() * 1000.0);
    gripper_point.World_coord[2] = static_cast<float>(point_in_gripper_frame.z() * 1000.0);
    return true;
}

bool is_local_bind_point_in_range(const tie_robot_msgs::PointCoords& point)
{
    return point.World_coord[0] >= 0.0f &&
           point.World_coord[0] <= kTravelMaxXMm &&
           point.World_coord[1] >= 0.0f &&
           point.World_coord[1] <= kTravelMaxYMm &&
           point.World_coord[2] >= 0.0f &&
           point.World_coord[2] <= kTravelMaxZMm;
}

std::vector<tie_robot_msgs::PointCoords> dedupe_world_points(
    const std::vector<tie_robot_msgs::PointCoords>& input_points
)
{
    std::vector<tie_robot_msgs::PointCoords> deduped_points;
    for (const auto& point : input_points) {
        bool is_duplicate = false;
        for (const auto& existing_point : deduped_points) {
            if (point_distance_mm(point, existing_point) < kPseudoSlamDedupDistanceMm) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            deduped_points.push_back(point);
        }
    }
    return deduped_points;
}

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_close_xy_point_clusters(
    const std::vector<tie_robot_msgs::PointCoords>& input_points
)
{
    if (input_points.empty()) {
        return {};
    }

    std::unordered_set<size_t> rejected_indexes;
    for (size_t candidate_index = 0; candidate_index < input_points.size(); ++candidate_index) {
        const auto& candidate_point = input_points[candidate_index];
        for (size_t other_index = candidate_index + 1; other_index < input_points.size(); ++other_index) {
            const auto& other_point = input_points[other_index];
            if (std::fabs(candidate_point.World_coord[0] - other_point.World_coord[0]) <
                    kPseudoSlamClosePointClusterXYToleranceMm &&
                std::fabs(candidate_point.World_coord[1] - other_point.World_coord[1]) <
                    kPseudoSlamClosePointClusterXYToleranceMm) {
                rejected_indexes.insert(candidate_index);
                rejected_indexes.insert(other_index);
            }
        }
    }

    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(input_points.size());
    for (size_t candidate_index = 0; candidate_index < input_points.size(); ++candidate_index) {
        if (rejected_indexes.count(candidate_index) == 0) {
            filtered_points.push_back(input_points[candidate_index]);
        }
    }
    return filtered_points;
}

std::vector<tie_robot_msgs::PointCoords> filter_new_scan_points_against_existing_xy_tolerance(
    const std::vector<tie_robot_msgs::PointCoords>& candidate_points,
    const std::vector<tie_robot_msgs::PointCoords>& existing_points,
    float tolerance_mm
)
{
    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(candidate_points.size());
    const double tolerance_sq = static_cast<double>(tolerance_mm) * static_cast<double>(tolerance_mm);

    std::vector<tie_robot_msgs::PointCoords> accepted_history_points = existing_points;
    accepted_history_points.reserve(existing_points.size() + candidate_points.size());
    for (const auto& candidate_point : candidate_points) {
        bool is_duplicate_scan_point = false;
        for (const auto& existing_point : accepted_history_points) {
            const double dx =
                static_cast<double>(candidate_point.World_coord[0]) -
                static_cast<double>(existing_point.World_coord[0]);
            const double dy =
                static_cast<double>(candidate_point.World_coord[1]) -
                static_cast<double>(existing_point.World_coord[1]);
            if (dx * dx + dy * dy <= tolerance_sq) {
                is_duplicate_scan_point = true;
                break;
            }
        }
        if (!is_duplicate_scan_point) {
            filtered_points.push_back(candidate_point);
            accepted_history_points.push_back(candidate_point);
        }
    }
    return filtered_points;
}

void add_point_to_overlap_cluster(
    PseudoSlamOverlapCluster& cluster,
    const tie_robot_msgs::PointCoords& point)
{
    cluster.member_points.push_back(point);
    cluster.sum_x_mm += static_cast<double>(point.World_coord[0]);
    cluster.sum_y_mm += static_cast<double>(point.World_coord[1]);
}

double compute_overlap_cluster_center_x(const PseudoSlamOverlapCluster& cluster)
{
    if (cluster.member_points.empty()) {
        return 0.0;
    }
    return cluster.sum_x_mm / static_cast<double>(cluster.member_points.size());
}

double compute_overlap_cluster_center_y(const PseudoSlamOverlapCluster& cluster)
{
    if (cluster.member_points.empty()) {
        return 0.0;
    }
    return cluster.sum_y_mm / static_cast<double>(cluster.member_points.size());
}

tie_robot_msgs::PointCoords select_overlap_cluster_representative(
    const PseudoSlamOverlapCluster& cluster)
{
    if (cluster.member_points.empty()) {
        return tie_robot_msgs::PointCoords();
    }

    const double cluster_center_x = compute_overlap_cluster_center_x(cluster);
    const double cluster_center_y = compute_overlap_cluster_center_y(cluster);
    size_t best_member_index = 0;
    double best_member_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t member_index = 0; member_index < cluster.member_points.size(); ++member_index) {
        const auto& member_point = cluster.member_points[member_index];
        const double dx = static_cast<double>(member_point.World_coord[0]) - cluster_center_x;
        const double dy = static_cast<double>(member_point.World_coord[1]) - cluster_center_y;
        const double member_distance_sq = dx * dx + dy * dy;
        if (member_distance_sq < best_member_distance_sq) {
            best_member_distance_sq = member_distance_sq;
            best_member_index = member_index;
        }
    }
    return cluster.member_points[best_member_index];
}

std::vector<tie_robot_msgs::PointCoords> build_scan_cluster_representatives(
    const std::vector<PseudoSlamOverlapCluster>& clusters)
{
    std::vector<tie_robot_msgs::PointCoords> representative_points;
    representative_points.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        if (!cluster.member_points.empty()) {
            representative_points.push_back(select_overlap_cluster_representative(cluster));
        }
    }
    return representative_points;
}

void merge_frame_points_into_overlap_clusters(
    const std::vector<tie_robot_msgs::PointCoords>& frame_world_points,
    int scan_pose_index,
    std::vector<PseudoSlamOverlapCluster>& clusters,
    float tolerance_mm)
{
    (void)scan_pose_index;
    const size_t cluster_count_before_frame = clusters.size();
    const double tolerance_sq = static_cast<double>(tolerance_mm) * static_cast<double>(tolerance_mm);
    std::unordered_map<size_t, std::vector<tie_robot_msgs::PointCoords>> matched_points_by_cluster;
    std::vector<tie_robot_msgs::PointCoords> unmatched_points;
    unmatched_points.reserve(frame_world_points.size());

    for (const auto& candidate_point : frame_world_points) {
        bool matched_cluster = false;
        size_t matched_cluster_index = 0;
        double best_cluster_distance_sq = std::numeric_limits<double>::infinity();
        for (size_t cluster_index = 0; cluster_index < cluster_count_before_frame; ++cluster_index) {
            const double cluster_center_x = compute_overlap_cluster_center_x(clusters[cluster_index]);
            const double cluster_center_y = compute_overlap_cluster_center_y(clusters[cluster_index]);
            const double dx =
                static_cast<double>(candidate_point.World_coord[0]) - cluster_center_x;
            const double dy =
                static_cast<double>(candidate_point.World_coord[1]) - cluster_center_y;
            const double cluster_distance_sq = dx * dx + dy * dy;
            if (cluster_distance_sq <= tolerance_sq &&
                cluster_distance_sq < best_cluster_distance_sq) {
                matched_cluster = true;
                matched_cluster_index = cluster_index;
                best_cluster_distance_sq = cluster_distance_sq;
            }
        }
        if (matched_cluster) {
            matched_points_by_cluster[matched_cluster_index].push_back(candidate_point);
        } else {
            unmatched_points.push_back(candidate_point);
        }
    }

    for (const auto& matched_entry : matched_points_by_cluster) {
        auto& cluster = clusters[matched_entry.first];
        for (const auto& matched_point : matched_entry.second) {
            add_point_to_overlap_cluster(cluster, matched_point);
        }
    }

    for (const auto& unmatched_point : unmatched_points) {
        PseudoSlamOverlapCluster new_cluster;
        add_point_to_overlap_cluster(new_cluster, unmatched_point);
        clusters.push_back(std::move(new_cluster));
    }
}

float median_world_z_mm(const std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    if (world_points.empty()) {
        return 0.0f;
    }

    std::vector<float> z_values;
    z_values.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        z_values.push_back(world_point.World_coord[2]);
    }

    std::sort(z_values.begin(), z_values.end());
    const size_t middle_index = z_values.size() / 2;
    if (z_values.size() % 2 == 1) {
        return z_values[middle_index];
    }
    return (z_values[middle_index - 1] + z_values[middle_index]) * 0.5f;
}

struct PseudoSlamPlaneModel
{
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    bool used_horizontal_fallback = false;
    bool used_ransac = false;
    int inlier_count = 0;
};

float compute_pseudo_slam_plane_z_residual_mm(
    const tie_robot_msgs::PointCoords& world_point,
    const PseudoSlamPlaneModel& plane_model)
{
    return world_point.World_coord[2] -
           (plane_model.a * world_point.World_coord[0] +
            plane_model.b * world_point.World_coord[1] +
            plane_model.c);
}

PseudoSlamPlaneModel fit_pseudo_slam_plane_least_squares(
    const std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    PseudoSlamPlaneModel plane_model;
    if (world_points.empty()) {
        return plane_model;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xz = 0.0;
    double sum_yz = 0.0;

    for (const auto& world_point : world_points) {
        const double x = world_point.World_coord[0];
        const double y = world_point.World_coord[1];
        const double z = world_point.World_coord[2];
        sum_x += x;
        sum_y += y;
        sum_z += z;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        sum_xz += x * z;
        sum_yz += y * z;
    }

    const double n = static_cast<double>(world_points.size());
    const double matrix[3][4] = {
        {sum_xx, sum_xy, sum_x, sum_xz},
        {sum_xy, sum_yy, sum_y, sum_yz},
        {sum_x,  sum_y,  n,     sum_z },
    };
    double reduced_matrix[3][4];
    std::memcpy(reduced_matrix, matrix, sizeof(matrix));

    for (int pivot_index = 0; pivot_index < 3; ++pivot_index) {
        int best_row = pivot_index;
        for (int row = pivot_index + 1; row < 3; ++row) {
            if (std::fabs(reduced_matrix[row][pivot_index]) >
                std::fabs(reduced_matrix[best_row][pivot_index])) {
                best_row = row;
            }
        }
        if (std::fabs(reduced_matrix[best_row][pivot_index]) < 1e-6) {
            plane_model.used_horizontal_fallback = true;
            plane_model.c = static_cast<float>(sum_z / n);
            return plane_model;
        }
        if (best_row != pivot_index) {
            for (int column = 0; column < 4; ++column) {
                std::swap(reduced_matrix[pivot_index][column], reduced_matrix[best_row][column]);
            }
        }

        const double pivot_value = reduced_matrix[pivot_index][pivot_index];
        for (int column = pivot_index; column < 4; ++column) {
            reduced_matrix[pivot_index][column] /= pivot_value;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == pivot_index) {
                continue;
            }
            const double factor = reduced_matrix[row][pivot_index];
            if (std::fabs(factor) < 1e-9) {
                continue;
            }
            for (int column = pivot_index; column < 4; ++column) {
                reduced_matrix[row][column] -= factor * reduced_matrix[pivot_index][column];
            }
        }
    }

    plane_model.a = static_cast<float>(reduced_matrix[0][3]);
    plane_model.b = static_cast<float>(reduced_matrix[1][3]);
    plane_model.c = static_cast<float>(reduced_matrix[2][3]);
    plane_model.inlier_count = static_cast<int>(world_points.size());
    return plane_model;
}

bool solve_pseudo_slam_plane_from_three_points(
    const tie_robot_msgs::PointCoords& point_a,
    const tie_robot_msgs::PointCoords& point_b,
    const tie_robot_msgs::PointCoords& point_c,
    PseudoSlamPlaneModel& plane_model)
{
    const double matrix[3][4] = {
        {point_a.World_coord[0], point_a.World_coord[1], 1.0, point_a.World_coord[2]},
        {point_b.World_coord[0], point_b.World_coord[1], 1.0, point_b.World_coord[2]},
        {point_c.World_coord[0], point_c.World_coord[1], 1.0, point_c.World_coord[2]},
    };
    double reduced_matrix[3][4];
    std::memcpy(reduced_matrix, matrix, sizeof(matrix));

    for (int pivot_index = 0; pivot_index < 3; ++pivot_index) {
        int best_row = pivot_index;
        for (int row = pivot_index + 1; row < 3; ++row) {
            if (std::fabs(reduced_matrix[row][pivot_index]) >
                std::fabs(reduced_matrix[best_row][pivot_index])) {
                best_row = row;
            }
        }
        if (std::fabs(reduced_matrix[best_row][pivot_index]) < 1e-6) {
            return false;
        }
        if (best_row != pivot_index) {
            for (int column = 0; column < 4; ++column) {
                std::swap(reduced_matrix[pivot_index][column], reduced_matrix[best_row][column]);
            }
        }

        const double pivot_value = reduced_matrix[pivot_index][pivot_index];
        for (int column = pivot_index; column < 4; ++column) {
            reduced_matrix[pivot_index][column] /= pivot_value;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == pivot_index) {
                continue;
            }
            const double factor = reduced_matrix[row][pivot_index];
            if (std::fabs(factor) < 1e-9) {
                continue;
            }
            for (int column = pivot_index; column < 4; ++column) {
                reduced_matrix[row][column] -= factor * reduced_matrix[pivot_index][column];
            }
        }
    }

    plane_model.a = static_cast<float>(reduced_matrix[0][3]);
    plane_model.b = static_cast<float>(reduced_matrix[1][3]);
    plane_model.c = static_cast<float>(reduced_matrix[2][3]);
    plane_model.used_horizontal_fallback = false;
    return true;
}

PseudoSlamPlaneModel fit_pseudo_slam_plane(
    const std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    if (world_points.size() < 3) {
        return fit_pseudo_slam_plane_least_squares(world_points);
    }

    const float outlier_threshold_mm = load_pseudo_slam_planning_z_outlier_threshold_mm();
    const float ransac_inlier_threshold_mm = std::max(outlier_threshold_mm * 2.0f, 12.0f);
    const int max_iterations = std::min(
        120,
        std::max(30, static_cast<int>(world_points.size()) * 2)
    );

    std::mt19937 rng(20260418u + static_cast<unsigned int>(world_points.size()));
    std::uniform_int_distribution<int> index_distribution(
        0,
        static_cast<int>(world_points.size()) - 1
    );

    int best_inlier_count = -1;
    double best_residual_sum = std::numeric_limits<double>::max();
    PseudoSlamPlaneModel best_plane_model;
    std::vector<int> best_inlier_indices;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        const int index_a = index_distribution(rng);
        int index_b = index_distribution(rng);
        int index_c = index_distribution(rng);
        if (index_b == index_a) {
            index_b = (index_b + 1) % static_cast<int>(world_points.size());
        }
        if (index_c == index_a || index_c == index_b) {
            index_c = (index_c + 2) % static_cast<int>(world_points.size());
        }
        if (index_a == index_b || index_a == index_c || index_b == index_c) {
            continue;
        }

        PseudoSlamPlaneModel candidate_plane_model;
        if (!solve_pseudo_slam_plane_from_three_points(
                world_points[static_cast<size_t>(index_a)],
                world_points[static_cast<size_t>(index_b)],
                world_points[static_cast<size_t>(index_c)],
                candidate_plane_model
            )) {
            continue;
        }

        std::vector<int> candidate_inlier_indices;
        double candidate_residual_sum = 0.0;
        for (size_t point_index = 0; point_index < world_points.size(); ++point_index) {
            const float residual_mm = compute_pseudo_slam_plane_z_residual_mm(
                world_points[point_index],
                candidate_plane_model
            );
            if (std::fabs(residual_mm) <= ransac_inlier_threshold_mm) {
                candidate_inlier_indices.push_back(static_cast<int>(point_index));
                candidate_residual_sum += std::fabs(residual_mm);
            }
        }

        if (static_cast<int>(candidate_inlier_indices.size()) > best_inlier_count ||
            (static_cast<int>(candidate_inlier_indices.size()) == best_inlier_count &&
             candidate_residual_sum < best_residual_sum)) {
            best_inlier_count = static_cast<int>(candidate_inlier_indices.size());
            best_residual_sum = candidate_residual_sum;
            best_plane_model = candidate_plane_model;
            best_inlier_indices = std::move(candidate_inlier_indices);
        }
    }

    if (best_inlier_count < 3 || best_inlier_indices.empty()) {
        return fit_pseudo_slam_plane_least_squares(world_points);
    }

    std::vector<tie_robot_msgs::PointCoords> inlier_points;
    inlier_points.reserve(best_inlier_indices.size());
    for (const int point_index : best_inlier_indices) {
        inlier_points.push_back(world_points[static_cast<size_t>(point_index)]);
    }

    PseudoSlamPlaneModel fitted_plane_model = fit_pseudo_slam_plane_least_squares(inlier_points);
    fitted_plane_model.used_ransac = true;
    fitted_plane_model.inlier_count = static_cast<int>(inlier_points.size());
    return fitted_plane_model;
}

bool compute_pseudo_slam_global_scan_pose(
    const std::vector<Cabin_Point>& con_path,
    float planning_cabin_height,
    Cabin_Point& scan_center,
    float& scan_height,
    std::string& error_message)
{
    scan_center = Cabin_Point{};
    scan_height = planning_cabin_height + kPseudoSlamGlobalScanHeightOffsetMm;
    error_message.clear();

    try {
        std::ifstream infile(path_points_json_file);
        if (infile.is_open()) {
            nlohmann::json path_json;
            infile >> path_json;
            const float marking_x = path_json.value("marking_x", 0.0f);
            const float marking_y = path_json.value("marking_y", 0.0f);
            const float zone_x = path_json.value("zone_x", 0.0f);
            const float zone_y = path_json.value("zone_y", 0.0f);
            const float robot_x_step = path_json.value("robot_x_step", 0.0f);
            const float robot_y_step = path_json.value("robot_y_step", 0.0f);
            if (robot_x_step > 0.0f && robot_y_step > 0.0f && zone_x > 0.0f && zone_y > 0.0f) {
                const float workspace_min_x = marking_x - robot_x_step / 2.0f;
                const float workspace_min_y = marking_y - robot_y_step / 2.0f;
                scan_center.x = workspace_min_x + zone_x / 2.0f;
                scan_center.y = workspace_min_y + zone_y / 2.0f;
                return true;
            }
        }
    } catch (const std::exception& ex) {
        printCurrentTime();
        printf(
            "Cabin_Warn: 读取path_points.json工作区几何失败，将退回到路径边界中心扫描：%s\n",
            ex.what()
        );
    }

    if (con_path.empty()) {
        error_message = "没有可执行区域，无法计算全局扫描中心";
        return false;
    }

    float min_x = con_path.front().x;
    float max_x = con_path.front().x;
    float min_y = con_path.front().y;
    float max_y = con_path.front().y;
    for (const auto& cabin_point : con_path) {
        min_x = std::min(min_x, cabin_point.x);
        max_x = std::max(max_x, cabin_point.x);
        min_y = std::min(min_y, cabin_point.y);
        max_y = std::max(max_y, cabin_point.y);
    }
    scan_center.x = (min_x + max_x) / 2.0f;
    scan_center.y = (min_y + max_y) / 2.0f;

    printCurrentTime();
    printf(
        "Cabin_Warn: path_points.json缺少完整工作区几何，已退回到路径边界中心(%f,%f)执行单次全局扫描。\n",
        scan_center.x,
        scan_center.y
    );
    return true;
}

std::vector<float> cluster_checkerboard_axis_centers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points,
    int axis_index,
    float threshold_mm
)
{
    if (world_points.empty()) {
        return {};
    }

    std::vector<float> axis_values;
    axis_values.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        axis_values.push_back(world_point.World_coord[axis_index]);
    }
    std::sort(axis_values.begin(), axis_values.end());

    std::vector<float> centers;
    std::vector<float> current_group = {axis_values.front()};
    double current_mean = static_cast<double>(axis_values.front());
    for (size_t value_index = 1; value_index < axis_values.size(); ++value_index) {
        const double axis_value = static_cast<double>(axis_values[value_index]);
        if (std::fabs(axis_value - current_mean) <= threshold_mm) {
            current_group.push_back(axis_values[value_index]);
            double axis_sum = 0.0;
            for (const float grouped_value : current_group) {
                axis_sum += grouped_value;
            }
            current_mean = axis_sum / static_cast<double>(current_group.size());
        } else {
            centers.push_back(static_cast<float>(current_mean));
            current_group = {axis_values[value_index]};
            current_mean = axis_value;
        }
    }
    if (!current_group.empty()) {
        centers.push_back(static_cast<float>(current_mean));
    }
    return centers;
}

int find_nearest_checkerboard_center_index(float axis_value, const std::vector<float>& centers)
{
    if (centers.empty()) {
        return -1;
    }

    int best_index = 0;
    float best_gap = std::fabs(axis_value - centers.front());
    for (size_t center_index = 1; center_index < centers.size(); ++center_index) {
        const float gap = std::fabs(axis_value - centers[center_index]);
        if (gap < best_gap) {
            best_gap = gap;
            best_index = static_cast<int>(center_index);
        }
    }
    return best_index;
}

int find_nearest_checkerboard_center_key(
    float axis_value,
    const std::unordered_map<int, float>& centers_by_key
)
{
    if (centers_by_key.empty()) {
        return -1;
    }

    int best_key = -1;
    float best_gap = 0.0f;
    bool has_best_key = false;
    for (const auto& entry : centers_by_key) {
        const float gap = std::fabs(axis_value - entry.second);
        if (!has_best_key || gap < best_gap) {
            best_key = entry.first;
            best_gap = gap;
            has_best_key = true;
        }
    }
    return best_key;
}

long long encode_checkerboard_cell_key(int global_row, int global_col)
{
    return (static_cast<long long>(global_row) << 32) ^
           static_cast<unsigned int>(global_col);
}

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_planning_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
)
{
    if (world_points.empty()) {
        return {};
    }

    const PseudoSlamPlaneModel plane_model = fit_pseudo_slam_plane(world_points);
    const float outlier_threshold_mm = load_pseudo_slam_planning_z_outlier_threshold_mm();
    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(world_points.size());
    int removed_count = 0;
    for (const auto& world_point : world_points) {
        if (std::fabs(compute_pseudo_slam_plane_z_residual_mm(world_point, plane_model)) >
            outlier_threshold_mm) {
            removed_count++;
            continue;
        }
        filtered_points.push_back(world_point);
    }

    if (removed_count > 0) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam平面离群过滤：方式=%s，平面 z=%.6fx + %.6fy + %.3f，保留%d个点，过滤%d个点，阈值=±%.2fmm%s。\n",
            plane_model.used_ransac ? "RANSAC" : "最小二乘",
            plane_model.a,
            plane_model.b,
            plane_model.c,
            static_cast<int>(filtered_points.size()),
            removed_count,
            outlier_threshold_mm,
            plane_model.used_horizontal_fallback ? "，当前使用水平面退化拟合" : ""
        );
    }
    return filtered_points;
}

std::vector<tie_robot_msgs::PointCoords> collect_pseudo_slam_planning_z_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
)
{
    if (world_points.empty()) {
        return {};
    }

    const PseudoSlamPlaneModel plane_model = fit_pseudo_slam_plane(world_points);
    const float outlier_threshold_mm = load_pseudo_slam_planning_z_outlier_threshold_mm();
    std::vector<tie_robot_msgs::PointCoords> outlier_points;
    outlier_points.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        if (std::fabs(compute_pseudo_slam_plane_z_residual_mm(world_point, plane_model)) >
            outlier_threshold_mm) {
            outlier_points.push_back(world_point);
        }
    }
    return outlier_points;
}

std::unordered_set<int> collect_pseudo_slam_outlier_secondary_plane_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
)
{
    std::unordered_set<int> outlier_secondary_plane_global_indices;
    if (outlier_points.size() < static_cast<size_t>(kPseudoSlamOutlierSecondaryPlaneMinPointCount)) {
        return outlier_secondary_plane_global_indices;
    }

    const PseudoSlamPlaneModel secondary_plane_model = fit_pseudo_slam_plane(outlier_points);
    const float outlier_threshold_mm = load_pseudo_slam_outlier_secondary_plane_threshold_mm();
    for (const auto& outlier_point : outlier_points) {
        if (std::fabs(compute_pseudo_slam_plane_z_residual_mm(outlier_point, secondary_plane_model)) <=
            outlier_threshold_mm) {
            outlier_secondary_plane_global_indices.insert(outlier_point.idx);
        }
    }

    if (outlier_secondary_plane_global_indices.size() <
        static_cast<size_t>(kPseudoSlamOutlierSecondaryPlaneMinPointCount)) {
        outlier_secondary_plane_global_indices.clear();
        return outlier_secondary_plane_global_indices;
    }

    printCurrentTime();
    printf(
        "Cabin_log: pseudo_slam离群点二次平面拟合：方式=%s，平面 z=%.6fx + %.6fy + %.3f，二次平面成员点%d/%d，阈值=±%.2fmm%s。\n",
        secondary_plane_model.used_ransac ? "RANSAC" : "最小二乘",
        secondary_plane_model.a,
        secondary_plane_model.b,
        secondary_plane_model.c,
        static_cast<int>(outlier_secondary_plane_global_indices.size()),
        static_cast<int>(outlier_points.size()),
        outlier_threshold_mm,
        secondary_plane_model.used_horizontal_fallback ? "，当前使用水平面退化拟合" : ""
    );
    return outlier_secondary_plane_global_indices;
}

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_secondary_plane_members(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& secondary_plane_outlier_points
)
{
    if (planning_points.empty() || secondary_plane_outlier_points.empty()) {
        return planning_points;
    }
    const float neighbor_tolerance_mm = load_pseudo_slam_outlier_secondary_plane_neighbor_tolerance_mm();

    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(planning_points.size());
    int removed_point_count = 0;
    for (const auto& planning_point : planning_points) {
        bool should_remove = false;
        for (const auto& outlier_point : secondary_plane_outlier_points) {
            if (std::fabs(planning_point.World_coord[0] - outlier_point.World_coord[0]) <=
                    neighbor_tolerance_mm &&
                std::fabs(planning_point.World_coord[1] - outlier_point.World_coord[1]) <=
                    neighbor_tolerance_mm) {
                should_remove = true;
                break;
            }
        }

        if (should_remove) {
            removed_point_count++;
            continue;
        }
        filtered_points.push_back(planning_point);
    }

    if (removed_point_count > 0) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam离群二次平面成员附近xy±%.1fmm内正常点视为不可执行，本次排除%d个点。\n",
            neighbor_tolerance_mm,
            removed_point_count
        );
    }
    return filtered_points;
}

float compute_pseudo_slam_xy_point_to_line_distance_mm(
    const tie_robot_msgs::PointCoords& point,
    const tie_robot_msgs::PointCoords& line_point_a,
    const tie_robot_msgs::PointCoords& line_point_b)
{
    const double x0 = point.World_coord[0];
    const double y0 = point.World_coord[1];
    const double x1 = line_point_a.World_coord[0];
    const double y1 = line_point_a.World_coord[1];
    const double x2 = line_point_b.World_coord[0];
    const double y2 = line_point_b.World_coord[1];
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double denominator = std::sqrt(dx * dx + dy * dy);
    if (denominator < 1e-6) {
        return std::numeric_limits<float>::max();
    }
    const double numerator = std::fabs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1);
    return static_cast<float>(numerator / denominator);
}

std::unordered_set<int> collect_pseudo_slam_outlier_line_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
)
{
    std::unordered_set<int> outlier_line_global_indices;
    if (outlier_points.size() < static_cast<size_t>(kPseudoSlamOutlierLineMinPointCount)) {
        return outlier_line_global_indices;
    }

    std::vector<tie_robot_msgs::PointCoords> remaining_outlier_points = outlier_points;
    while (remaining_outlier_points.size() >= static_cast<size_t>(kPseudoSlamOutlierLineMinPointCount)) {
        int best_inlier_count = -1;
        double best_residual_sum = std::numeric_limits<double>::max();
        std::vector<int> best_inlier_positions;

        for (size_t point_a_index = 0; point_a_index < remaining_outlier_points.size(); ++point_a_index) {
            for (size_t point_b_index = point_a_index + 1; point_b_index < remaining_outlier_points.size(); ++point_b_index) {
                const float line_length_mm = std::hypot(
                    remaining_outlier_points[point_a_index].World_coord[0] -
                        remaining_outlier_points[point_b_index].World_coord[0],
                    remaining_outlier_points[point_a_index].World_coord[1] -
                        remaining_outlier_points[point_b_index].World_coord[1]
                );
                if (line_length_mm < 1e-3f) {
                    continue;
                }

                std::vector<int> candidate_inlier_positions;
                double residual_sum = 0.0;
                for (size_t point_index = 0; point_index < remaining_outlier_points.size(); ++point_index) {
                    const float line_distance_mm = compute_pseudo_slam_xy_point_to_line_distance_mm(
                        remaining_outlier_points[point_index],
                        remaining_outlier_points[point_a_index],
                        remaining_outlier_points[point_b_index]
                    );
                    if (line_distance_mm <= kPseudoSlamOutlierLineDistanceToleranceMm) {
                        candidate_inlier_positions.push_back(static_cast<int>(point_index));
                        residual_sum += line_distance_mm;
                    }
                }

                if (static_cast<int>(candidate_inlier_positions.size()) < kPseudoSlamOutlierLineMinPointCount) {
                    continue;
                }
                if (static_cast<int>(candidate_inlier_positions.size()) > best_inlier_count ||
                    (static_cast<int>(candidate_inlier_positions.size()) == best_inlier_count &&
                     residual_sum < best_residual_sum)) {
                    best_inlier_count = static_cast<int>(candidate_inlier_positions.size());
                    best_residual_sum = residual_sum;
                    best_inlier_positions = std::move(candidate_inlier_positions);
                }
            }
        }

        if (best_inlier_count < kPseudoSlamOutlierLineMinPointCount || best_inlier_positions.empty()) {
            break;
        }

        std::unordered_set<int> best_inlier_position_set(
            best_inlier_positions.begin(),
            best_inlier_positions.end()
        );
        std::vector<tie_robot_msgs::PointCoords> next_remaining_outlier_points;
        next_remaining_outlier_points.reserve(remaining_outlier_points.size());
        for (size_t point_index = 0; point_index < remaining_outlier_points.size(); ++point_index) {
            if (best_inlier_position_set.count(static_cast<int>(point_index)) > 0) {
                outlier_line_global_indices.insert(remaining_outlier_points[point_index].idx);
            } else {
                next_remaining_outlier_points.push_back(remaining_outlier_points[point_index]);
            }
        }
        remaining_outlier_points = std::move(next_remaining_outlier_points);
    }

    if (!outlier_line_global_indices.empty()) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam离群线拟合：在离群点中识别出线成员点%d个，使用专色显示。\n",
            static_cast<int>(outlier_line_global_indices.size())
        );
    }
    return outlier_line_global_indices;
}

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_columns(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
)
{
    if (planning_points.empty() || outlier_points.size() < 2) {
        return planning_points;
    }

    const std::vector<float> outlier_column_centers = cluster_checkerboard_axis_centers(
        outlier_points,
        0,
        kPseudoSlamOutlierColumnAxisToleranceMm
    );
    if (outlier_column_centers.empty()) {
        return planning_points;
    }

    std::unordered_map<int, std::vector<tie_robot_msgs::PointCoords>> outlier_points_by_column;
    for (const auto& outlier_point : outlier_points) {
        const int column_index = find_nearest_checkerboard_center_index(
            outlier_point.World_coord[0],
            outlier_column_centers
        );
        if (column_index < 0) {
            continue;
        }
        const float column_center = outlier_column_centers[static_cast<size_t>(column_index)];
        if (std::fabs(outlier_point.World_coord[0] - column_center) <=
            kPseudoSlamOutlierColumnAxisToleranceMm) {
            outlier_points_by_column[column_index].push_back(outlier_point);
        }
    }

    std::vector<tie_robot_msgs::PointCoords> qualified_outlier_points;
    for (const auto& entry : outlier_points_by_column) {
        if (entry.second.size() >= 2) {
            qualified_outlier_points.insert(
                qualified_outlier_points.end(),
                entry.second.begin(),
                entry.second.end()
            );
        }
    }
    if (qualified_outlier_points.empty()) {
        return planning_points;
    }

    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(planning_points.size());
    int removed_point_count = 0;
    for (const auto& planning_point : planning_points) {
        bool should_remove = false;
        for (const auto& outlier_point : qualified_outlier_points) {
            if (std::fabs(planning_point.World_coord[0] - outlier_point.World_coord[0]) <=
                    kPseudoSlamOutlierColumnPointToleranceMm &&
                std::fabs(planning_point.World_coord[1] - outlier_point.World_coord[1]) <=
                    kPseudoSlamOutlierColumnPointToleranceMm) {
                should_remove = true;
                break;
            }
        }

        if (should_remove) {
            removed_point_count++;
            continue;
        }
        filtered_points.push_back(planning_point);
    }

    if (removed_point_count > 0) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam离群列点位过滤：拟合成列的z离群点附近±10mm内正常点视为不可执行，移除%d个点，剩余%d个规划点。\n",
            removed_point_count,
            static_cast<int>(filtered_points.size())
        );
    }

    return filtered_points;
}

std::unordered_set<int> collect_pseudo_slam_outlier_column_neighbor_blocked_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
)
{
    std::unordered_set<int> blocked_global_indices;
    if (planning_points.empty() || outlier_points.size() < 2) {
        return blocked_global_indices;
    }

    const std::vector<float> outlier_column_centers = cluster_checkerboard_axis_centers(
        outlier_points,
        0,
        kPseudoSlamOutlierColumnAxisToleranceMm
    );
    if (outlier_column_centers.empty()) {
        return blocked_global_indices;
    }

    std::unordered_map<int, std::vector<tie_robot_msgs::PointCoords>> outlier_points_by_column;
    for (const auto& outlier_point : outlier_points) {
        const int column_index = find_nearest_checkerboard_center_index(
            outlier_point.World_coord[0],
            outlier_column_centers
        );
        if (column_index < 0) {
            continue;
        }
        const float column_center = outlier_column_centers[static_cast<size_t>(column_index)];
        if (std::fabs(outlier_point.World_coord[0] - column_center) <=
            kPseudoSlamOutlierColumnAxisToleranceMm) {
            outlier_points_by_column[column_index].push_back(outlier_point);
        }
    }

    std::vector<tie_robot_msgs::PointCoords> qualified_outlier_points;
    for (const auto& entry : outlier_points_by_column) {
        if (entry.second.size() >= 2) {
            qualified_outlier_points.insert(
                qualified_outlier_points.end(),
                entry.second.begin(),
                entry.second.end()
            );
        }
    }
    if (qualified_outlier_points.empty()) {
        return blocked_global_indices;
    }

    for (const auto& planning_point : planning_points) {
        for (const auto& outlier_point : qualified_outlier_points) {
            if (std::fabs(planning_point.World_coord[0] - outlier_point.World_coord[0]) <=
                    kPseudoSlamOutlierColumnPointToleranceMm &&
                std::fabs(planning_point.World_coord[1] - outlier_point.World_coord[1]) <=
                    kPseudoSlamOutlierColumnPointToleranceMm) {
                blocked_global_indices.insert(planning_point.idx);
                break;
            }
        }
    }

    return blocked_global_indices;
}

std::unordered_map<int, PseudoSlamCheckerboardInfo> build_checkerboard_info_by_global_index(
    const std::vector<tie_robot_msgs::PointCoords>& world_points,
    const Cabin_Point& path_origin
)
{
    std::unordered_map<int, PseudoSlamCheckerboardInfo> checkerboard_info_by_idx;
    if (world_points.empty()) {
        return checkerboard_info_by_idx;
    }

    const std::vector<float> x_centers = cluster_checkerboard_axis_centers(
        world_points,
        0,
        kPseudoSlamCheckerboardAxisThresholdMm
    );
    const std::vector<float> y_centers = cluster_checkerboard_axis_centers(
        world_points,
        1,
        kPseudoSlamCheckerboardAxisThresholdMm
    );
    if (x_centers.empty() || y_centers.empty()) {
        return checkerboard_info_by_idx;
    }

    std::unordered_set<long long> occupied_cells;
    int phase_reference = 0;
    double nearest_origin_distance_sq = std::numeric_limits<double>::max();
    for (const auto& world_point : world_points) {
        if (world_point.idx <= 0) {
            continue;
        }

        PseudoSlamCheckerboardInfo info;
        info.global_idx = world_point.idx;
        info.global_col = find_nearest_checkerboard_center_index(world_point.World_coord[0], x_centers);
        info.global_row = find_nearest_checkerboard_center_index(world_point.World_coord[1], y_centers);
        if (info.global_row < 0 || info.global_col < 0) {
            continue;
        }

        occupied_cells.insert(encode_checkerboard_cell_key(info.global_row, info.global_col));
        checkerboard_info_by_idx[world_point.idx] = info;
        const double distance_sq =
            static_cast<double>(world_point.World_coord[0] - path_origin.x) * static_cast<double>(world_point.World_coord[0] - path_origin.x) +
            static_cast<double>(world_point.World_coord[1] - path_origin.y) * static_cast<double>(world_point.World_coord[1] - path_origin.y);
        if (distance_sq < nearest_origin_distance_sq) {
            nearest_origin_distance_sq = distance_sq;
            phase_reference = (info.global_row + info.global_col) % 2;
        }
    }

    for (auto& entry : checkerboard_info_by_idx) {
        auto& info = entry.second;
        info.checkerboard_parity = (info.global_row + info.global_col + phase_reference) % 2;

        const bool has_horizontal_neighbor =
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row, info.global_col - 1)) > 0 ||
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row, info.global_col + 1)) > 0;
        const bool has_vertical_neighbor =
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row - 1, info.global_col)) > 0 ||
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row + 1, info.global_col)) > 0;
        const bool can_form_matrix = has_horizontal_neighbor && has_vertical_neighbor;
        const bool can_form_edge_pair = has_horizontal_neighbor || has_vertical_neighbor;
        info.is_checkerboard_member = can_form_matrix || can_form_edge_pair;
    }
    return checkerboard_info_by_idx;
}

std::unordered_map<int, PseudoSlamCheckerboardInfo> sync_merged_checkerboard_membership_with_planning(
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& merged_checkerboard_info_by_idx,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& planning_checkerboard_info_by_idx
)
{
    std::unordered_map<int, PseudoSlamCheckerboardInfo> synced_checkerboard_info_by_idx =
        merged_checkerboard_info_by_idx;
    for (auto& entry : synced_checkerboard_info_by_idx) {
        if (planning_checkerboard_info_by_idx.find(entry.first) !=
            planning_checkerboard_info_by_idx.end()) {
            continue;
        }
        auto& merged_info = entry.second;
        merged_info.is_checkerboard_member = false;
    }
    return synced_checkerboard_info_by_idx;
}

bool validate_scan_session_alignment(
    const nlohmann::json& artifact_json,
    const std::string& artifact_name,
    const BindExecutionMemory& bind_execution_memory,
    std::string& error_message
)
{
    error_message.clear();
    const std::string artifact_scan_session_id = artifact_json.value("scan_session_id", std::string());
    const bool scan_session_mismatch =
        artifact_scan_session_id != bind_execution_memory.scan_session_id;
    if (!artifact_scan_session_id.empty() &&
        !bind_execution_memory.scan_session_id.empty() &&
        !scan_session_mismatch) {
        return true;
    }

    std::ostringstream oss;
    oss << artifact_name << " 与 bind_execution_memory.json 的 scan_session_id不一致";
    if (artifact_scan_session_id.empty()) {
        oss << "（" << artifact_name << "缺少scan_session_id";
        if (bind_execution_memory.scan_session_id.empty()) {
            oss << "，账本也缺少scan_session_id";
        }
        oss << "）";
    } else if (bind_execution_memory.scan_session_id.empty()) {
        oss << "（bind_execution_memory.json缺少scan_session_id）";
    } else {
        oss << "（产物=" << artifact_scan_session_id
            << "，账本=" << bind_execution_memory.scan_session_id << "）";
    }
    oss << "，按fail-closed策略拒绝继续执行";
    error_message = oss.str();
    printCurrentTime();
    printf("Cabin_Warn: %s\n", error_message.c_str());
    return false;
}

bool validate_path_signature_alignment(
    const nlohmann::json& artifact_json,
    const std::string& artifact_name,
    const BindExecutionMemory& bind_execution_memory,
    const std::string& current_path_signature,
    std::string& error_message
)
{
    error_message.clear();
    const std::string artifact_path_signature = artifact_json.value("path_signature", std::string());
    const bool artifact_path_signature_missing = artifact_path_signature.empty();
    const bool ledger_path_signature_missing = bind_execution_memory.path_signature.empty();
    const bool artifact_path_signature_mismatch = artifact_path_signature != current_path_signature;
    const bool ledger_path_signature_mismatch = bind_execution_memory.path_signature != current_path_signature;
    if (!artifact_path_signature_missing &&
        !ledger_path_signature_missing &&
        !artifact_path_signature_mismatch &&
        !ledger_path_signature_mismatch) {
        return true;
    }

    std::ostringstream oss;
    oss << artifact_name << " / bind_execution_memory.json / 当前路径配置 的 path_signature不一致";
    if (artifact_path_signature.empty()) {
        oss << "（" << artifact_name << "缺少path_signature";
        if (bind_execution_memory.path_signature.empty()) {
            oss << "，账本也缺少path_signature";
        }
        oss << "）";
    } else if (bind_execution_memory.path_signature.empty()) {
        oss << "（bind_execution_memory.json缺少path_signature）";
    } else {
        oss << "（产物与当前路径是否一致="
            << (artifact_path_signature_mismatch ? "否" : "是")
            << "，账本与当前路径是否一致="
            << (ledger_path_signature_mismatch ? "否" : "是")
            << "）";
    }
    oss << "，说明扫描时使用的路径或关键运动参数已经变化，请先重新扫描后再执行";
    error_message = oss.str();
    printCurrentTime();
    printf("Cabin_Warn: %s\n", error_message.c_str());
    return false;
}

bool load_scan_artifact_json(
    const std::string& artifact_path,
    const std::string& artifact_name,
    nlohmann::json& artifact_json,
    std::string& error_message
)
{
    artifact_json = nlohmann::json();
    error_message.clear();

    std::ifstream infile(artifact_path);
    if (!infile.is_open()) {
        error_message = "未找到" + artifact_name + "，请先执行扫描建图";
        return false;
    }

    try {
        infile >> artifact_json;
        if (infile.fail() && !infile.eof()) {
            throw std::runtime_error("artifact read failure");
        }
        if (!artifact_json.is_object()) {
            throw std::runtime_error("artifact root must be object");
        }
    } catch (const std::exception&) {
        error_message = artifact_name + "读取失败";
        artifact_json = nlohmann::json();
        return false;
    }

    return true;
}

bool load_scan_artifacts_for_execution(
    nlohmann::json& points_json,
    nlohmann::json& bind_path_json,
    const BindExecutionMemory& bind_execution_memory,
    const std::string& current_path_signature,
    std::string& error_message
)
{
    error_message.clear();

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

    std::string bind_path_error;
    if (!load_scan_artifact_json(
            pseudo_slam_bind_path_json_file,
            "pseudo_slam_bind_path.json",
            bind_path_json,
            bind_path_error
        )) {
        error_message = bind_path_error;
        return false;
    }

    if (!validate_scan_session_alignment(
            points_json,
            "pseudo_slam_points.json",
            bind_execution_memory,
            error_message
        )) {
        return false;
    }

    if (!validate_scan_session_alignment(
            bind_path_json,
            "pseudo_slam_bind_path.json",
            bind_execution_memory,
            error_message
        )) {
        return false;
    }

    if (!validate_path_signature_alignment(
            points_json,
            "pseudo_slam_points.json",
            bind_execution_memory,
            current_path_signature,
            error_message
        )) {
        return false;
    }

    if (!validate_path_signature_alignment(
            bind_path_json,
            "pseudo_slam_bind_path.json",
            bind_execution_memory,
            current_path_signature,
            error_message
        )) {
        return false;
    }

    return true;
}

bool load_live_visual_checkerboard_grid(
    const nlohmann::json& points_json,
    LiveVisualCheckerboardGrid& checkerboard_grid,
    std::string& error_message
)
{
    checkerboard_grid = LiveVisualCheckerboardGrid{};
    error_message.clear();

    try {
        if (!points_json.contains("pseudo_slam_points") ||
            !points_json["pseudo_slam_points"].is_array()) {
            error_message = "pseudo_slam_points.json格式错误";
            return false;
        }

        std::unordered_map<int, float> row_sums_by_global_row;
        std::unordered_map<int, int> row_counts_by_global_row;
        std::unordered_map<int, float> col_sums_by_global_col;
        std::unordered_map<int, int> col_counts_by_global_col;
        for (const auto& point_json : points_json["pseudo_slam_points"]) {
            if (!point_json.value("is_planning_checkerboard_member", false)) {
                continue;
            }

            const int global_row = point_json.value("planning_global_row", -1);
            const int global_col = point_json.value("planning_global_col", -1);
            if (global_row < 0 || global_col < 0) {
                continue;
            }

            row_sums_by_global_row[global_row] +=
                point_json.value("y", point_json.value("world_y", 0.0f));
            row_counts_by_global_row[global_row] += 1;
            col_sums_by_global_col[global_col] +=
                point_json.value("x", point_json.value("world_x", 0.0f));
            col_counts_by_global_col[global_col] += 1;

            PseudoSlamCheckerboardInfo info;
            info.global_idx = point_json.value("global_idx", point_json.value("idx", -1));
            info.global_row = global_row;
            info.global_col = global_col;
            info.checkerboard_parity = point_json.value("planning_checkerboard_parity", -1);
            info.is_checkerboard_member = true;
            checkerboard_grid.info_by_cell_key[encode_checkerboard_cell_key(global_row, global_col)] = info;
        }

        if (checkerboard_grid.info_by_cell_key.empty()) {
            error_message = "pseudo_slam_points.json中缺少可执行全局棋盘格";
            return false;
        }

        for (const auto& entry : row_sums_by_global_row) {
            const int global_row = entry.first;
            checkerboard_grid.row_centers_by_global_row[global_row] =
                entry.second / static_cast<float>(row_counts_by_global_row[global_row]);
        }

        for (const auto& entry : col_sums_by_global_col) {
            const int global_col = entry.first;
            checkerboard_grid.col_centers_by_global_col[global_col] =
                entry.second / static_cast<float>(col_counts_by_global_col[global_col]);
        }
    } catch (const std::exception&) {
        error_message = "pseudo_slam_points.json读取失败";
        return false;
    }

    return true;
}

bool classify_live_visual_point_into_checkerboard(
    const tie_robot_msgs::PointCoords& world_point,
    const LiveVisualCheckerboardGrid& checkerboard_grid,
    nlohmann::json& classified_point_json
)
{
    classified_point_json = nlohmann::json();
    if (checkerboard_grid.row_centers_by_global_row.empty() ||
        checkerboard_grid.col_centers_by_global_col.empty()) {
        return false;
    }

    const int global_col = find_nearest_checkerboard_center_key(
        world_point.World_coord[0],
        checkerboard_grid.col_centers_by_global_col
    );
    const int global_row = find_nearest_checkerboard_center_key(
        world_point.World_coord[1],
        checkerboard_grid.row_centers_by_global_row
    );
    if (global_row < 0 || global_col < 0) {
        return false;
    }

    if (std::fabs(
            world_point.World_coord[0] -
            checkerboard_grid.col_centers_by_global_col.at(global_col)
        ) >
            kPseudoSlamCheckerboardAxisThresholdMm ||
        std::fabs(
            world_point.World_coord[1] -
            checkerboard_grid.row_centers_by_global_row.at(global_row)
        ) >
            kPseudoSlamCheckerboardAxisThresholdMm) {
        return false;
    }

    const auto checkerboard_it = checkerboard_grid.info_by_cell_key.find(
        encode_checkerboard_cell_key(global_row, global_col)
    );
    if (checkerboard_it == checkerboard_grid.info_by_cell_key.end()) {
        return false;
    }

    classified_point_json = {
        {"idx", world_point.idx},
        {"local_idx", world_point.idx},
        {"global_idx", checkerboard_it->second.global_idx},
        {"global_row", global_row},
        {"global_col", global_col},
        {"checkerboard_parity", checkerboard_it->second.checkerboard_parity},
        {"is_checkerboard_member", checkerboard_it->second.is_checkerboard_member},
        {"x", world_point.World_coord[0]},
        {"y", world_point.World_coord[1]},
        {"z", world_point.World_coord[2]},
        {"world_x", world_point.World_coord[0]},
        {"world_y", world_point.World_coord[1]},
        {"world_z", world_point.World_coord[2]},
        {"angle", world_point.Angle},
    };
    return true;
}

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_non_checkerboard_points(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx
)
{
    if (planning_points.empty() || checkerboard_info_by_idx.empty()) {
        return planning_points;
    }

    std::vector<tie_robot_msgs::PointCoords> filtered_points;
    filtered_points.reserve(planning_points.size());
    int removed_count = 0;
    for (const auto& planning_point : planning_points) {
        const auto checkerboard_it = checkerboard_info_by_idx.find(planning_point.idx);
        if (checkerboard_it == checkerboard_info_by_idx.end() ||
            !checkerboard_it->second.is_checkerboard_member) {
            removed_count++;
            continue;
        }
        filtered_points.push_back(planning_point);
    }

    if (removed_count > 0) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam棋盘格成员过滤：无法融入棋盘格行列邻接的点视为离群点，移除%d个点，剩余%d个规划点。\n",
            removed_count,
            static_cast<int>(filtered_points.size())
        );
    }

    return filtered_points;
}
