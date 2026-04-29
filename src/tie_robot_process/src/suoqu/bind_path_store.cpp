#include "suoqu_runtime_internal.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

bool write_json_file_atomically(
    const std::string& final_path,
    const nlohmann::json& json_value,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    const std::string temp_path = final_path + ".tmp";
    std::ofstream file_obj(temp_path);
    if (!file_obj.is_open()) {
        if (error_message != nullptr) {
            *error_message = "无法打开临时文件进行写入";
        }
        return false;
    }

    file_obj << json_value.dump(4);
    file_obj.flush();
    file_obj.close();
    if (!file_obj.good()) {
        std::remove(temp_path.c_str());
        if (error_message != nullptr) {
            *error_message = "写入临时文件失败";
        }
        return false;
    }
    if (std::rename(temp_path.c_str(), final_path.c_str()) != 0) {
        std::remove(temp_path.c_str());
        if (error_message != nullptr) {
            *error_message = "原子替换目标文件失败";
        }
        return false;
    }

    return true;
}

bool write_pseudo_slam_points_json(
    const std::vector<tie_robot_msgs::PointCoords>& merged_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& planning_checkerboard_info_by_idx,
    const std::unordered_set<int>& outlier_secondary_plane_global_indices,
    const std::unordered_set<int>& outlier_line_global_indices,
    const std::unordered_set<int>& outlier_column_neighbor_blocked_global_indices,
    const std::string& scan_session_id,
    const std::string& path_signature,
    std::string* error_message
)
{
    nlohmann::json points_json;
    points_json["scan_session_id"] = scan_session_id;
    points_json["path_signature"] = path_signature;
    points_json["pseudo_slam_points"] = nlohmann::json::array();
    for (const auto& point : merged_points) {
        auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
        auto planning_checkerboard_it = planning_checkerboard_info_by_idx.find(point.idx);
        const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
        const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
        const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
        const bool is_checkerboard_member =
            checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
        const int planning_global_row =
            planning_checkerboard_it != planning_checkerboard_info_by_idx.end() ? planning_checkerboard_it->second.global_row : -1;
        const int planning_global_col =
            planning_checkerboard_it != planning_checkerboard_info_by_idx.end() ? planning_checkerboard_it->second.global_col : -1;
        const int planning_checkerboard_parity =
            planning_checkerboard_it != planning_checkerboard_info_by_idx.end() ? planning_checkerboard_it->second.checkerboard_parity : -1;
        const bool is_planning_checkerboard_member =
            planning_checkerboard_it != planning_checkerboard_info_by_idx.end() ?
                planning_checkerboard_it->second.is_checkerboard_member :
                false;
        const bool is_planning_outlier = !is_planning_checkerboard_member;
        const bool is_planning_outlier_line_member =
            outlier_line_global_indices.count(point.idx) > 0;
        const bool is_outlier_secondary_plane_member =
            outlier_secondary_plane_global_indices.count(point.idx) > 0;
        const bool is_outlier_column_neighbor_blocked =
            outlier_column_neighbor_blocked_global_indices.count(point.idx) > 0;
        points_json["pseudo_slam_points"].push_back(
            {
                {"idx", point.idx},
                {"global_idx", point.idx},
                {"global_row", global_row},
                {"global_col", global_col},
                {"checkerboard_parity", checkerboard_parity},
                {"is_checkerboard_member", is_checkerboard_member},
                {"planning_global_row", planning_global_row},
                {"planning_global_col", planning_global_col},
                {"planning_checkerboard_parity", planning_checkerboard_parity},
                {"is_planning_checkerboard_member", is_planning_checkerboard_member},
                {"is_planning_outlier", is_planning_outlier},
                {"is_planning_outlier_line_member", is_planning_outlier_line_member},
                {"is_outlier_secondary_plane_member", is_outlier_secondary_plane_member},
                {"is_outlier_column_neighbor_blocked", is_outlier_column_neighbor_blocked},
                {"x", point.World_coord[0]},
                {"y", point.World_coord[1]},
                {"z", point.World_coord[2]},
                {"angle", point.Angle},
            }
        );
    }

    return write_json_file_atomically(pseudo_slam_points_json_file, points_json, error_message);
}

bool write_pseudo_slam_bind_path_json(
    const std::vector<PseudoSlamGroupedAreaEntry>& area_entries,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const BindExecutionPathOriginPose& path_origin,
    float cabin_height,
    float cabin_speed,
    const std::string& scan_session_id,
    const std::string& path_signature,
    std::string* error_message
)
{
    tf2::Transform gripper_from_base_link;
    if (!lookup_gripper_from_base_link_transform(gripper_from_base_link)) {
        if (error_message != nullptr) {
            *error_message = "无法获取base_link->gripper_frame变换，无法为pseudo_slam_bind_path.json写入TCP局部坐标";
        }
        return false;
    }

    nlohmann::json bind_path_json;
    bind_path_json["scan_session_id"] = scan_session_id;
    bind_path_json["path_signature"] = path_signature;
    bind_path_json["scan_mode"] = "scan_only";
    bind_path_json["cabin_height"] = cabin_height;
    bind_path_json["cabin_speed"] = cabin_speed;
    bind_path_json["path_origin"] = {
        {"x", path_origin.x},
        {"y", path_origin.y},
        {"z", path_origin.z},
    };
    bind_path_json["areas"] = nlohmann::json::array();

    for (const auto& area_entry : area_entries) {
        nlohmann::json area_json;
        area_json["area_index"] = area_entry.area_index;
        area_json["cabin_pose"] = {
            {"x", area_entry.cabin_point.x},
            {"y", area_entry.cabin_point.y},
            {"z", area_entry.cabin_z},
        };
        area_json["groups"] = nlohmann::json::array();
        for (const auto& bind_group : area_entry.bind_groups) {
            nlohmann::json group_json;
            group_json["group_index"] = bind_group.group_index;
            group_json["group_type"] = bind_group.group_type;
            group_json["points"] = nlohmann::json::array();
            int local_idx = 1;
            for (const auto& point : bind_group.bind_points_world) {
                auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
                const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
                const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
                const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
                const bool is_checkerboard_member =
                    checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
                nlohmann::json point_json = {
                    {"idx", local_idx},
                    {"local_idx", local_idx},
                    {"global_idx", point.idx},
                    {"global_row", global_row},
                    {"global_col", global_col},
                    {"checkerboard_parity", checkerboard_parity},
                    {"is_checkerboard_member", is_checkerboard_member},
                    {"world_x", point.World_coord[0]},
                    {"world_y", point.World_coord[1]},
                    {"world_z", point.World_coord[2]},
                    {"angle", point.Angle},
                };
                if (!assign_planned_gripper_coords_to_bind_point_json(
                        point_json,
                        area_entry.cabin_point,
                        area_entry.cabin_z,
                        gripper_from_base_link
                    )) {
                    if (error_message != nullptr) {
                        *error_message =
                            "无法根据规划区域cabin_pose为pseudo_slam_bind_path.json生成TCP局部点坐标";
                    }
                    return false;
                }
                group_json["points"].push_back(point_json);
                local_idx++;
            }
            area_json["groups"].push_back(group_json);
        }
        bind_path_json["areas"].push_back(area_json);
    }

    return write_json_file_atomically(pseudo_slam_bind_path_json_file, bind_path_json, error_message);
}

bool invalidate_scan_session_in_artifact_file(
    const std::string& artifact_path,
    const std::string& artifact_name,
    const std::string& invalid_reason,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    std::ifstream artifact_file(artifact_path);
    if (!artifact_file.is_open()) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "不存在，无法失效化scan_session_id";
        }
        return false;
    }

    nlohmann::json artifact_json;
    try {
        artifact_file >> artifact_json;
        if (artifact_file.fail() && !artifact_file.eof()) {
            throw std::runtime_error("artifact read failure");
        }
        if (!artifact_json.is_object()) {
            throw std::runtime_error("artifact root must be object");
        }
    } catch (const std::exception&) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "读取或解析失败，无法失效化scan_session_id";
        }
        return false;
    }

    artifact_json["scan_session_id"] = "";
    artifact_json["scan_session_invalid_reason"] = invalid_reason;
    artifact_json["requires_rescan"] = true;

    std::string write_error;
    if (!write_json_file_atomically(artifact_path, artifact_json, &write_error)) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "写回失败：" + write_error;
        }
        return false;
    }

    return true;
}

bool invalidate_current_scan_artifacts_after_execution_memory_write_failure(
    const std::string& write_failure_reason,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    const std::string invalid_reason =
        "bind_execution_memory.json写入失败：" + write_failure_reason +
        "；为避免重启后继续消费旧账本，需要重新扫描/重新建图";

    std::string points_error;
    const bool points_invalidated = invalidate_scan_session_in_artifact_file(
        pseudo_slam_points_json_file,
        "pseudo_slam_points.json",
        invalid_reason,
        &points_error
    );
    std::string bind_path_error;
    const bool bind_path_invalidated = invalidate_scan_session_in_artifact_file(
        pseudo_slam_bind_path_json_file,
        "pseudo_slam_bind_path.json",
        invalid_reason,
        &bind_path_error
    );
    if (points_invalidated && bind_path_invalidated) {
        return true;
    }

    std::ostringstream oss;
    oss << "账本写盘失败后失效化当前扫描产物session失败";
    if (!points_invalidated) {
        oss << "；pseudo_slam_points.json=" << points_error;
    }
    if (!bind_path_invalidated) {
        oss << "；pseudo_slam_bind_path.json=" << bind_path_error;
    }
    if (error_message != nullptr) {
        *error_message = oss.str();
    }
    return false;
}
