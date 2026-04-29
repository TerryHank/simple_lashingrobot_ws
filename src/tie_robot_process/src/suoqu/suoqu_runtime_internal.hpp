#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/Point.h>
#include <tie_robot_process/json.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tie_robot_msgs/ExecuteBindPoints.h"
#include "tie_robot_msgs/MotionControl.h"
#include "tie_robot_msgs/Pathguihua.h"
#include "tie_robot_msgs/PointCoords.h"
#include "tie_robot_msgs/ProcessImage.h"
#include "tie_robot_msgs/SingleMove.h"
#include "tie_robot_msgs/StartGlobalWork.h"
#include "tie_robot_msgs/StartPseudoSlamScan.h"
#include "tie_robot_process/planning/dynamic_bind_planning.hpp"
#include <tie_robot_process/common.hpp>

struct Cabin_Point
{
    float x;
    float y;
};

struct Cabin_State
{
    float X;
    float Y;
    float Z;
    float A;
    float B;
    float C;
    int motion_status;
    int device_alarm;
    int internal_calc_error;
    float cabin_x_gesture;
    float cabin_y_gesture;
};

enum class GlobalExecutionMode
{
    kSlamPrecomputed = 0,
    kLiveVisual = 1,
};

enum class PseudoSlamScanStrategy
{
    kSingleCenter = 0,
    kMultiPose = 1,
    kFixedManualWorkspace = 2,
};

struct PseudoSlamCheckerboardInfo
{
    int global_idx = -1;
    int global_row = -1;
    int global_col = -1;
    int checkerboard_parity = -1;
    bool is_checkerboard_member = false;
};

struct PseudoSlamMarkerExecutionState
{
    int current_area_index = -1;
    std::unordered_set<int> highlighted_area_global_indices;
    std::unordered_set<int> active_dispatch_global_indices;
};

struct LiveVisualCheckerboardGrid
{
    std::unordered_map<int, float> row_centers_by_global_row;
    std::unordered_map<int, float> col_centers_by_global_col;
    std::unordered_map<long long, PseudoSlamCheckerboardInfo> info_by_cell_key;
};

struct BindExecutionPointRecord
{
    int global_row = -1;
    int global_col = -1;
    int checkerboard_parity = -1;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float world_z = 0.0f;
    std::string source_mode;
};

struct BindExecutionMemory
{
    std::string scan_session_id;
    std::string path_signature;
    Cabin_Point path_origin{};
    std::vector<BindExecutionPointRecord> executed_points;
};

struct PseudoSlamOverlapCluster
{
    std::vector<tie_robot_msgs::PointCoords> member_points;
    double sum_x_mm = 0.0;
    double sum_y_mm = 0.0;
};

using PseudoSlamBindGroup = tie_robot_process::planning::PseudoSlamBindGroup;
using PseudoSlamGroupedAreaEntry = tie_robot_process::planning::PseudoSlamGroupedAreaEntry;
using BindExecutionPathOriginPose = tie_robot_process::planning::BindExecutionPathOriginPose;

constexpr int MOTION_COMMAND_STATUS_RETRY_LOG_INTERVAL_SEC = 2;
constexpr uint8_t kProcessImageModeScanOnly = 3;
constexpr uint8_t kProcessImageModeExecutionRefine = 4;
constexpr float kTravelMaxXMm = 360.0f;
constexpr float kTravelMaxYMm = 320.0f;
constexpr float kTravelMaxZMm = 140.0f;
constexpr float kBindExecutionCabinMinZMm = 485.0f;
constexpr float kPseudoSlamDedupDistanceMm = 100.0f;
constexpr float kPseudoSlamClosePointClusterXYToleranceMm = 100.0f;
constexpr float kPseudoSlamScanDuplicateXYToleranceMm = 10.0f;
constexpr float kPseudoSlamGlobalScanHeightOffsetMm = 1500.0f;
constexpr float kPseudoSlamFixedManualWorkspaceScanXmm = -260.0f;
constexpr float kPseudoSlamFixedManualWorkspaceScanYmm = 1700.0f;
constexpr float kPseudoSlamFixedManualWorkspaceScanZmm = 3197.0f;
constexpr float kPseudoSlamFixedManualWorkspaceScanSpeedMmPerSec = 100.0f;
constexpr float kPseudoSlamPlanningZOutlierMm = 8.0f;
constexpr float kPseudoSlamOutlierColumnAxisToleranceMm = 10.0f;
constexpr float kPseudoSlamOutlierColumnPointToleranceMm = 10.0f;
constexpr float kPseudoSlamOutlierLineDistanceToleranceMm = 12.0f;
constexpr int kPseudoSlamOutlierLineMinPointCount = 3;
constexpr int kPseudoSlamOutlierSecondaryPlaneMinPointCount = 6;
constexpr float kPseudoSlamOutlierSecondaryPlaneNeighborToleranceMm = 100.0f;
constexpr float kPseudoSlamCheckerboardAxisThresholdMm = 80.0f;
constexpr float kLiveVisualMicroAdjustXYToleranceMm = 30.0f;
constexpr float kLiveVisualMicroAdjustZToleranceMm = 6.0f;
constexpr float kCurrentAreaBindTestCabinSpeedMultiplier = 1.5f;
constexpr float kCurrentAreaBindTestMinCabinSpeedMmPerSec = 450.0f;
constexpr float kExecutionArrivalToleranceMm = 40.0f;
constexpr int kExecutionArrivalStableSampleCount = 2;
constexpr float kExecutionArrivalPoseDeltaToleranceMm = 5.0f;
constexpr float kDynamicBindTemplateCenterXMm = 150.0f;
constexpr float kDynamicBindTemplateCenterYMm = 150.0f;
constexpr float kDynamicBindTemplateCenterZMm = 70.0f;
constexpr float kDynamicBindSnakeRowToleranceMm = 90.0f;
constexpr int kDynamicBindSeedNeighborCount = 8;
constexpr int kPseudoSlamScanFrameCount = 2;
constexpr int kPseudoSlamScanMinPointCount = 5;
constexpr double kPseudoSlamScanRetryIntervalSec = 0.2;
const std::string kPseudoSlamCaptureGateImageTopic = "/Scepter/ir/image_raw";
constexpr int kPseudoSlamCaptureGateStableSampleCount = 3;
constexpr double kPseudoSlamCaptureGatePollIntervalSec = 0.1;
constexpr double kPseudoSlamCaptureGateImageMeanDiffThreshold = 2.5;
constexpr double kPseudoSlamCaptureGateLogIntervalSec = 1.0;
constexpr float kPseudoSlamCaptureGateTargetToleranceMm = 25.0f;
constexpr float kPseudoSlamCaptureGatePoseDeltaToleranceMm = 1.0f;
constexpr int kPseudoSlamCaptureGateRoiMinX = 20;
constexpr int kPseudoSlamCaptureGateRoiMaxX = 620;
constexpr int kPseudoSlamCaptureGateRoiMinY = 114;
constexpr int kPseudoSlamCaptureGateRoiMaxY = 460;

extern std::string path_points_json_file;
extern std::string pseudo_slam_points_json_file;
extern std::string pseudo_slam_bind_path_json_file;
extern const std::string kBindExecutionMemoryJsonPath;
extern const char* kBindExecutionMemoryUnreadableError;

extern std::mutex cabin_state_mutex;
extern std::mutex pseudo_slam_workflow_mutex;
extern std::mutex pseudo_slam_tf_points_mutex;
extern std::mutex pseudo_slam_ir_image_mutex;
extern std::mutex pseudo_slam_marker_state_mutex;
extern std::mutex pseudo_slam_marker_path_origin_mutex;

extern Cabin_State cabin_state;
extern ros::Publisher pub_pseudo_slam_markers;
extern std::vector<tie_robot_msgs::PointCoords> pseudo_slam_marker_points;
extern std::unordered_set<int> pseudo_slam_marker_outlier_global_indices;
extern std::unordered_set<int> pseudo_slam_marker_outlier_secondary_plane_global_indices;
extern std::unordered_set<int> pseudo_slam_marker_outlier_line_global_indices;
extern std::unordered_set<int> pseudo_slam_marker_outlier_column_neighbor_global_indices;
extern PseudoSlamMarkerExecutionState pseudo_slam_marker_execution_state;
extern Cabin_Point pseudo_slam_marker_path_origin;
extern bool pseudo_slam_marker_path_origin_valid;
extern std::atomic<float> pseudo_slam_marker_last_outlier_threshold_mm;
extern std::atomic<float> pseudo_slam_marker_last_outlier_secondary_plane_threshold_mm;
extern std::atomic<float> pseudo_slam_marker_last_outlier_secondary_plane_neighbor_tolerance_mm;
extern std::atomic<int> global_execution_mode;
extern std::atomic<bool> cabin_driver_enabled;
extern std::atomic<double> cabin_driver_last_state_stamp_sec;
extern std::atomic<bool> moduan_work_flag;

void printCurrentTime();
bool connectToServer();
float get_global_cabin_move_speed_mm_per_sec();
bool lookup_gripper_from_base_link_transform(tf2::Transform& gripper_from_base_link);
bool lookup_current_gripper_from_cabin_transform(tf2::Transform& gripper_from_cabin);
float load_pseudo_slam_planning_z_outlier_threshold_mm();
float load_pseudo_slam_outlier_secondary_plane_threshold_mm();
float load_pseudo_slam_outlier_secondary_plane_neighbor_tolerance_mm();
GlobalExecutionMode get_global_execution_mode();
const char* global_execution_mode_name(GlobalExecutionMode mode);
PseudoSlamScanStrategy normalize_pseudo_slam_scan_strategy(uint8_t raw_strategy);

void clear_pseudo_slam_markers();
void publish_pseudo_slam_markers(const std::vector<tie_robot_msgs::PointCoords>& world_points);
std::unordered_set<int> collect_global_indices_from_point_json_array(const nlohmann::json& points_json);
std::unordered_set<int> collect_global_indices_from_group_json(const nlohmann::json& group_json);
std::unordered_set<int> collect_global_indices_from_area_json(const nlohmann::json& area_json);
void set_pseudo_slam_marker_execution_state(
    int area_index,
    const std::unordered_set<int>& highlighted_area_global_indices,
    const std::unordered_set<int>& active_dispatch_global_indices
);
void set_pseudo_slam_marker_path_origin(const Cabin_Point& path_origin);
bool recompute_pseudo_slam_marker_outlier_sets(
    const std::vector<tie_robot_msgs::PointCoords>& marker_points_snapshot,
    const Cabin_Point& path_origin,
    std::unordered_set<int>& outlier_global_indices,
    std::unordered_set<int>& outlier_secondary_plane_global_indices,
    std::unordered_set<int>& outlier_line_global_indices,
    std::unordered_set<int>& outlier_column_neighbor_global_indices
);
bool refresh_pseudo_slam_marker_outlier_state_from_current_points(bool log_refresh);
void maybe_refresh_pseudo_slam_marker_outlier_threshold();
void clear_pseudo_slam_marker_execution_state();
bool load_pseudo_slam_marker_points_from_json(
    std::vector<tie_robot_msgs::PointCoords>& restored_marker_points,
    std::unordered_set<int>& restored_outlier_global_indices,
    std::unordered_set<int>& restored_outlier_secondary_plane_global_indices,
    std::unordered_set<int>& restored_outlier_line_global_indices,
    std::unordered_set<int>& restored_outlier_column_neighbor_global_indices,
    std::string& error_message
);
bool load_pseudo_slam_marker_path_origin_from_bind_path_json(
    Cabin_Point& restored_path_origin,
    std::string& error_message
);
void restore_pseudo_slam_markers_from_json_on_startup();

double point_distance_mm(
    const tie_robot_msgs::PointCoords& lhs,
    const tie_robot_msgs::PointCoords& rhs
);
bool transform_cabin_world_point_to_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    tie_robot_msgs::PointCoords& gripper_point
);
bool is_local_bind_point_in_range(const tie_robot_msgs::PointCoords& point);
std::vector<tie_robot_msgs::PointCoords> dedupe_world_points(
    const std::vector<tie_robot_msgs::PointCoords>& input_points
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_close_xy_point_clusters(
    const std::vector<tie_robot_msgs::PointCoords>& input_points
);
std::vector<tie_robot_msgs::PointCoords> filter_new_scan_points_against_existing_xy_tolerance(
    const std::vector<tie_robot_msgs::PointCoords>& candidate_points,
    const std::vector<tie_robot_msgs::PointCoords>& existing_points,
    float tolerance_mm
);
std::vector<tie_robot_msgs::PointCoords> build_scan_cluster_representatives(
    const std::vector<PseudoSlamOverlapCluster>& clusters
);
void merge_frame_points_into_overlap_clusters(
    const std::vector<tie_robot_msgs::PointCoords>& frame_world_points,
    int scan_pose_index,
    std::vector<PseudoSlamOverlapCluster>& clusters,
    float tolerance_mm
);
bool compute_pseudo_slam_global_scan_pose(
    const std::vector<Cabin_Point>& con_path,
    float planning_cabin_height,
    Cabin_Point& scan_center,
    float& scan_height,
    std::string& error_message
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_columns(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_planning_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
);
std::vector<tie_robot_msgs::PointCoords> collect_pseudo_slam_planning_z_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_secondary_plane_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_secondary_plane_members(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& secondary_plane_outlier_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_line_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_column_neighbor_blocked_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::unordered_map<int, PseudoSlamCheckerboardInfo> build_checkerboard_info_by_global_index(
    const std::vector<tie_robot_msgs::PointCoords>& world_points,
    const Cabin_Point& path_origin
);
std::unordered_map<int, PseudoSlamCheckerboardInfo> sync_merged_checkerboard_membership_with_planning(
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& merged_checkerboard_info_by_idx,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& planning_checkerboard_info_by_idx
);
long long encode_checkerboard_cell_key(int global_row, int global_col);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_non_checkerboard_points(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx
);
bool validate_scan_session_alignment(
    const nlohmann::json& artifact_json,
    const std::string& artifact_name,
    const BindExecutionMemory& bind_execution_memory,
    std::string& error_message
);
bool validate_path_signature_alignment(
    const nlohmann::json& artifact_json,
    const std::string& artifact_name,
    const BindExecutionMemory& bind_execution_memory,
    const std::string& current_path_signature,
    std::string& error_message
);
bool load_scan_artifact_json(
    const std::string& artifact_path,
    const std::string& artifact_name,
    nlohmann::json& artifact_json,
    std::string& error_message
);
bool load_scan_artifacts_for_execution(
    nlohmann::json& points_json,
    nlohmann::json& bind_path_json,
    const BindExecutionMemory& bind_execution_memory,
    const std::string& current_path_signature,
    std::string& error_message
);
bool load_live_visual_checkerboard_grid(
    const nlohmann::json& points_json,
    LiveVisualCheckerboardGrid& checkerboard_grid,
    std::string& error_message
);
bool classify_live_visual_point_into_checkerboard(
    const tie_robot_msgs::PointCoords& world_point,
    const LiveVisualCheckerboardGrid& checkerboard_grid,
    nlohmann::json& classified_point_json
);
float clamp_bind_execution_cabin_z(float planned_cabin_z);
bool assign_planned_gripper_coords_to_bind_point_json(
    nlohmann::json& point_json,
    const Cabin_Point& cabin_point,
    float cabin_z,
    const tf2::Transform& gripper_from_base_link
);
bool assign_planned_gripper_coords_to_bind_point_json(
    nlohmann::json& point_json,
    const tie_robot_process::planning::CabinPoint& cabin_point,
    float cabin_z,
    const tf2::Transform& gripper_from_base_link
);
bool transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const Cabin_Point& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_base_link,
    tie_robot_msgs::PointCoords& gripper_point
);
bool transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const tie_robot_process::planning::CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_base_link,
    tie_robot_msgs::PointCoords& gripper_point
);
void align_execution_path_origin_xy_to_first_area_if_needed(
    const nlohmann::json& areas_json,
    float& path_origin_x,
    float& path_origin_y,
    const char* execution_mode_name
);
std::unordered_set<int> collect_blocked_execution_global_indices_from_points_json(
    const nlohmann::json& points_json
);
nlohmann::json filter_precomputed_group_points_for_execution(
    const nlohmann::json& group_json,
    const BindExecutionMemory& memory,
    const std::unordered_set<int>& blocked_global_indices,
    bool only_checkerboard_parity_zero
);
bool load_bind_execution_memory_json(
    BindExecutionMemory& memory,
    std::string& error_message
);
bool write_bind_execution_memory_json(
    const BindExecutionMemory& memory,
    std::string* error_message
);
BindExecutionMemory reset_bind_execution_memory_for_scan_session(
    const std::string& scan_session_id,
    const std::string& path_signature,
    const Cabin_Point& path_origin
);
bool invalidate_current_scan_artifacts_after_execution_memory_write_failure(
    const std::string& write_failure_reason,
    std::string* error_message
);
bool is_point_already_executed(
    const BindExecutionMemory& memory,
    int global_row,
    int global_col
);
void record_successful_execution_point(
    BindExecutionMemory& memory,
    const nlohmann::json& point_json,
    const std::string& source_mode = "slam_precomputed"
);
bool write_json_file_atomically(
    const std::string& final_path,
    const nlohmann::json& json_value,
    std::string* error_message
);
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
);
bool write_pseudo_slam_bind_path_json(
    const std::vector<PseudoSlamGroupedAreaEntry>& area_entries,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const BindExecutionPathOriginPose& path_origin,
    float cabin_height,
    float cabin_speed,
    const std::string& scan_session_id,
    const std::string& path_signature,
    std::string* error_message
);

bool load_configured_path(
    std::vector<Cabin_Point>& con_path,
    float& cabin_height,
    float& cabin_speed
);
bool load_current_path_signature_for_execution(
    std::string& current_path_signature,
    std::string& error_message
);
bool reset_bind_execution_memory_from_current_scan_artifacts(
    const std::string& current_path_signature,
    std::string& error_message
);
bool run_pseudo_slam_scan(
    std::vector<Cabin_Point> con_path,
    float planning_cabin_height,
    float cabin_speed,
    PseudoSlamScanStrategy scan_strategy,
    bool enable_capture_gate,
    std::string& message
);
bool run_current_area_bind_from_scan_test(std::string& message);
bool run_bind_path_direct_test(std::string& message);
bool run_live_visual_global_work(std::string& message);
bool run_bind_from_scan(std::string& message);

bool startPseudoSlamScan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool cabinDriverStartService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool cabinDriverStopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool cabinDriverRestartService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool cabinMotionStopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool startPseudoSlamScanWithOptions(
    tie_robot_msgs::StartPseudoSlamScan::Request& req,
    tie_robot_msgs::StartPseudoSlamScan::Response& res
);
bool bind_current_area_from_scan_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool bind_path_direct_test_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
bool startGlobalWork(
    tie_robot_msgs::MotionControl::Request& req,
    tie_robot_msgs::MotionControl::Response& res
);
bool startGlobalWorkWithOptions(
    tie_robot_msgs::StartGlobalWork::Request& req,
    tie_robot_msgs::StartGlobalWork::Response& res
);
