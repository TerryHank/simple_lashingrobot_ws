#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "tie_robot_process/planning/dynamic_bind_planning.hpp"

namespace tie_robot_process {
namespace planning {
namespace {

tie_robot_msgs::PointCoords make_world_point(int idx, float x, float y, float z)
{
    tie_robot_msgs::PointCoords point;
    point.idx = idx;
    point.World_coord[0] = x;
    point.World_coord[1] = y;
    point.World_coord[2] = z;
    point.Angle = 0.0f;
    point.is_shuiguan = false;
    return point;
}

DynamicBindGridIndex make_grid_index(int idx, int row, int col)
{
    DynamicBindGridIndex grid_index;
    grid_index.global_idx = idx;
    grid_index.global_row = row;
    grid_index.global_col = col;
    return grid_index;
}

std::set<int> collect_unique_indices(const PseudoSlamBindGroup& bind_group)
{
    std::set<int> unique_indices;
    for (const auto& point : bind_group.bind_points_world) {
        unique_indices.insert(point.idx);
    }
    return unique_indices;
}

std::vector<int> collect_area_point_indices(const PseudoSlamGroupedAreaEntry& area_entry)
{
    std::vector<int> indices;
    for (const auto& bind_group : area_entry.bind_groups) {
        for (const auto& point : bind_group.bind_points_world) {
            indices.push_back(point.idx);
        }
    }
    return indices;
}

std::array<float, 2> compute_group_min_world_xy(const PseudoSlamGroupedAreaEntry& area_entry)
{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    for (const auto& bind_group : area_entry.bind_groups) {
        for (const auto& point : bind_group.bind_points_world) {
            min_x = std::min(min_x, point.World_coord[0]);
            min_y = std::min(min_y, point.World_coord[1]);
        }
    }
    if (min_x == std::numeric_limits<float>::max()) {
        return {0.0f, 0.0f};
    }
    return {min_x, min_y};
}

std::array<float, 3> compute_local_group_center(
    const PseudoSlamGroupedAreaEntry& area_entry,
    const tf2::Transform& gripper_from_base_link)
{
    tf2::Transform cabin_from_base_link;
    cabin_from_base_link.setIdentity();
    cabin_from_base_link.setOrigin(tf2::Vector3(
        static_cast<double>(area_entry.cabin_point.x) / 1000.0,
        static_cast<double>(area_entry.cabin_point.y) / 1000.0,
        static_cast<double>(area_entry.cabin_z) / 1000.0));
    const tf2::Transform gripper_from_cabin =
        gripper_from_base_link * cabin_from_base_link.inverse();

    std::array<double, 3> local_sum{0.0, 0.0, 0.0};
    int point_count = 0;
    for (const auto& bind_group : area_entry.bind_groups) {
        for (const auto& point : bind_group.bind_points_world) {
            const tf2::Vector3 point_in_map(
                static_cast<double>(point.World_coord[0]) / 1000.0,
                static_cast<double>(point.World_coord[1]) / 1000.0,
                static_cast<double>(point.World_coord[2]) / 1000.0);
            const tf2::Vector3 point_in_gripper = gripper_from_cabin * point_in_map;
            local_sum[0] += point_in_gripper.x() * 1000.0;
            local_sum[1] += point_in_gripper.y() * 1000.0;
            local_sum[2] += point_in_gripper.z() * 1000.0;
            point_count++;
        }
    }

    if (point_count <= 0) {
        return {0.0f, 0.0f, 0.0f};
    }
    return {
        static_cast<float>(local_sum[0] / static_cast<double>(point_count)),
        static_cast<float>(local_sum[1] / static_cast<double>(point_count)),
        static_cast<float>(local_sum[2] / static_cast<double>(point_count)),
    };
}

void expect_area_group_center_matches_workspace_center(
    const PseudoSlamGroupedAreaEntry& area_entry,
    const tf2::Transform& gripper_from_base_link,
    const DynamicBindPlannerConfig& config)
{
    const auto local_center = compute_local_group_center(
        area_entry,
        gripper_from_base_link);
    EXPECT_NEAR(local_center[0], config.tcp_max_x_mm * 0.5f, 1e-3f);
    EXPECT_NEAR(local_center[1], config.tcp_max_y_mm * 0.5f, 1e-3f);
    EXPECT_NEAR(local_center[2], config.tcp_max_z_mm * 0.5f, 1e-3f);
}

std::vector<std::tuple<int, int, int>> collect_area_grid_cells(
    const PseudoSlamGroupedAreaEntry& area_entry,
    int column_count)
{
    std::vector<std::tuple<int, int, int>> cells;
    for (const int point_idx : collect_area_point_indices(area_entry)) {
        const int zero_based = point_idx - 1;
        cells.emplace_back(point_idx, zero_based / column_count, zero_based % column_count);
    }
    return cells;
}

std::unordered_map<int, std::pair<int, int>> build_cell_map(
    const std::vector<DynamicBindGridIndex>& grid_indices)
{
    std::unordered_map<int, std::pair<int, int>> cells_by_index;
    for (const auto& grid_index : grid_indices) {
        cells_by_index[grid_index.global_idx] = {grid_index.global_row, grid_index.global_col};
    }
    return cells_by_index;
}

std::unordered_map<int, std::pair<int, int>> build_linear_cell_map(int row_count, int column_count)
{
    std::unordered_map<int, std::pair<int, int>> cells_by_index;
    for (int row = 0; row < row_count; ++row) {
        for (int col = 0; col < column_count; ++col) {
            cells_by_index[row * column_count + col + 1] = {row, col};
        }
    }
    return cells_by_index;
}

void expect_group_uses_adjacent_checkerboard_cells(
    const std::vector<int>& point_indices,
    const std::unordered_map<int, std::pair<int, int>>& cells_by_index)
{
    ASSERT_TRUE(point_indices.size() == 2u || point_indices.size() == 4u);
    std::set<std::pair<int, int>> unique_cells;
    std::set<int> row_indices;
    std::set<int> column_indices;
    for (const int point_idx : point_indices) {
        ASSERT_TRUE(cells_by_index.count(point_idx) > 0) << point_idx;
        const auto cell = cells_by_index.at(point_idx);
        EXPECT_TRUE(unique_cells.insert(cell).second);
        row_indices.insert(cell.first);
        column_indices.insert(cell.second);
    }

    if (point_indices.size() == 2u) {
        ASSERT_EQ(unique_cells.size(), 2u);
        const auto first_cell = cells_by_index.at(point_indices[0]);
        const auto second_cell = cells_by_index.at(point_indices[1]);
        const int row_delta = std::abs(first_cell.first - second_cell.first);
        const int column_delta = std::abs(first_cell.second - second_cell.second);
        EXPECT_TRUE((row_delta == 0 && column_delta == 1) || (row_delta == 1 && column_delta == 0));
        return;
    }

    EXPECT_EQ(unique_cells.size(), 4u);
    ASSERT_EQ(row_indices.size(), 2u);
    ASSERT_EQ(column_indices.size(), 2u);
    EXPECT_EQ(*row_indices.rbegin(), *row_indices.begin() + 1);
    EXPECT_EQ(*column_indices.rbegin(), *column_indices.begin() + 1);
}

tf2::Transform make_gripper_from_base_link_transform()
{
    constexpr double kPi = 3.14159265358979323846;
    tf2::Quaternion rotation;
    rotation.setRPY(kPi, 0.0, 0.0);

    tf2::Transform gripper_from_base_link;
    gripper_from_base_link.setIdentity();
    gripper_from_base_link.setRotation(rotation);
    return gripper_from_base_link;
}

TEST(DynamicBindPlanningTest, BuildsStrictTwoByTwoRectangleGroup)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 1075.0f, 2075.0f, 430.0f),
        make_world_point(2, 1225.0f, 2075.0f, 430.0f),
        make_world_point(3, 1075.0f, 2225.0f, 430.0f),
        make_world_point(4, 1225.0f, 2225.0f, 430.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{1000.0f, 2000.0f},
        500.0f,
        gripper_from_base_link);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    ASSERT_EQ(bind_area_entries.front().bind_groups.size(), 1u);
    const auto& bind_group = bind_area_entries.front().bind_groups.front();
    EXPECT_EQ(bind_group.group_type, "matrix_2x2");
    EXPECT_EQ(bind_group.bind_points_world.size(), 4u);
    EXPECT_EQ(collect_unique_indices(bind_group).size(), 4u);
}

TEST(DynamicBindPlanningTest, InfersGridRowsFromStableWorldXWhenDpRowsMapToWorldX)
{
    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 1000.0f, 2000.0f, 430.0f),
        make_world_point(2, 1004.0f, 2150.0f, 430.0f),
        make_world_point(3, 995.0f, 2300.0f, 430.0f),
        make_world_point(4, 1150.0f, 2002.0f, 430.0f),
        make_world_point(5, 1155.0f, 2148.0f, 430.0f),
        make_world_point(6, 1148.0f, 2301.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 0, 2),
        make_grid_index(4, 1, 0),
        make_grid_index(5, 1, 1),
        make_grid_index(6, 1, 2),
    };

    const DynamicBindGridAxisMapping axis_mapping =
        infer_dynamic_bind_grid_axis_mapping(planning_world_points, grid_indices);

    EXPECT_EQ(axis_mapping.row_axis, DynamicBindWorldAxis::kX);
    EXPECT_EQ(axis_mapping.col_axis, DynamicBindWorldAxis::kY);
    EXPECT_TRUE(axis_mapping.inferred_from_spans);
    EXPECT_LT(axis_mapping.row_mean_span_x_mm, axis_mapping.row_mean_span_y_mm);
    EXPECT_LT(axis_mapping.col_mean_span_y_mm, axis_mapping.col_mean_span_x_mm);
}

TEST(DynamicBindPlanningTest, CentersTwoByTwoGroupInTcpWorkspaceAtPlannedHeight)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();
    const DynamicBindPlannerConfig config;

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 1075.0f, 2075.0f, 430.0f),
        make_world_point(2, 1225.0f, 2075.0f, 430.0f),
        make_world_point(3, 1075.0f, 2225.0f, 430.0f),
        make_world_point(4, 1225.0f, 2225.0f, 430.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{1000.0f, 2000.0f},
        500.0f,
        gripper_from_base_link,
        config);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    const auto local_center = compute_local_group_center(
        bind_area_entries.front(),
        gripper_from_base_link);
    EXPECT_FLOAT_EQ(config.tcp_max_x_mm, 380.0f);
    EXPECT_FLOAT_EQ(config.tcp_max_y_mm, 330.0f);
    EXPECT_NEAR(local_center[0], config.tcp_max_x_mm * 0.5f, 1e-3f);
    EXPECT_NEAR(local_center[1], config.tcp_max_y_mm * 0.5f, 1e-3f);
    EXPECT_NEAR(local_center[2], config.tcp_max_z_mm * 0.5f, 1e-3f);
}

TEST(DynamicBindPlanningTest, IgnoresLegacyTemplateOffsetWhenCenteringGroupInWorkspace)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();
    DynamicBindPlannerConfig config;
    config.template_center_x_mm = 10.0f;
    config.template_center_y_mm = 20.0f;
    config.template_center_z_mm = 30.0f;

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 1075.0f, 2075.0f, 430.0f),
        make_world_point(2, 1225.0f, 2075.0f, 430.0f),
        make_world_point(3, 1075.0f, 2225.0f, 430.0f),
        make_world_point(4, 1225.0f, 2225.0f, 430.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{1000.0f, 2000.0f},
        500.0f,
        gripper_from_base_link,
        config);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    expect_area_group_center_matches_workspace_center(
        bind_area_entries.front(),
        gripper_from_base_link,
        config);
}

TEST(DynamicBindPlanningTest, KeepsTwoByTwoGroupWhenMinimumCabinHeightClampsPose)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();
    DynamicBindPlannerConfig config;
    config.bind_execution_cabin_min_z_mm = 485.0f;

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 1075.0f, 2075.0f, 390.0f),
        make_world_point(2, 1225.0f, 2075.0f, 390.0f),
        make_world_point(3, 1075.0f, 2225.0f, 390.0f),
        make_world_point(4, 1225.0f, 2225.0f, 390.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{1000.0f, 2000.0f},
        500.0f,
        gripper_from_base_link,
        config);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries.front()), (std::vector<int>{1, 2, 3, 4}));
    EXPECT_FLOAT_EQ(bind_area_entries.front().cabin_z, config.bind_execution_cabin_min_z_mm);
}

TEST(DynamicBindPlanningTest, TraversesGridAsTwoColumnBandsFromWorldMinimumPoint)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    for (int row = 3; row >= 0; --row) {
        for (int col = 3; col >= 0; --col) {
            const int idx = row * 4 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                100.0f + static_cast<float>(col) * 150.0f,
                200.0f + static_cast<float>(row) * 150.0f,
                430.0f));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{100.0f, 200.0f},
        500.0f,
        gripper_from_base_link);

    const auto cells_by_index = build_linear_cell_map(4, 4);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), planning_world_points.size());
}

TEST(DynamicBindPlanningTest, UsesProvidedGridIndicesInsteadOfReclusteringTiltedWorldRows)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            const int idx = row * 4 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                100.0f + static_cast<float>(col) * 150.0f,
                200.0f + static_cast<float>(row) * 150.0f + static_cast<float>(col) * 55.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{100.0f, 200.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_FALSE(bind_area_entries.empty());
    const auto cells_by_index = build_cell_map(grid_indices);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), planning_world_points.size());
}

TEST(DynamicBindPlanningTest, StartsProvidedGridTraversalFromMinimumWorldCoordinatePoint)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int world_row = 0; world_row < 4; ++world_row) {
        for (int world_col = 0; world_col < 4; ++world_col) {
            const int idx = world_row * 4 + world_col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                100.0f + static_cast<float>(world_col) * 150.0f,
                200.0f + static_cast<float>(world_row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, 3 - world_row, 3 - world_col));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{100.0f, 200.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_FALSE(bind_area_entries.empty());
    ASSERT_FALSE(bind_area_entries.front().bind_groups.empty());
    ASSERT_FALSE(bind_area_entries.front().bind_groups.front().bind_points_world.empty());
    EXPECT_EQ(bind_area_entries.front().bind_groups.front().bind_points_world.front().idx, 1);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries.front()).front(), 1);
}

TEST(DynamicBindPlanningTest, PartitionsProvidedGridIntoFixedTwoByTwoBlocks)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            const int idx = row * 4 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 4u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 5, 6}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{3, 4, 7, 8}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[2]), (std::vector<int>{11, 12, 15, 16}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[3]), (std::vector<int>{9, 10, 13, 14}));
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        EXPECT_EQ(area_entry.bind_groups.front().group_type, "matrix_2x2");
    }
}

TEST(DynamicBindPlanningTest, PartitionsProvidedGridIntoRequestedTwoByThreeBlocksWhenSixPointsFit)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 6; ++col) {
            const int idx = row * 6 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    DynamicBindPlannerConfig config;
    config.requested_group_point_count = 6;

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        config,
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 7, 8, 9}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{4, 5, 6, 10, 11, 12}));
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        EXPECT_EQ(area_entry.bind_groups.front().group_type, "matrix_2x3");
    }
}

TEST(DynamicBindPlanningTest, UsesSmallerGroupsInsteadOfSwitchingSixPointOrientation)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 2; ++col) {
            const int idx = row * 2 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    DynamicBindPlannerConfig config;
    config.requested_group_point_count = 6;

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        config,
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 4}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{5, 6}));
    EXPECT_EQ(bind_area_entries[0].bind_groups.front().group_type, "matrix_2x2");
    EXPECT_EQ(bind_area_entries[1].bind_groups.front().group_type, "matrix_1x2");
}

TEST(DynamicBindPlanningTest, FillsSixPointModeRemainderWithSmallerReachableGroups)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 5; ++col) {
            const int idx = row * 5 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    DynamicBindPlannerConfig config;
    config.requested_group_point_count = 6;

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        config,
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 6, 7, 8}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{4, 5, 9, 10}));
    EXPECT_EQ(bind_area_entries[0].bind_groups.front().group_type, "matrix_2x3");
    EXPECT_EQ(bind_area_entries[1].bind_groups.front().group_type, "matrix_2x2");
}

TEST(DynamicBindPlanningTest, FillsNinePointModeWithSmallerGroupsWhenFullGroupCannotReachAllPoints)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int idx = row * 3 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 190.0f,
                static_cast<float>(row) * 190.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    DynamicBindPlannerConfig config;
    config.requested_group_point_count = 9;

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        config,
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 4, 5, 6}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{7, 8, 9}));
    EXPECT_EQ(bind_area_entries[0].bind_groups.front().group_type, "matrix_2x3");
    EXPECT_EQ(bind_area_entries[1].bind_groups.front().group_type, "matrix_1x3");
}

TEST(DynamicBindPlanningTest, FallsBackAroundWorldMinimumBeforeLaterReachableNinePointGroups)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 6; ++col) {
            const int idx = row * 6 + col + 1;
            const bool bottom_left_block_row = row == 2 && col < 3;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                bottom_left_block_row ? 250.0f : 430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    DynamicBindPlannerConfig config;
    config.requested_group_point_count = 9;

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        config,
        grid_indices);

    ASSERT_GE(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 7, 8, 9}));
    EXPECT_EQ(bind_area_entries[0].bind_groups.front().group_type, "matrix_2x3");
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{4, 5, 6, 10, 11, 12}));
    EXPECT_EQ(bind_area_entries[1].bind_groups.front().group_type, "matrix_2x3");
}

TEST(DynamicBindPlanningTest, TraversesProvidedGridAsSnakeRowsAlongPositiveX)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 6; ++col) {
            const int idx = row * 6 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 6u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 7, 8}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{3, 4, 9, 10}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[2]), (std::vector<int>{5, 6, 11, 12}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[3]), (std::vector<int>{17, 18, 23, 24}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[4]), (std::vector<int>{15, 16, 21, 22}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[5]), (std::vector<int>{13, 14, 19, 20}));
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        EXPECT_EQ(area_entry.bind_groups.front().group_type, "matrix_2x2");
    }
}

TEST(DynamicBindPlanningTest, KeepsProvidedOddGridRemainderPairsOnlyOnOuterEdges)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int idx = row * 3 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 3u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 4, 5}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{3, 6}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[2]), (std::vector<int>{7, 8}));
    EXPECT_EQ(bind_area_entries[0].bind_groups.front().group_type, "matrix_2x2");
    EXPECT_EQ(bind_area_entries[1].bind_groups.front().group_type, "matrix_2x2_edge_pair");
    EXPECT_EQ(bind_area_entries[2].bind_groups.front().group_type, "matrix_2x2_edge_pair");
}

TEST(DynamicBindPlanningTest, KeepsOddGridEdgesAsTwoPointGroupsWithoutOneOrThreePointGroups)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int idx = row * 3 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                -300.0f + static_cast<float>(col) * 150.0f,
                500.0f + static_cast<float>(row) * 150.0f,
                430.0f));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{-300.0f, 500.0f},
        485.0f,
        gripper_from_base_link);

    ASSERT_FALSE(bind_area_entries.empty());
    const auto cells_by_index = build_linear_cell_map(3, 3);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), 8u);
}

TEST(DynamicBindPlanningTest, DoesNotCreateInteriorPairsForPartialProvidedGridBlocks)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(101, -900.0f, 1000.0f, 430.0f),
        make_world_point(102, -750.0f, 1010.0f, 430.0f),
        make_world_point(103, -895.0f, 1160.0f, 430.0f),
        make_world_point(104, -745.0f, 1170.0f, 430.0f),
        make_world_point(7, -632.7f, 1672.7f, 430.0f),
        make_world_point(23, -445.3f, 1686.7f, 430.0f),
        make_world_point(20, -463.5f, 2142.9f, 430.0f),
        make_world_point(3, -636.9f, 2273.1f, 430.0f),
        make_world_point(19, -461.1f, 2286.7f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(101, 0, 0),
        make_grid_index(102, 0, 1),
        make_grid_index(103, 1, 0),
        make_grid_index(104, 1, 1),
        make_grid_index(7, 9, 0),
        make_grid_index(23, 9, 1),
        make_grid_index(20, 12, 1),
        make_grid_index(3, 13, 0),
        make_grid_index(19, 13, 1),
    };
    const auto cells_by_index = build_cell_map(grid_indices);

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{-636.9f, 1672.7f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        const auto& bind_group = area_entry.bind_groups.front();
        ASSERT_EQ(bind_group.bind_points_world.size(), 4u);
        for (const auto& point : bind_group.bind_points_world) {
            ASSERT_TRUE(cells_by_index.count(point.idx) > 0);
            EXPECT_TRUE(emitted_indices.insert(point.idx).second);
        }
        expect_group_uses_adjacent_checkerboard_cells(collect_area_point_indices(area_entry), cells_by_index);
    }
    EXPECT_EQ(emitted_indices.size(), 4u);
    EXPECT_TRUE(emitted_indices.count(101) > 0);
    EXPECT_TRUE(emitted_indices.count(102) > 0);
    EXPECT_TRUE(emitted_indices.count(103) > 0);
    EXPECT_TRUE(emitted_indices.count(104) > 0);
    EXPECT_EQ(emitted_indices.count(7), 0u);
    EXPECT_EQ(emitted_indices.count(23), 0u);
}

TEST(DynamicBindPlanningTest, DoesNotPairAdjacentPointsAcrossFixedTwoByTwoBlocks)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 0.0f, 0.0f, 430.0f),
        make_world_point(2, 150.0f, 0.0f, 430.0f),
        make_world_point(3, 0.0f, 150.0f, 430.0f),
        make_world_point(4, 150.0f, 150.0f, 430.0f),
        make_world_point(5, 0.0f, 300.0f, 430.0f),
        make_world_point(6, 150.0f, 600.0f, 430.0f),
        make_world_point(7, 300.0f, 600.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 1, 0),
        make_grid_index(4, 1, 1),
        make_grid_index(5, 2, 0),
        make_grid_index(6, 4, 1),
        make_grid_index(7, 4, 2),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(bind_area_entries.size(), 1u);
    EXPECT_EQ(emitted_indices.size(), 4u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries.front()), (std::vector<int>{1, 2, 3, 4}));
    EXPECT_EQ(emitted_indices.count(6), 0u);
    EXPECT_EQ(emitted_indices.count(7), 0u);
}

TEST(DynamicBindPlanningTest, RecoversPhysicallyAdjacentLeftoversFromDuplicateGridCellGap)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 0.0f, 0.0f, 430.0f),
        make_world_point(2, 150.0f, 0.0f, 430.0f),
        make_world_point(3, 0.0f, 150.0f, 430.0f),
        make_world_point(4, 150.0f, 150.0f, 430.0f),
        make_world_point(5, 0.0f, 300.0f, 430.0f),
        make_world_point(6, 150.0f, 300.0f, 430.0f),
        make_world_point(7, 0.0f, 450.0f, 430.0f),
        make_world_point(8, 150.0f, 450.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 1, 0),
        make_grid_index(4, 1, 1),
        make_grid_index(5, 1, 0),
        make_grid_index(6, 2, 1),
        make_grid_index(7, 3, 0),
        make_grid_index(8, 3, 1),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    std::set<int> emitted_indices;
    bool recovered_duplicate_gap_square = false;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
        if (point_indices == (std::vector<int>{5, 6, 7, 8})) {
            recovered_duplicate_gap_square = true;
        }
    }

    EXPECT_EQ(emitted_indices.size(), planning_world_points.size());
    EXPECT_TRUE(recovered_duplicate_gap_square);
}

TEST(DynamicBindPlanningTest, SkipsInteriorLeftoversAcrossFixedTwoByTwoBlocks)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 0.0f, 0.0f, 430.0f),
        make_world_point(2, 150.0f, 0.0f, 430.0f),
        make_world_point(3, 0.0f, 300.0f, 430.0f),
        make_world_point(4, 150.0f, 300.0f, 430.0f),
        make_world_point(5, 300.0f, 300.0f, 430.0f),
        make_world_point(6, 150.0f, 450.0f, 430.0f),
        make_world_point(7, 300.0f, 450.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 1, 0),
        make_grid_index(4, 1, 1),
        make_grid_index(5, 1, 2),
        make_grid_index(6, 2, 1),
        make_grid_index(7, 2, 2),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    const auto cells_by_index = build_cell_map(grid_indices);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), 4u);
    EXPECT_EQ(emitted_indices.count(5), 0u);
    EXPECT_EQ(emitted_indices.count(6), 0u);
    EXPECT_EQ(emitted_indices.count(7), 0u);
}

TEST(DynamicBindPlanningTest, SkipsAdjacentPointsInsideIncompleteFixedBlock)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 0.0f, 0.0f, 430.0f),
        make_world_point(2, 150.0f, 0.0f, 430.0f),
        make_world_point(3, 0.0f, 150.0f, 430.0f),
        make_world_point(4, 150.0f, 150.0f, 430.0f),
        make_world_point(5, 0.0f, 300.0f, 430.0f),
        make_world_point(6, 150.0f, 300.0f, 430.0f),
        make_world_point(7, 300.0f, 300.0f, 430.0f),
        make_world_point(8, 450.0f, 300.0f, 430.0f),
        make_world_point(9, 300.0f, 450.0f, 430.0f),
        make_world_point(10, 450.0f, 450.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 1, 0),
        make_grid_index(4, 1, 1),
        make_grid_index(5, 2, 0),
        make_grid_index(6, 2, 1),
        make_grid_index(7, 2, 2),
        make_grid_index(8, 2, 3),
        make_grid_index(9, 3, 2),
        make_grid_index(10, 3, 3),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    const auto cells_by_index = build_cell_map(grid_indices);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), 8u);
    EXPECT_EQ(emitted_indices.count(5), 0u);
    EXPECT_EQ(emitted_indices.count(6), 0u);
}

TEST(DynamicBindPlanningTest, ChoosesRegularDuplicateCellPointsForTwoByTwoWindow)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 0.0f, 0.0f, 430.0f),
        make_world_point(2, 150.0f, 0.0f, 430.0f),
        make_world_point(3, 0.0f, 300.0f, 430.0f),
        make_world_point(4, 0.0f, 150.0f, 430.0f),
        make_world_point(5, 150.0f, 300.0f, 430.0f),
        make_world_point(6, 150.0f, 150.0f, 430.0f),
    };
    const std::vector<DynamicBindGridIndex> grid_indices = {
        make_grid_index(1, 0, 0),
        make_grid_index(2, 0, 1),
        make_grid_index(3, 1, 0),
        make_grid_index(4, 1, 0),
        make_grid_index(5, 1, 1),
        make_grid_index(6, 1, 1),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 1u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries.front()), (std::vector<int>{1, 2, 4, 6}));
}

TEST(DynamicBindPlanningTest, KeepsSparseFieldWithinCompleteFixedTwoByTwoBlocks)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();
    const std::vector<std::string> occupied_rows = {
        "X.XXXXXXXXXXXXXX",
        "XXXXXXX.X.XXXXXX",
        "XXXXXXXX.XXXXXXX",
        "XXX.XXXXXX.XXXXX",
        "XXX.XXXXXXXX.XXX",
        "XXXXXX.X.XXXXXXX",
        "XXXXXXX.XXXXXXXX",
        "XXXXX.XXXXXXXXXX",
        "XXXXXX.XXXXXXXXX",
        "XXXX...XXXXXXXXX",
        "X.XXXXXXXXXXXXXX",
        "XXXXXXXXXXXXXXX.",
        "....XXXXXXXXXXX.",
        ".XXXXXXXXX.XXXX.",
        "XXXXX..XXXXXXXX",
        "XXXXX...XXXXXXXX",
    };

    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int row = 0; row < static_cast<int>(occupied_rows.size()); ++row) {
        for (int col = 0; col < static_cast<int>(occupied_rows[static_cast<size_t>(row)].size()); ++col) {
            if (occupied_rows[static_cast<size_t>(row)][static_cast<size_t>(col)] != 'X') {
                continue;
            }
            const int idx = row * 16 + col + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(col) * 150.0f,
                static_cast<float>(row) * 150.0f,
                430.0f));
            grid_indices.push_back(make_grid_index(idx, row, col));
        }
    }
    const auto cells_by_index = build_cell_map(grid_indices);

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        const auto point_indices = collect_area_point_indices(area_entry);
        ASSERT_EQ(point_indices.size(), 4u);
        for (const int point_idx : point_indices) {
            ASSERT_TRUE(cells_by_index.count(point_idx) > 0);
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
        expect_group_uses_adjacent_checkerboard_cells(point_indices, cells_by_index);
    }
    EXPECT_EQ(emitted_indices.size(), 164u);
}

TEST(DynamicBindPlanningTest, TraversesProvidedImageGridSnakeRowsAlongWorldPositiveX)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();
    std::vector<tie_robot_msgs::PointCoords> planning_world_points;
    std::vector<DynamicBindGridIndex> grid_indices;
    for (int world_y_band = 0; world_y_band < 4; ++world_y_band) {
        for (int world_x_band = 0; world_x_band < 6; ++world_x_band) {
            const int idx = world_y_band * 6 + world_x_band + 1;
            planning_world_points.push_back(make_world_point(
                idx,
                static_cast<float>(world_x_band) * 150.0f,
                static_cast<float>(world_y_band) * 150.0f,
                430.0f));

            grid_indices.push_back(make_grid_index(
                idx,
                world_x_band,
                3 - world_y_band));
        }
    }

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{0.0f, 0.0f},
        500.0f,
        gripper_from_base_link,
        DynamicBindPlannerConfig{},
        grid_indices);

    ASSERT_EQ(bind_area_entries.size(), 6u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 7, 8}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{3, 4, 9, 10}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[2]), (std::vector<int>{5, 6, 11, 12}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[3]), (std::vector<int>{17, 18, 23, 24}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[4]), (std::vector<int>{15, 16, 21, 22}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[5]), (std::vector<int>{13, 14, 19, 20}));

    const auto first_group_min_xy = compute_group_min_world_xy(bind_area_entries.front());
    EXPECT_FLOAT_EQ(first_group_min_xy[0], 0.0f);
    EXPECT_FLOAT_EQ(first_group_min_xy[1], 0.0f);
}

TEST(DynamicBindPlanningTest, SkipsIrregularFourPointShapeThatIsNotATwoByTwoMatrix)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, -449.0f, 598.0f, 416.0f),
        make_world_point(2, -449.0f, 751.0f, 420.0f),
        make_world_point(3, -150.0f, 890.0f, 423.0f),
        make_world_point(4, -299.0f, 747.0f, 428.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{-204.33333f, 686.33331f},
        489.66666f,
        gripper_from_base_link);

    EXPECT_TRUE(bind_area_entries.empty());
}

}  // namespace
}  // namespace planning
}  // namespace tie_robot_process
