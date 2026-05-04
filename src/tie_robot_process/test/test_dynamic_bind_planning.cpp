#include <gtest/gtest.h>

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

    ASSERT_EQ(bind_area_entries.size(), 4u);
    EXPECT_EQ(collect_area_grid_cells(bind_area_entries[0], 4), (std::vector<std::tuple<int, int, int>>{
                                                               {1, 0, 0}, {2, 0, 1}, {5, 1, 0}, {6, 1, 1}}));
    EXPECT_EQ(collect_area_grid_cells(bind_area_entries[1], 4), (std::vector<std::tuple<int, int, int>>{
                                                               {9, 2, 0}, {10, 2, 1}, {13, 3, 0}, {14, 3, 1}}));
    EXPECT_EQ(collect_area_grid_cells(bind_area_entries[2], 4), (std::vector<std::tuple<int, int, int>>{
                                                               {15, 3, 2}, {16, 3, 3}, {11, 2, 2}, {12, 2, 3}}));
    EXPECT_EQ(collect_area_grid_cells(bind_area_entries[3], 4), (std::vector<std::tuple<int, int, int>>{
                                                               {7, 1, 2}, {8, 1, 3}, {3, 0, 2}, {4, 0, 3}}));
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

    ASSERT_EQ(bind_area_entries.size(), 4u);
    for (const auto& area_entry : bind_area_entries) {
        EXPECT_EQ(collect_area_point_indices(area_entry).size(), 4u);
        EXPECT_EQ(area_entry.bind_groups.front().group_type, "matrix_2x2");
    }
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 5, 6}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{9, 10, 13, 14}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[2]), (std::vector<int>{15, 16, 11, 12}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[3]), (std::vector<int>{7, 8, 3, 4}));
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

    ASSERT_EQ(bind_area_entries.size(), 3u);

    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        EXPECT_TRUE(point_indices.size() == 4u || point_indices.size() == 2u);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_EQ(emitted_indices.size(), 8u);
}

TEST(DynamicBindPlanningTest, DoesNotCreateDiagonalPairsOrDuplicatePointsForPartialBlocks)
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

    ASSERT_EQ(bind_area_entries.size(), 3u);
    std::set<int> emitted_indices;
    for (const auto& area_entry : bind_area_entries) {
        ASSERT_EQ(area_entry.bind_groups.size(), 1u);
        const auto& bind_group = area_entry.bind_groups.front();
        ASSERT_TRUE(bind_group.bind_points_world.size() == 2u || bind_group.bind_points_world.size() == 4u);
        if (bind_group.bind_points_world.size() == 4u) {
            for (const auto& point : bind_group.bind_points_world) {
                EXPECT_TRUE(emitted_indices.insert(point.idx).second);
            }
            continue;
        }
        const int first_index = bind_group.bind_points_world[0].idx;
        const int second_index = bind_group.bind_points_world[1].idx;
        ASSERT_TRUE(cells_by_index.count(first_index) > 0);
        ASSERT_TRUE(cells_by_index.count(second_index) > 0);
        const auto first_cell = cells_by_index.at(first_index);
        const auto second_cell = cells_by_index.at(second_index);
        const int row_delta = std::abs(first_cell.first - second_cell.first);
        const int column_delta = std::abs(first_cell.second - second_cell.second);
        EXPECT_TRUE((row_delta == 0 && column_delta == 1) || (row_delta == 1 && column_delta == 0));
        EXPECT_TRUE(emitted_indices.insert(first_index).second);
        EXPECT_TRUE(emitted_indices.insert(second_index).second);
    }
    EXPECT_EQ(emitted_indices.size(), 8u);
    EXPECT_TRUE(emitted_indices.count(101) > 0);
    EXPECT_TRUE(emitted_indices.count(102) > 0);
    EXPECT_TRUE(emitted_indices.count(103) > 0);
    EXPECT_TRUE(emitted_indices.count(104) > 0);
}

TEST(DynamicBindPlanningTest, PairsLaterPendingEdgePointsWhenEarlierSingleIsIsolated)
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

    ASSERT_EQ(bind_area_entries.size(), 2u);
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[0]), (std::vector<int>{1, 2, 3, 4}));
    EXPECT_EQ(collect_area_point_indices(bind_area_entries[1]), (std::vector<int>{6, 7}));
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

TEST(DynamicBindPlanningTest, UsesGeometricallyBetterOffsetTwoByTwoWindow)
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

    bool has_regular_offset_square = false;
    for (const auto& area_entry : bind_area_entries) {
        const auto point_indices = collect_area_point_indices(area_entry);
        if (point_indices == (std::vector<int>{4, 5, 6, 7})) {
            has_regular_offset_square = true;
        }
    }
    EXPECT_TRUE(has_regular_offset_square);
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

TEST(DynamicBindPlanningTest, CoversSparseFullFieldWithPairsInsteadOfLeavingGroupableCellsUnplanned)
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
        ASSERT_TRUE(point_indices.size() == 2u || point_indices.size() == 4u);
        for (const int point_idx : point_indices) {
            EXPECT_TRUE(emitted_indices.insert(point_idx).second);
        }
    }
    EXPECT_GE(emitted_indices.size(), 222u);
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
