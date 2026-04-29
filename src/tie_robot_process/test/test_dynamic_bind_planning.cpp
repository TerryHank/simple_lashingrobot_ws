#include <gtest/gtest.h>

#include <set>
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

std::set<int> collect_unique_indices(const PseudoSlamBindGroup& bind_group)
{
    std::set<int> unique_indices;
    for (const auto& point : bind_group.bind_points_world) {
        unique_indices.insert(point.idx);
    }
    return unique_indices;
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

TEST(DynamicBindPlanningTest, SkipsThreePointLShapeInsteadOfDuplicatingToFourPoints)
{
    const tf2::Transform gripper_from_base_link = make_gripper_from_base_link_transform();

    const std::vector<tie_robot_msgs::PointCoords> planning_world_points = {
        make_world_point(1, 308.0f, 575.0f, 411.0f),
        make_world_point(2, 608.0f, 569.0f, 411.0f),
        make_world_point(3, 458.0f, 721.0f, 412.0f),
    };

    const auto bind_area_entries = build_dynamic_bind_area_entries_from_scan_world(
        planning_world_points,
        CabinPoint{308.0f, 575.0f},
        485.0f,
        gripper_from_base_link);

    EXPECT_TRUE(bind_area_entries.empty());
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
