#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
    addWallsAndBase();
  }

private:
  void addWallsAndBase() {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(5);

    // Size for the walls
    double wall_height = 1.1;  // 2 meter high
    double wall_thickness = 0.01;  // 1 cm thick
    double wall_width = 1.4;
    double wall_distance = 0.73;  // 0.8 meter away from base

    // Size for the base
    double base_height = 0.01;  // 1 cm thick
    double base_side = 1.4;  // 1 meter radius, should be larger than reach of robot

    // Size for the tool box
    double box_height = 0.75;
    double box_thickness = 0.08;
    double box_width = 0.12;

    for (int i = 0; i < 3; i++) {
      collision_objects[i].header.frame_id = move_group_interface_.getPlanningFrame();
      collision_objects[i].id = "wall" + std::to_string(i+1);

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = (i < 2) ? wall_thickness : wall_width;
      primitive.dimensions[primitive.BOX_Y] = (i < 2) ? wall_width + 0.02 : wall_thickness;
      primitive.dimensions[primitive.BOX_Z] = wall_height;

      collision_objects[i].primitives.push_back(primitive);
      collision_objects[i].primitive_poses.resize(1);
      collision_objects[i].primitive_poses[0].orientation.w = 1.0;
      collision_objects[i].primitive_poses[0].position.y = (i < 2) ? 0.25 : -0.46;
      collision_objects[i].primitive_poses[0].position.x = (i < 2) ? ((i % 2 == 0) ? wall_distance : -wall_distance+0.06) : 0.03;
      collision_objects[i].primitive_poses[0].position.z = wall_height / 2;
      collision_objects[i].operation = collision_objects[i].ADD;


    }

    // Add the base
    collision_objects[3].header.frame_id = move_group_interface_.getPlanningFrame();
    collision_objects[3].id = "base";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = base_side;
    primitive.dimensions[primitive.BOX_Y] = base_side + 0.02;
    primitive.dimensions[primitive.BOX_Z] = base_height;

    collision_objects[3].primitives.push_back(primitive);
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    collision_objects[3].primitive_poses[0].position.x = 0.03;
    collision_objects[3].primitive_poses[0].position.y = 0.25;
    collision_objects[3].primitive_poses[0].position.z = -base_height;
    collision_objects[3].operation = collision_objects[3].ADD;


    // Add the ceiling
    collision_objects[4].header.frame_id = move_group_interface_.getPlanningFrame();
    collision_objects[4].id = "ceiling";

    shape_msgs::msg::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[primitive1.BOX_X] = base_side;
    primitive1.dimensions[primitive1.BOX_Y] = base_side + 0.02;
    primitive1.dimensions[primitive1.BOX_Z] = base_height;

    collision_objects[4].primitives.push_back(primitive1);
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].orientation.w = 1.0;
    collision_objects[4].primitive_poses[0].position.x = 0.03;
    collision_objects[4].primitive_poses[0].position.y = 0.25;
    collision_objects[4].primitive_poses[0].position.z = wall_height;
    collision_objects[4].operation = collision_objects[4].ADD;


/*
    // Add tool box
    collision_objects[5].header.frame_id = move_group_interface_.getPlanningFrame();
    collision_objects[5].id = "tool box";

    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive2.BOX_X] = box_width;
    primitive2.dimensions[primitive2.BOX_Y] = box_thickness;
    primitive2.dimensions[primitive2.BOX_Z] = box_height;

    collision_objects[5].primitives.push_back(primitive2);
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].orientation.w = 1.0;
    collision_objects[5].primitive_poses[0].position.y = 0.67;
    collision_objects[5].primitive_poses[0].position.x = 0.55;
    collision_objects[5].primitive_poses[0].position.z = box_height / 2;
    collision_objects[5].operation = collision_objects[5].ADD;
*/



    // Now, let's add the collision object into the world
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_name = "ur_manipulator";
  
  auto moveit_node = std::make_shared<UR10EMoveit>(node, move_group_name);
  rclcpp::spin(moveit_node);
  rclcpp::shutdown();
  return 0;
}
