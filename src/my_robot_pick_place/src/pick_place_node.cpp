#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node")
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    RCLCPP_INFO(get_logger(), "PickPlaceNode created");
  }

  void init()
  {
    RCLCPP_INFO(get_logger(), "Initializing MoveIt interfaces");

    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");

    gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");

    scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    arm_->setMaxVelocityScalingFactor(1.0);
    arm_->setMaxAccelerationScalingFactor(1.0);

    // Allow MoveIt, TF, controllers to sync
    rclcpp::sleep_for(2s);

    execute();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;

  /* ---------------- TASK SEQUENCE ---------------- */

  void execute()
  {
    add_table();
    add_object();
    rclcpp::sleep_for(1s);

    move_arm("home");
    open_gripper();

    move_arm("pose_1");
    close_gripper();

    attach_object();

    move_arm("pose_2");

    open_gripper();
    detach_object();

    move_arm("home");

    RCLCPP_INFO(get_logger(), "Pick & Place completed successfully");
  }

  /* ---------------- ARM / GRIPPER ---------------- */

  void move_arm(const std::string & name)
  {
    RCLCPP_INFO(get_logger(), "Moving arm to '%s'", name.c_str());
    arm_->setNamedTarget(name);
    arm_->move();
  }

  void open_gripper()
  {
    std::map<std::string, double> target;
    target["gripper_left_finger_joint"]  = 0.0;
    target["gripper_right_finger_joint"] = 0.0;

    gripper_->setJointValueTarget(target);
    gripper_->move();
  }

  void close_gripper()
  {
    std::map<std::string, double> target;
    target["gripper_left_finger_joint"]  = 0.05;
    target["gripper_right_finger_joint"] = -0.05;

    gripper_->setJointValueTarget(target);
    gripper_->move();
  }

  /* ---------------- COLLISION OBJECTS ---------------- */

  void add_table()
  {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.6, 1.0, 0.3};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.35;
    pose.position.y = 1.0;
    pose.position.z = 0.15;
    pose.orientation.w = 1.0;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;

    scene_->applyCollisionObject(table);
  }

  void add_object()
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "object";
    obj.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.05, 0.05, 0.06};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.35;
    pose.position.y = 1.0;
    pose.position.z = 0.33;
    pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;

    scene_->applyCollisionObject(obj);
  }

  /* ---------------- ATTACH / DETACH ---------------- */

  void attach_object()
  {
    RCLCPP_INFO(get_logger(), "Attaching object");

    // MUST remove from world first
    scene_->removeCollisionObjects({"object"});
    rclcpp::sleep_for(500ms);

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "gripper_base_link";
    attached.object.id = "object";
    attached.object.operation = attached.object.ADD;

    scene_->applyAttachedCollisionObject(attached);
  }

  void detach_object()
  {
    RCLCPP_INFO(get_logger(), "Detaching object");

    scene_->removeCollisionObjects(
      std::vector<std::string>{"object"});

    rclcpp::sleep_for(500ms);

    moveit_msgs::msg::CollisionObject obj;
    obj.id = "object";
    obj.header.frame_id = "world";
    obj.operation = obj.ADD;

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.35;
    pose.position.y = 1.0;
    pose.position.z = 0.33;
    pose.orientation.w = 1.0;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.05, 0.05, 0.06};

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);

    scene_->applyCollisionObject(obj);
  }
};

/* ---------------- MAIN ---------------- */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
