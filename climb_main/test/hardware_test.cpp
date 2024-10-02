#include <gtest/gtest.h>
#include "climb_main/hardware/dynamixel_interface.hpp"
#include "climb_main/hardware/hardware_interface.hpp"
#include "climb_msgs/msg/joint_command.hpp"

TEST(hardware, hardware_interface)
{
  std::unique_ptr<HardwareInterface> interface =
    std::make_unique<DynamixelInterface>();

  // Test hardware interface getters and setters

  interface->addActuators({1, 2, 3}, {"j1", "j2", "j2"}, "model1", 0.5);
  interface->addActuators({4, 5, 6}, {"j3", "j4", "j4"}, "model2", 2.0);
  interface->removeActuators({1});
  interface->removeJoints({"j4"});
  interface->setRatios({2}, 5.0);

  EXPECT_EQ(interface->getIds(), std::vector<int>({2, 3, 4}));
  EXPECT_EQ(interface->getId("j2"), std::vector<int>({2, 3}));
  EXPECT_EQ(interface->getId("j1"), std::vector<int>({}));
  EXPECT_EQ(interface->getJoint(4), "j3");
  EXPECT_EQ(interface->getJoint(5), "");
  EXPECT_EQ(interface->getActuator(2), "model1");
  EXPECT_EQ(interface->getRatio(2), 5);
  EXPECT_EQ(interface->getRatio(3), 0.5);
  EXPECT_EQ(interface->getRatio(1), 0);

  // Test joint command validation

  JointCommand cmd;
  EXPECT_EQ(interface->validateJointCommand(cmd), false)
    << "Empty command is invalid";
  cmd.name = {"j1", "j2"};
  cmd.mode = {JointCommand::MODE_POSITION, JointCommand::MODE_POSITION};
  cmd.position = {1.0, 2.0};
  EXPECT_EQ(interface->validateJointCommand(cmd), false)
    << "Joints must exist";
  cmd.name = {"j2", "j3"};
  EXPECT_EQ(interface->validateJointCommand(cmd), true)
    << "Velocity/effort limits are optional in position mode";
  cmd.mode = {JointCommand::MODE_VELOCITY, JointCommand::MODE_VELOCITY};
  EXPECT_EQ(interface->validateJointCommand(cmd), false)
    << "Velocity limits are required in velocity mode";
  cmd.velocity = {1.0, 2.0};
  cmd.position = {};
  EXPECT_EQ(interface->validateJointCommand(cmd), true)
    << "Position is optional in velocity mode";
  cmd.mode = {JointCommand::MODE_EFFORT, JointCommand::MODE_POSITION};
  cmd.effort = {1.0, 2.0};
  EXPECT_EQ(interface->validateJointCommand(cmd), false)
    << "Position is required in position mode";
  cmd.position = {1.0, 2.0};
  EXPECT_EQ(interface->validateJointCommand(cmd), true)
    << "Velocity is optional in effort mode";
  cmd.effort = {1.0};
  EXPECT_EQ(interface->validateJointCommand(cmd), false)
    << "Lengths must match";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
