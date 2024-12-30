#include <gtest/gtest.h>
#include "climb_robot_driver/hardware_interfaces/dynamixel_interface.hpp"
#include "climb_robot_driver/hardware_interfaces/dummy_interface.hpp"

class HardwareTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dxl_ = std::make_unique<DynamixelInterface>();
    dxl_->addActuators({1, 2, 3}, {"j1", "j2", "j2"}, "model1", 0.5);
    dxl_->addActuators({4, 5, 6}, {"j3", "j4", "j4"}, "model2", 2.0);
    dxl_->removeActuators({1});
    dxl_->removeJoints({"j4"});
    dxl_->setRatios({2}, 5.0);

    dummy_ = std::make_unique<DummyInterface>();
    dummy_->addActuators({1, 2, 3}, {"j1", "j2", "j2"}, "model1", 0.5);
    dummy_->addActuators({4, 5, 6}, {"j3", "j4", "j4"}, "model2", 2.0);
    dummy_->removeActuators({1});
    dummy_->removeJoints({"j4"});
    dummy_->setRatios({2}, 5.0);
    dummy_->connect();
    dummy_->enable();
  }

  std::unique_ptr<HardwareInterface> dxl_;
  std::unique_ptr<HardwareInterface> dummy_;
};

TEST_F(HardwareTest, Configuration)
{
  EXPECT_EQ(dxl_->getIds(), std::vector<int>({2, 3, 4}));
  EXPECT_EQ(dxl_->getId("j2"), std::vector<int>({2, 3}));
  EXPECT_EQ(dxl_->getId("j1"), std::vector<int>({}));
  EXPECT_EQ(dxl_->getJoint(4), "j3");
  EXPECT_EQ(dxl_->getJoint(5), "");
  EXPECT_EQ(dxl_->getActuator(2), "model1");
  EXPECT_EQ(dxl_->getRatio(2), 5);
  EXPECT_EQ(dxl_->getRatio(3), 0.5);
  EXPECT_EQ(dxl_->getRatio(1), 0);
}

TEST_F(HardwareTest, DynamixelReadWrite)
{
  JointCommand command;
  command.name = {"j2", "j3"};
  command.mode = {JointCommand::MODE_POSITION, JointCommand::MODE_POSITION};
  command.position = {1.0, 2.0};
  EXPECT_FALSE(dxl_->writeJointCommand(command));

  auto joint_state = dxl_->readJointState();
  EXPECT_EQ(joint_state.name, std::vector<std::string>({"j2", "j3"}));
  EXPECT_EQ(joint_state.position, std::vector<double>({}));

  auto actuator_state = dxl_->readActuatorState();
  EXPECT_EQ(actuator_state.id, std::vector<int>({2, 3, 4}));
  EXPECT_EQ(actuator_state.joint, std::vector<std::string>({"j2", "j2", "j3"}));
  EXPECT_EQ(actuator_state.voltage, std::vector<double>({}));
}

TEST_F(HardwareTest, DummyReadWrite)
{
  JointCommand command;
  command.name = {"j2", "j3"};
  command.mode = {JointCommand::MODE_POSITION, JointCommand::MODE_POSITION};
  command.position = {1.0, 2.0};
  command.effort = {3.0, 4.0};
  EXPECT_TRUE(dummy_->writeJointCommand(command));

  auto joint_state = dummy_->readJointState();
  EXPECT_EQ(joint_state.name, std::vector<std::string>({"j2", "j3"}));
  EXPECT_EQ(joint_state.position, std::vector<double>({1.0, 2.0}));
  EXPECT_EQ(joint_state.velocity, std::vector<double>({0, 0}));
  EXPECT_EQ(joint_state.effort, std::vector<double>({3.0, 4.0}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
