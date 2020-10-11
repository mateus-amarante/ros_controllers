/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Mateus Amarante.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

int g_argc;
char **g_argv;

class PositionControllersTest : public testing::Test
{
public:
    void SetUp() override
    {
        ros::init(g_argc, g_argv, "position_controllers_test");

        ros::NodeHandle nh;

        joint_state_sub = nh.subscribe("joint_states", 1, &PositionControllersTest::jointStatesCallback, this);
        pos_cmd_pub = nh.advertise<std_msgs::Float64>("position_controller/command", 1);
        group_pos_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("group_position_controller/command", 1);

        ASSERT_TRUE(waitForSubscribers());
    }

    void TearDown() override
    {
        ros::shutdown();
        ros::waitForShutdown();
    }

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        joint_states = *msg;
    }

    bool waitForSubscribers(const ros::Duration &timeout = ros::Duration(5.))
    {
        while (ros::ok())
        {
            if (pos_cmd_pub.getNumSubscribers() > 0 && group_pos_cmd_pub.getNumSubscribers() > 0)
            {
                return true;
            }
            ros::Duration(0.01).sleep();
        }
        return false;
    }

    bool waitForJointPositionUpdate(size_t joint_idx, double expected_value, ros::Duration timeout = ros::Duration(1.))
    {
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time) < timeout)
        {
            if (joint_states.position.size() > 0 && std::fabs(joint_states.position[joint_idx] - expected_value) < 1e-8)
            {
                return true;
            }
            ros::spinOnce();
        }
        return false;
    }

    ros::Subscriber joint_state_sub;
    ros::Publisher pos_cmd_pub;
    ros::Publisher group_pos_cmd_pub;

    sensor_msgs::JointState joint_states;
};

TEST_F(PositionControllersTest, JointPositionController)
{
    std_msgs::Float64 cmd;
    cmd.data = 1.;

    pos_cmd_pub.publish(cmd);

    ASSERT_TRUE(waitForJointPositionUpdate(0, cmd.data));
}

TEST_F(PositionControllersTest, JointGroupPositionControllerValidInput)
{
    std_msgs::Float64MultiArray cmd;
    cmd.data.push_back(2.);
    cmd.data.push_back(3.);

    group_pos_cmd_pub.publish(cmd);
    ros::spinOnce();

    ASSERT_TRUE(waitForJointPositionUpdate(1, cmd.data[0]));
    ASSERT_TRUE(waitForJointPositionUpdate(2, cmd.data[1]));
}

TEST_F(PositionControllersTest, JointGroupPositionControllerInvalidInput)
{
    std_msgs::Float64MultiArray cmd;
    cmd.data.push_back(4.);

    group_pos_cmd_pub.publish(cmd);

    ASSERT_FALSE(waitForJointPositionUpdate(1, cmd.data[0]));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    g_argc = argc;
    g_argv = argv;
    ros::Time::init();
    return RUN_ALL_TESTS();
}
