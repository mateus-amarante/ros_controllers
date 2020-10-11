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

#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>

class DummyRobot : public hardware_interface::RobotHW
{
public:
    DummyRobot()
    {
        for (size_t i = 0; i < 3; i++)
        {
            const std::string joint_name = "joint" + std::to_string(i);
            // joint state interface
            hardware_interface::JointStateHandle js_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
            js_iface_.registerHandle(js_handle);

            // joint command interface
            hardware_interface::JointHandle pos_handle(js_handle, &cmd_[i]);
            cmd_iface_.registerHandle(pos_handle);
        }
        registerInterface(&js_iface_);
        registerInterface(&cmd_iface_);
    }

    void write(const ros::Time &, const ros::Duration &) override
    {
        for (size_t i = 0; i < 3; i++)
        {
            pos_[i] = cmd_[i];
        }
    }

private:
    double pos_[3] = {0, 0, 0};
    double vel_[3] = {0, 0, 0};
    double eff_[3] = {0, 0, 0};
    double cmd_[3] = {0, 0, 0};

    hardware_interface::JointStateInterface js_iface_;
    hardware_interface::PositionJointInterface cmd_iface_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_robot");

    DummyRobot robot;
    controller_manager::ControllerManager cm(&robot, ros::NodeHandle());

    ros::Rate rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // main loop
    while (ros::ok())
    {
        robot.read(ros::Time::now(), ros::Duration());
        cm.update(ros::Time::now(), rate.cycleTime());
        robot.write(ros::Time::now(), ros::Duration());

        rate.sleep();
    }
    spinner.stop();

    return 0;
}
