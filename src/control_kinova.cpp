/*
* Copyright (c) 2020 Minh Nguyen inc. All rights reserved.
*
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <chrono>
#include <time.h>

#include "kinova_util.h"
#include "abag.h"

#include <unistd.h>

namespace k_api = Kinova::Api;
namespace sc = std::chrono;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI

std::chrono::duration <double, std::micro> loop_interval{};

void example_cyclic_torque_control(KinovaBaseConnection &pBaseConnection, FILE * pLogFile)
{
    // Set last actuator in torque mode now that the command is equal to measure
    pBaseConnection.mCtrlModeInfo.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    pBaseConnection.mActuatorConfigClient->SetControlMode(pBaseConnection.mCtrlModeInfo, BASE_ID_CONFIG + JOINT_6_ID);

    const double alpha = 0.8;
    double desired_vel = 6.0, current_vel = 0.0; //deg/s
    double error = 0.0;
    double abag_command = 0.0;
    abagState_t abagState;
    initialize_abagState(&abagState);

    // Real-time loop
    const int SECOND_IN_MICROSECONDS = 1000000;
    const int RATE_HZ = 600; // Hz
    const int TASK_TIME_LIMIT_SEC = 30;
    const sc::microseconds TASK_TIME_LIMIT_MICRO(TASK_TIME_LIMIT_SEC * SECOND_IN_MICROSECONDS);
    const sc::microseconds LOOP_DURATION(SECOND_IN_MICROSECONDS / RATE_HZ);

    int slowLoopCount = 0;
    sc::time_point<sc::steady_clock> controlStartTime = sc::steady_clock::now();;
    sc::time_point<sc::steady_clock> loopStartTime;
    sc::duration<int64_t, nano> totalElapsedTime;
    while (loopStartTime - controlStartTime < TASK_TIME_LIMIT_MICRO)
    {
        loopStartTime = sc::steady_clock::now();
        totalElapsedTime = loopStartTime - controlStartTime;

        pBaseConnection.refreshFeedBack();

        // for (int i = 0; i < ACTUATOR_COUNT; i++)
        // {
        //     jnt_position(i) = DEG_TO_RAD(base_feedback.actuators(i).position()); //deg
        //     jnt_velocity(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity()); // deg
        //     jnt_torque(i)   = base_feedback.actuators(i).torque(); //nm
        // }

        // Kinova API provides only positive angle values
        // This operation is required to align the logic with our safety monitor
        // We need to convert some angles to negative values
        // if (jnt_position(1) > DEG_TO_RAD(180.0)) jnt_position(1) = jnt_position(1) - DEG_TO_RAD(360.0);
        // if (jnt_position(3) > DEG_TO_RAD(180.0)) jnt_position(3) = jnt_position(3) - DEG_TO_RAD(360.0);
        // if (jnt_position(5) > DEG_TO_RAD(180.0)) jnt_position(5) = jnt_position(5) - DEG_TO_RAD(360.0);

        // for (int i = 0; i < ACTUATOR_COUNT; i++)
        // {
        //     base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
        //     base_command.mutable_actuators(i)->set_torque_joint(jnt_command_torque(i));
        // }

        current_vel = pBaseConnection.mBaseFb.actuators(6).velocity();
        if (totalElapsedTime > TASK_TIME_LIMIT_MICRO * 0.1 && totalElapsedTime < TASK_TIME_LIMIT_MICRO * 0.9) {
            error = desired_vel - current_vel;
        } else {
            error = 0.0 - current_vel;
        }

        fprintf(pLogFile, "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
            error, abagState.signedErr_access, abagState.bias_access, abagState.gain_access,abagState.eBar_access,
            abag_command, current_vel);
        abag_sched(&abagState, &error, &abag_command, &alpha);

        pBaseConnection.setTorqueSingleJoint(JOINT_6_ID, abag_command);

        // Enforce the constant loop time and count how many times the loop was late
        if (waitMicroSeconds(loopStartTime, LOOP_DURATION) != 0) slowLoopCount++;
    }

    std::cout << "Torque control example completed" << std::endl;
    std::cout << "Number of loops which took longer than specified duration: " << slowLoopCount << std::endl;
    // Set first actuator back in position
    pBaseConnection.mCtrlModeInfo.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    pBaseConnection.mActuatorConfigClient->SetControlMode(pBaseConnection.mCtrlModeInfo, BASE_ID_CONFIG + JOINT_6_ID);
    std::cout << "Torque control example clean exit" << std::endl;

    // Set the servoing mode back to Single Level
    pBaseConnection.mServoingModeInfo.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    pBaseConnection.mBaseClient->SetServoingMode(pBaseConnection.mServoingModeInfo);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

int main(int argc, char **argv)
{
    FILE * fp = nullptr;
    try
    {
        KinovaBaseConnection connection(IP_ADDRESS, PORT, PORT_REAL_TIME, "admin", "kinova1_area4251");
        connection.moveToHomePosition();

        fp = fopen("simulated_data.csv", "w+");
        fprintf(fp, "error, signed, bias, gain, e_bar, e_bar_prev, actuation, currentVel\n");

        example_cyclic_torque_control(connection, fp);

        if (fp != nullptr) fclose(fp);
    }
    catch (k_api::KDetailedException& ex)
    {
        if (fp != nullptr) fclose(fp);
        handleKinovaException(ex);
        return EXIT_FAILURE;
    }
    catch (std::runtime_error& ex2)
    {
        if (fp != nullptr) fclose(fp);
        std::cout << "Error: " << ex2.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (std::exception &ex)
    {
        if (fp != nullptr) fclose(fp);
        std::cout << "Unknown Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}