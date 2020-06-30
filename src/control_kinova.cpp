/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
* DESCRIPTION OF CURRENT EXAMPLE:
* ===============================
* This example works as a simili-haptic demo.
*     
* The last actuator, the small one holding the interconnect, acts as a torque sensing device commanding the first actuator.
* The first actuator, the big one on the base, is controlled in torque and its position is sent as a command to the last one.
* 
* The script can be launched through command line with python3: python torqueControl_example.py
* The PC should be connected through ethernet with the arm. Default IP address 192.168.1.10 is used as arm address.
* 
* 1- Connection with the base:
*     1- A TCP session is started on port 10000 for most API calls. Refresh is at 25ms on this port.
*     2- A UDP session is started on port 10001 for BaseCyclic calls. Refresh is at 1ms on this port only.
* 2- Initialization
*     1- First frame is built based on arm feedback to ensure continuity
*     2- First actuator torque command is set as well
*     3- Base is set in low-level servoing
*     4- First frame is sent
*     3- First actuator is switched to torque mode
* 3- Cyclic thread is running at 1ms
*     1- Torque command to first actuator is set to a multiple of last actuator torque measure minus its initial value to
*        avoid an initial offset error
*     2- Position command to last actuator equals first actuator position minus initial delta
*     
* 4- On keyboard interrupt, example stops
*     1- Cyclic thread is stopped
*     2- First actuator is set back to position control
*     3- Base is set in single level servoing (default)
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <chrono>
#include <time.h>
#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;
const int SECOND = 1000000;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI
#define ACTUATOR_COUNT 7

float TIME_DURATION = 30.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
 
    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

//Make sure that the control loop runs exactly with the specified frequency
int enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

        return 0;
    }
    else return -1; //Loop is too slow
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }
}

bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    const int RATE_HZ = 600; // Hz
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    double total_time_sec = 0.0;
    double task_time_limit_sec = 60.0;
    int iteration_count = 0;
    int control_loop_delay_count = 0;
    int return_flag = 0;
    bool return_status = true;

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        // Save the current actuator position, to avoid a following error
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set last actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        int last_actuator_device_id = 7;
        actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);


        const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
        double desired_vel = 4.0; //deg/s
        double error = 0.0;
        double abag_command = 0.0;

        // Real-time loop
        while (total_time_sec < task_time_limit_sec)
        {
            loop_start_time = std::chrono::steady_clock::now();
            iteration_count++;
            total_time_sec = iteration_count * DT_SEC;

            try
            {
                base_feedback = base_cyclic->RefreshFeedback();
            }
            catch (Kinova::Api::KDetailedException& ex)
            {
                std::cout << "Kortex exception 1: " << ex.what() << std::endl;
            }

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

            error = desired_vel - base_feedback.actuators(6).velocity();
            printf("error: %f\n \n", error);
            // abag_command = abag.do_stuff(error);

            base_command.mutable_actuators(6)->set_position(base_feedback.actuators(6).position());

            // base_command.mutable_actuators(6)->set_torque_joint(abag_command * joint_torque_limits[6]);
            base_command.mutable_actuators(6)->set_torque_joint(1.0);

            // Incrementing identifier ensures actuators can reject out of time frames
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535) base_command.set_frame_id(0);

            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());



            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;

                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            // Enforce the constant loop time and count how many times the loop was late
            if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;
        }

        std::cout << "Torque control example completed" << std::endl;

        // Set first actuator back in position 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("kinova1_area4251");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    example_move_to_home_position(base);
    auto isOk = example_cyclic_torque_control(base, base_cyclic, actuator_config);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_torque_control() function." << endl;;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}