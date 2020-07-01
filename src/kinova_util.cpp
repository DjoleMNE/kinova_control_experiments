#include <chrono>
#include <KDetailedException.h>
#include "kinova_util.h"

KinovaBaseConnection::KinovaBaseConnection (
    std::string pHost, uint32_t pTcpPort, uint32_t pUdpPort, std::string pUsername, std::string pPassword
) {
    auto errorCallback = [](k_api::KError err){ cout << "__ callback error __" << err.toString(); };

    std::cout << "Creating TCP transport client" << std::endl;
    mTransportClientTcp = std::make_shared<k_api::TransportClientTcp>();
    mRouterTcp = std::make_shared<k_api::RouterClient>(mTransportClientTcp.get(), errorCallback);
    if (!mTransportClientTcp->connect(pHost, pTcpPort)) {
        std::stringstream errMsgStream;
        errMsgStream << "failed to create TCP client for host '" << pHost << "' on port '" << pTcpPort << "'\n";
        throw std::runtime_error(errMsgStream.str());
    }

    std::cout << "Creating UDP transport client (real time)" << std::endl;
    mTransportClientUdp = std::make_shared<k_api::TransportClientUdp>();
    mRouterUdp = std::make_shared<k_api::RouterClient>(mTransportClientUdp.get(), errorCallback);
    if (!mTransportClientUdp->connect(pHost, pUdpPort)) {
        std::stringstream errMsgStream;
        errMsgStream << "failed to create UDP client for host '" << pHost << "' on port '" << pUdpPort << "'\n";
        throw std::runtime_error(errMsgStream.str());
    }

    // Set session data connection information
    auto createSessionInfo = k_api::Session::CreateSessionInfo();
    createSessionInfo.set_username(pUsername);
    createSessionInfo.set_password(pPassword);
    createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
    createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    mSessionManagerTcp = std::make_shared<k_api::SessionManager>(mRouterTcp.get());
    mSessionManagerTcp->CreateSession(createSessionInfo);
    mSessionManagerUdp = std::make_shared<k_api::SessionManager>(mRouterUdp.get());
    mSessionManagerUdp->CreateSession(createSessionInfo);
    std::cout << "Sessions created" << std::endl;

    // Create services
    mBaseClient = std::make_shared<k_api::Base::BaseClient>(mRouterTcp.get());
    mBaseCyclicClient = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(mRouterUdp.get());
    mActuatorConfigClient = std::make_shared<k_api::ActuatorConfig::ActuatorConfigClient>(mRouterTcp.get());

    // Clearing faults
    try {
        mBaseClient->ClearFaults();
    } catch(...) {
        std::cerr << "error clearing robot faults" << std::endl;
        throw;
    }

    // Initialize each actuator to their current position
    // Save the current actuator position, to avoid a following error
    try {
        mServoingModeInfo.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        mBaseClient->SetServoingMode(mServoingModeInfo);

        mBaseFb = mBaseCyclicClient->RefreshFeedback();

        for (int i = BASE_ID_CONTROL; i < ACTUATOR_COUNT + BASE_ID_CONTROL; i++)
            mBaseCmd.add_actuators()->set_position(mBaseFb.actuators(i).position());

        // Send a first frame
        mBaseFb = mBaseCyclicClient->Refresh(mBaseCmd);

    } catch (...) {
        std::cerr << "error initializing joint positions" << std::endl;
        throw;
    }
}

KinovaBaseConnection::~KinovaBaseConnection()
{
    stopRobotMotion();

    // Close API session
    mSessionManagerTcp->CloseSession();
    mSessionManagerUdp->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    mRouterTcp->SetActivationStatus(false);
    mTransportClientTcp->disconnect();
    mRouterUdp->SetActivationStatus(false);
    mTransportClientUdp->disconnect();
}

void KinovaBaseConnection::refreshFeedBack() {
    try {
        mBaseFb = mBaseCyclicClient->RefreshFeedback();
    } catch (...) {
        std::cerr << "refreshFeedBack: error reading arm sensors" << std::endl;
    }
}

void KinovaBaseConnection::setTorqueSingleJoint(uint8_t pJointId, double pMaxTorqueFraction) {
    if (pJointId > ACTUATOR_COUNT - 1)
        throw runtime_error("setJointTorqueSingle: invalid Joint ID: " + std::to_string(pJointId));
    if (pMaxTorqueFraction > 1.0)
        throw runtime_error("setJointTorqueSingle: torque fraction cannot be > 1: " +
                            std::to_string(pMaxTorqueFraction));

    mBaseCmd.mutable_actuators(pJointId)->set_position(mBaseCmd.actuators(pJointId).position());
    mBaseCmd.mutable_actuators(pJointId)->set_torque_joint(pMaxTorqueFraction * cJointTorqueLimits[pJointId]);

    incrementFrameId();

    try
    {
        mBaseFb = mBaseCyclicClient->Refresh(mBaseCmd, 0);
    }
    catch(...)
    {
        std::cerr << "setJointTorqueSingle: error sending command" << std::endl;
        throw;
    }
}

void KinovaBaseConnection::setControlModeAllJoints(k_api::ActuatorConfig::ControlMode pMode)
{
    mCtrlModeInfo.set_control_mode(pMode);
    for (int actuatorId = BASE_ID_CONFIG; actuatorId < ACTUATOR_COUNT + BASE_ID_CONFIG; actuatorId++)
        mActuatorConfigClient->SetControlMode(mCtrlModeInfo, actuatorId);
}

void KinovaBaseConnection::stopRobotMotion()
{
    // read fb
    try {
        mBaseFb = mBaseCyclicClient->RefreshFeedback();
    } catch(...) {
        std::cerr << "error when reading sensors before stopping the robot" << std::endl;
        throw;
    }

    for (int i = BASE_ID_CONTROL; i < ACTUATOR_COUNT + BASE_ID_CONTROL; i++)
        mBaseCmd.mutable_actuators(i)->set_position(mBaseFb.actuators(i).position());

    setControlModeAllJoints(k_api::ActuatorConfig::ControlMode::POSITION);

    incrementFrameId();

    try {
        mBaseFb = mBaseCyclicClient->Refresh(mBaseCmd);
    } catch (...) {
        std::cerr << "error when sending command to stop the robot" << std::endl;
        throw;
    }
}

void KinovaBaseConnection::incrementFrameId()
{
    mBaseCmd.set_frame_id(mBaseCmd.frame_id() + 1);
    if (mBaseCmd.frame_id() > MAX_FRAME_ID) mBaseCmd.set_frame_id(0);

    for (int i = BASE_ID_CONTROL; i < ACTUATOR_COUNT + BASE_ID_CONTROL; i++)
        mBaseCmd.mutable_actuators(i)->set_command_id(mBaseCmd.frame_id());
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

void KinovaBaseConnection::moveToHomePosition(uint32_t pTimeoutSec)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    mServoingModeInfo.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    mBaseClient->SetServoingMode(mServoingModeInfo);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = mBaseClient->ReadAllActions(action_type);
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
        auto promise_notification_handle = mBaseClient->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        mBaseClient->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(std::chrono::seconds{pTimeoutSec});
        mBaseClient->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }
}

void handleKinovaException(k_api::KDetailedException& pEx) {
    std::cerr << "Kortex exception: " << pEx.what() << std::endl;
    std::cerr << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(pEx.getErrorInfo().getError().error_sub_code()))
              << std::endl;
}

bool waitMicroSeconds(const sc::time_point<sc::steady_clock> &pStartTime, const sc::microseconds &pDuration)
{
    auto now = sc::steady_clock::now();
    if (now - pStartTime > pDuration) return false;

    do {
        now = sc::steady_clock::now();
    } while (now - pStartTime < pDuration);
    return true;
}