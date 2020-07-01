#ifndef _KINOVA_UTIL_H_
#define _KINOVA_UTIL_H_

#include <string>
#include <memory>
#include <vector>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#define BASE_ID_CONFIG 1
#define BASE_ID_CONTROL 0
#define ACTUATOR_COUNT 7
#define MAX_FRAME_ID 65535
#define JOINT_6_ID 6

namespace k_api = Kinova::Api;
namespace sc = std::chrono;

class KinovaBaseConnection
{
private:
    std::shared_ptr<k_api::TransportClientTcp> mTransportClientTcp;
    std::shared_ptr<k_api::RouterClient> mRouterTcp;
    std::shared_ptr<k_api::TransportClientUdp> mTransportClientUdp;
    std::shared_ptr<k_api::RouterClient> mRouterUdp;
    std::shared_ptr<k_api::SessionManager> mSessionManagerTcp;
    std::shared_ptr<k_api::SessionManager> mSessionManagerUdp;

    void incrementFrameId();

public:
    const std::vector<double> cJointTorqueLimits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};

    std::shared_ptr<k_api::Base::BaseClient> mBaseClient;
    std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> mBaseCyclicClient;
    std::shared_ptr<k_api::ActuatorConfig::ActuatorConfigClient> mActuatorConfigClient;

    k_api::Base::ServoingModeInformation mServoingModeInfo;
    k_api::ActuatorConfig::ControlModeInformation mCtrlModeInfo;
    k_api::BaseCyclic::Command mBaseCmd;
    k_api::BaseCyclic::Feedback mBaseFb;

    KinovaBaseConnection(std::string, uint32_t, uint32_t, std::string, std::string);
    ~KinovaBaseConnection();

    void refreshFeedBack();
    void setTorqueSingleJoint(uint8_t, double);
    void moveToHomePosition(uint32_t pTimeoutSec = 20);
    void stopRobotMotion();
    void setControlModeAllJoints(k_api::ActuatorConfig::ControlMode);
};

void handleKinovaException(k_api::KDetailedException&);

bool waitMicroSeconds(const sc::time_point<sc::steady_clock>&, const sc::microseconds&);

#endif  // _KINOVA_UTIL_H
