#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#define TOPIC_HIGHSTATE "rt/sportmodestate"
using namespace unitree::common;
using namespace unitree::robot;

enum MessageType : uint8_t {
    MSG_POINTCLOUD = 1,
    MSG_HIGHSTATE  = 2
};

int udp_socket;
sockaddr_in target_addr;

void InitUDPSender(const char* ip, uint16_t port) {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &target_addr.sin_addr);
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr) {
        return;
    }

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint* pts = (LivoxLidarCartesianHighRawPoint*)data->data;

        std::vector<float> xyz;
        xyz.reserve(data->dot_num * 3);
        for (uint32_t i = 0; i < data->dot_num; ++i) {
            float x = static_cast<float>(pts[i].x) / 1000.0f;
            float y = static_cast<float>(pts[i].y) / 1000.0f;
            float z = static_cast<float>(pts[i].z) / 1000.0f;
            // Only keep points close to the horizontal plane
            if (z > -0.10f && z < 0.10f) {
                xyz.push_back(x);
                xyz.push_back(y);
                xyz.push_back(z);
            }
        }

        if (xyz.empty()) return;
        
        std::vector<uint8_t> buffer(sizeof(MessageType) + xyz.size() * sizeof(float));
        buffer[0] = MSG_POINTCLOUD;
        memcpy(buffer.data() + 1, xyz.data(), xyz.size() * sizeof(float));
        sendto(udp_socket, buffer.data(), buffer.size(), 0, (sockaddr*)&target_addr, sizeof(target_addr));
        // sendto(udp_socket, xyz.data(), xyz.size() * sizeof(float), 0, (sockaddr*)&target_addr, sizeof(target_addr));
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
    if (status != kLivoxLidarStatusSuccess) {
        printf("Query lidar internal info failed.\n");
        QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
        return;
    }

    if (response == nullptr) {
        return;
    }

    uint8_t host_point_ipaddr[4] {0};
    uint16_t host_point_port = 0;
    uint16_t lidar_point_port = 0;

    uint8_t host_imu_ipaddr[4] {0};
    uint16_t host_imu_data_port = 0;
    uint16_t lidar_imu_data_port = 0;

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
        LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
        if (kv->key == kKeyLidarPointDataHostIpCfg) {
        memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
        memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
        memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
        } else if (kv->key == kKeyLidarImuHostIpCfg) {
        memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
        memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
        memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }

    printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
        host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

    printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
        host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr) {
        return;
    }
    printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
        status, handle, response->ret_code, response->error_key);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    } 
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    
    // set the work mode to kLivoxLidarNormal, namely start the lidar
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
  
    // LivoxLidarIpInfo lidar_ip_info;
    // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
    // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
    // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
    // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void _HighStateHandler(const void *message) {
    unitree_go::msg::dds_::SportModeState_ high_state{}; // default init
    high_state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    float data[7];
    data[0] = high_state.position()[0];
    data[1] = high_state.position()[1];
    data[2] = high_state.position()[2];
    data[3] = high_state.imu_state().quaternion()[0];
    data[4] = high_state.imu_state().quaternion()[1];
    data[5] = high_state.imu_state().quaternion()[2];
    data[6] = high_state.imu_state().quaternion()[3];

    // printf("Position: %f, %f, %f\n", data[0], data[1], data[2]);
    // printf("IMU quaternion: %f, %f, %f, %f\n", data[3], data[4], data[5], data[6]);

    sendto(udp_socket, data, sizeof(data), 0, (sockaddr*)&target_addr, sizeof(target_addr));
    uint8_t buffer[1 + sizeof(data)];
    buffer[0] = MSG_HIGHSTATE;
    memcpy(buffer + 1, data, sizeof(data));
    sendto(udp_socket, buffer, sizeof(buffer), 0, (sockaddr*)&target_addr, sizeof(target_addr));
};

int main(int argc, const char *argv[]) {
    if (argc != 2) {
        printf("Params Invalid, must input config path.\n");
        return -1;
    }
    const std::string path = argv[1];

    // REQUIRED, to init Livox SDK2
    if (!LivoxLidarSdkInit(path.c_str())) {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    } else {
        printf("Livox Init Success\n");
    }

    InitUDPSender("192.168.8.214", 8888);

    // init go2
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    unitree_go::msg::dds_::SportModeState_ high_state{}; // default init
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(&_HighStateHandler, 1);
    
    // REQUIRED, to get point cloud data via 'PointCloudCallback'
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    
    // // OPTIONAL, to get imu data via 'ImuDataCallback'
    // // some lidar types DO NOT contain an imu component
    // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    
    // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    
    // // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LivoxLidarSdkUninit();
    printf("Livox Quick Start Demo End!\n");
    return 0;
}