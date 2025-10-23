#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#define TOPIC_HIGHSTATE "rt/sportmodestate"
using namespace unitree::common;
using namespace unitree::robot;

int udp_socket;
std::atomic<bool> client_ready(false);
sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);

void WaitForNextClient() {
    std::thread([] {
        uint8_t buf[1];
        sockaddr_in latest_client;
        socklen_t latest_client_len = sizeof(latest_client);

        while (true) {
            int n = recvfrom(udp_socket, buf, sizeof(buf), 0,
                             (sockaddr*)&latest_client, &latest_client_len);
            if (n > 0) {
                // Update the global client address atomically
                std::memcpy(&client_addr, &latest_client, sizeof(sockaddr_in));
                client_addr_len = latest_client_len;
                if (!client_ready.exchange(true)) {
                    printf("[UDP] First client connected from %s:%d\n",
                           inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                } else {
                    printf("[UDP] Switched to new client %s:%d\n",
                           inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                }
            }
        }
    }).detach();
}

void InitUDPServer(uint16_t port) {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("socket creation failed");
        return;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(udp_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(udp_socket);
        return;
    }

    WaitForNextClient();
}

void sendtoClient(const uint8_t* data, size_t len) {
    if (client_ready.load()) {
        sendto(udp_socket, data, len, 0, (sockaddr*)&client_addr, client_addr_len);
    }
}

void _HighStateHandler(const void *message) {
    using namespace std::chrono;
    static auto last_send_time = steady_clock::now();
    auto now = steady_clock::now();
    if (duration_cast<milliseconds>(now - last_send_time).count() < 20) {
        return;  // Skip if less than 20ms since last send
    }
    last_send_time = now;

    const auto* high_state = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    float data[13];
    data[0] = high_state->position()[0];
    data[1] = high_state->position()[1];
    data[2] = high_state->position()[2];
    data[3] = high_state->imu_state().quaternion()[0];
    data[4] = high_state->imu_state().quaternion()[1];
    data[5] = high_state->imu_state().quaternion()[2];
    data[6] = high_state->imu_state().quaternion()[3];
    data[7] = high_state->imu_state().accelerometer()[0];
    data[8] = high_state->imu_state().accelerometer()[1];
    data[9] = high_state->imu_state().accelerometer()[2];
    data[10] = high_state->imu_state().gyroscope()[0];
    data[11] = high_state->imu_state().gyroscope()[1];
    data[12] = high_state->imu_state().gyroscope()[2];

    // printf("Position: %f, %f, %f\n", data[0], data[1], data[2]);
    // printf("IMU quaternion: %f, %f, %f, %f\n", data[3], data[4], data[5], data[6]);

    // sendto(udp_socket, data, sizeof(data), 0, (sockaddr*)&target_addr, sizeof(target_addr));
    sendtoClient((uint8_t *)data, sizeof(data));
};

int main(int argc, const char *argv[]) {
    InitUDPServer(8889);

    // init go2
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    unitree_go::msg::dds_::SportModeState_ high_state{}; // default init
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(&_HighStateHandler, 1);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}