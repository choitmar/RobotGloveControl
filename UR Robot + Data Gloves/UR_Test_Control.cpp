#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <array>
#include <vector>
#include <thread>
#include <chrono>

#pragma comment(lib, "Ws2_32.lib")

using namespace ur_rtde;

// Byte order swap for doubles
uint64_t byteswap_uint64(uint64_t val) {
    return _byteswap_uint64(val);
}
double byteswap_double(double val) {
    uint64_t temp;
    std::memcpy(&temp, &val, sizeof(double));
    temp = byteswap_uint64(temp);
    std::memcpy(&val, &temp, sizeof(double));
    return val;
}

int main() {
    const std::string ROBOT_IP = "172.19.15.14";
    const int PORT = 9999;
    const double TOTAL_DURATION = 1.0;     // 1 second total movement
    const double SUB_INTERVAL = 0.05;      // check every 50ms
    const double MIN_Z = 0.05;             // Z >= 50mm
    const double MAX_RADIUS = 0.5;         // max reach from base

    RTDEControlInterface rtde_control(ROBOT_IP);
    RTDEReceiveInterface rtde_receive(ROBOT_IP);

    std::vector<double> start_pose = { 0.02, -0.3, 0.375, 1.39, 2.314, -0.36 };
    std::cout << "Moving to start position...\n";
    rtde_control.moveL(start_pose, 0.3, 0.3);
    std::cout << "Reached start position.\n";

    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    SOCKET listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    InetPton(AF_INET, L"127.0.0.1", &server.sin_addr);

    if (bind(listenSocket, (SOCKADDR*)&server, sizeof(server)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << "\n";
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    listen(listenSocket, SOMAXCONN);
    std::cout << "Waiting for Kotlin client...\n";

    SOCKET clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Client accept failed.\n";
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }
    std::cout << "Client connected.\n";

    const int BUFFER_SIZE = 3 * sizeof(double);
    std::array<double, 3> received_velocity;

    while (true) {
        int bytesReceived = recv(clientSocket, reinterpret_cast<char*>(received_velocity.data()), BUFFER_SIZE, MSG_WAITALL);
        if (bytesReceived != BUFFER_SIZE) {
            std::cerr << "Receive error or client disconnected.\n";
            break;
        }

        for (double& val : received_velocity) {
            val = byteswap_double(val);
        }

        std::cout << "Received velocity: dx=" << received_velocity[0]
            << ", dy=" << received_velocity[1]
            << ", dz=" << received_velocity[2] << "\n";

        // Repeat movement in safe intervals
        double elapsed = 0.0;
        while (elapsed < TOTAL_DURATION) {
            std::vector<double> current_pose = rtde_receive.getActualTCPPose();
            std::vector<double> projected_pose = current_pose;

            // Predict new position
            for (int i = 0; i < 3; ++i) {
                projected_pose[i] += received_velocity[i] * SUB_INTERVAL;
            }

            double x = projected_pose[0];
            double y = projected_pose[1];
            double z = projected_pose[2];
            double dist = std::sqrt(x * x + y * y + z * z);

            if (z >= MIN_Z && dist <= MAX_RADIUS) {
                std::vector<double> velocity_cmd(6, 0.0);
                for (int i = 0; i < 3; ++i) {
                    velocity_cmd[i] = received_velocity[i];
                }

                rtde_control.speedL(velocity_cmd, 0.25, SUB_INTERVAL);
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(SUB_INTERVAL * 1000)));
                elapsed += SUB_INTERVAL;
            }
            else {
                std::cout << "[STOP] Movement would exceed limits. Z: " << z << ", Radius: " << dist << "\n";
                rtde_control.speedStop();
                break;
            }
        }

        // Ensure stopping after each command
        rtde_control.speedStop();
    }

    rtde_control.stopScript();
    closesocket(clientSocket);
    closesocket(listenSocket);
    WSACleanup();
    return 0;
}
