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

// Byte-Swap für Netzwerkübertragung
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
    // Verbindung zum Roboter
    RTDEControlInterface rtde_control("172.17.170.228");
    RTDEReceiveInterface rtde_receive("172.17.170.228");

    // Socket Setup
    const int PORT = 9999;
    const int BUFFER_SIZE = 3 * sizeof(double);
    std::array<double, 3> received_velocity{};
    const double cycle_time = 0.01; // 10ms = 100Hz
    const double execution_duration = 1.0; // Geschwindigkeit für 1s anwenden

    // Winsock initialisieren
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    SOCKET listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    sockaddr_in service{};
    service.sin_family = AF_INET;
    service.sin_port = htons(PORT);
    InetPton(AF_INET, L"127.0.0.1", &service.sin_addr);

    if (::bind(listenSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << "\n";
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    listen(listenSocket, SOMAXCONN);
    std::cout << "Waiting for client on port " << PORT << "...\n";

    SOCKET clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Client connection failed.\n";
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Client connected.\n";

    while (true) {
        int bytesReceived = recv(clientSocket, reinterpret_cast<char*>(received_velocity.data()), BUFFER_SIZE, MSG_WAITALL);
        if (bytesReceived != BUFFER_SIZE) {
            std::cerr << "Receive error or client disconnected.\n";
            break;
        }

        // Byte-Swapping der Netzwerkdaten
        for (double& val : received_velocity) {
            val = byteswap_double(val);
        }

        // Aktuelle Pose vom Roboter
        std::vector<double> current_pose = rtde_receive.getActualTCPPose();
        std::vector<double> target_pose = current_pose;

        // Zielposition berechnen: Geschwindigkeit * Zeit
        for (int i = 0; i < 3; ++i) {
            target_pose[i] += received_velocity[i] * execution_duration;
        }

        // Suche nach erreichbarer Position durch Interpolation
        std::vector<double> valid_pose = current_pose;
        bool found_valid = false;

        for (double t = 0.1; t <= 1.0; t += 0.1) {
            std::vector<double> interp_pose = current_pose;
            for (int i = 0; i < 3; ++i) {
                interp_pose[i] += (target_pose[i] - current_pose[i]) * t;
            }

            if (rtde_control.isPoseWithinSafetyLimits(interp_pose)) {
                valid_pose = interp_pose;
                found_valid = true;
            }
            else {
                break;
            }
        }

        if (found_valid) {
            std::vector<double> velocity_command(6, 0.0);
            for (int i = 0; i < 3; ++i) {
                velocity_command[i] = (valid_pose[i] - current_pose[i]) / cycle_time;
            }

            rtde_control.speedL(velocity_command, 0.25, 0.1);
        }
        else {
            std::cout << "??  Bewegung abgebrochen – Ziel außerhalb des Arbeitsbereichs.\n";
            rtde_control.speedStop();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rtde_control.stopScript();
    closesocket(clientSocket);
    closesocket(listenSocket);
    WSACleanup();
    return 0;
}
