cmake_minimum_required(VERSION 3.1)
project(websocket_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wundef -Wno-missing-field-initializers -Wpedantic ")

include_directories(websocketpp)

find_package(Boost 1.58 COMPONENTS system regex  filesystem REQUIRED)
find_package(OpenSSL REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

# Server binary
add_executable(server   server.cpp
                        server_src/base_server.cpp
                        server_src/base_server.h
                        UDP_control/AbstractUdpControlConnection.cpp
                        UDP_control/UdpControlConnectionHost.cpp
                        TCP_Control_Stream/TCPControlStream.cpp
                        )

target_link_libraries(server    ${Boost_LIBRARIES}
                                -pthread
                                OpenSSL::Crypto)

# Client binary
add_executable(client   client.cpp
                        client_src/websocket_endpoint.cpp
                        client_src/connection_metadata.cpp
                        Video/VideoCapture.cpp Video/VideoWriter.cpp
                        Video/usbinfo.c
                        UDP_control/AbstractUdpControlConnection.cpp
                        UDP_control/UdpControlConnectionCli.cpp
			mav_link/serial_port.cpp 
			mav_link/autopilot_interface.cpp)

target_link_libraries(client    ${Boost_LIBRARIES}
                                -pthread
                                usb-1.0
                                -ludev
                                OpenSSL::Crypto)

