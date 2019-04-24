//
// Created by jin on 19.04.19.
//

#ifndef WEBSOCKET_TEST_TCPCONTROLSTREAM_H
#define WEBSOCKET_TEST_TCPCONTROLSTREAM_H

#include "server_src/base_server.h"
#include "../bitrate.h"

typedef std::function<void(int port)> udp_control;
typedef std::function<void(int port)> video_stream;

class TCPControlStream {
    public:
        TCPControlStream() = default;
        ~TCPControlStream();

        TCPControlStream(const TCPControlStream &rhs) = delete;
        TCPControlStream(const TCPControlStream &&rhs) = delete;
        TCPControlStream &operator=(const TCPControlStream &rhs) = delete;
        TCPControlStream &operator=(const TCPControlStream &&rhs) = delete;

        void set_latency_control(udp_control &udp_ctl);
        void set_video_stream(video_stream &video_stream);
        void start_udp_control_stream(int port);
        void start_video_stream(int port);
        void start_control_server(int port);

    private:
        base_server control_stream;
        udp_control udp_ctl_;
        video_stream v_stream_;
        std::thread latency_control;
        std::thread video_thread;
};


#endif //WEBSOCKET_TEST_TCPCONTROLSTREAM_H
