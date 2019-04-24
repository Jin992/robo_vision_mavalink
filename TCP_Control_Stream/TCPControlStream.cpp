//
// Created by jin on 19.04.19.
//

#include "TCPControlStream.h"

TCPControlStream::~TCPControlStream() {
    /// check latency_control, join if we can
    if (latency_control.joinable()) {
        latency_control.join();
    }
    /// check video_stream, join if we can
    if (video_thread.joinable()) {
        video_thread.join();
    }

}

/**
 * void TCPControlStream::set_latency_control(udp_control &udp_ctl)
 * @param udp_ctl - std::function object or lambda with implementation of latency server
 */
void TCPControlStream::set_latency_control(udp_control &udp_ctl) {
    udp_ctl_ = udp_ctl;
}

/**
 * void TCPControlStream::set_video_stream(video_stream &video_stream)
 * @param video_stream - std::function object or lambda with implementation of video stream server
 */
void TCPControlStream::set_video_stream(video_stream &video_stream) {
    v_stream_ = video_stream;
}

/**
 * void TCPControlStream::start_udp_control_stream(int port)
 * @brief create and launch TCPControlStream (web request server) in thread
 * @param port
 */
void TCPControlStream::start_udp_control_stream(int port) {
    latency_control = std::thread(udp_ctl_, port);
}

/**
 * void TCPControlStream::start_video_stream(int port)
 * @brief create and launch VideoStream server in thread
 * @param port
 */
void TCPControlStream::start_video_stream(int port) {
    video_thread = std::thread(v_stream_, port);
}

/**
 * void TCPControlStream::start_control_server(int port)
 * @brief set msg handler and launch web control server
 * @param port
 */
void TCPControlStream::start_control_server(int port) {
    msg_handler hdl = [](server::message_ptr &msg){
        if(msg->get_opcode() == websocketpp::frame::opcode::text) {
            //write(1, msg->get_payload().data(), msg->get_payload().size());
        }
    };
    control_stream.set_msg_hdl(hdl);
    control_stream.run(port);
}