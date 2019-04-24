#include <boost/asio.hpp>
#include <server_src/base_server.h>
#include "TCP_Control_Stream/TCPControlStream.h"
#include "UDP_control/UdpControlConnectionHost.h"

/**
 * void control_stream_server(int port)
 * @brief measures sequence and latency time to tune bitrate, also control reconnect
 * @param port - server port
 */
void control_stream_server(int port)
{
    /// Create io
    boost::asio::io_service io_service;
    UdpControlConnectionHost host(io_service);
    /// Initialize socket and magic string
    host.init(port, "Radion");
    /// Open socket
    host.open_connection();
    /// Start event loop
    host.run();
    /// Close socket
    host.close_connection();
}

/**
 * void video_stream_server(int port)
 * @brief accept and process video stream from client
 * @param port - stream port
 */
void video_stream_server(int port) {
    base_server video_stream;
    /// define msg handler
    /// it determines the behavior of the server stream processing function
    msg_handler hdl = [](server::message_ptr &msg) {
        /// if we receive binary data proceed
        if (msg->get_opcode() == websocketpp::frame::opcode::binary) {
            /// currently write to terminal
            write(1, msg->get_payload().data(), msg->get_payload().size());
        }
    };
    /// set msg handler
    video_stream.set_msg_hdl(hdl);
    /// trigger for bitrate switch, currently there is no use for it
    //video_stream.action = false;
    /// start video stream event loop
    video_stream.run(port);
}


int main() {
    /// Create TCP control object
    TCPControlStream control_server;

    /// Set our servers to functor objects
    std::function<void(int)> latency_server = control_stream_server;
    std::function<void(int)> video_server = video_stream_server;

    /// Add UDP Sequence&Latency and TCP Video stream servers to Control server thread
    control_server.set_video_stream(video_server);
    control_server.set_latency_control(latency_server);

    /// Launch all servers, TCP Control(Web Control) need to be launched last one
    control_server.start_udp_control_stream(25085);    /// Sequence&Latency server
    control_server.start_video_stream(25090);          /// Video Stream server
    //control_server.start_control_server(25085);        /// Web Control server
    return 0;
}





