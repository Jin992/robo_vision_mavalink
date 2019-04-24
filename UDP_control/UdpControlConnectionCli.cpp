//
// Created by jin on 19.04.19.
//

#include <iostream>
#include "UdpControlConnectionCli.h"
#include "../bitrate.h"

//#define CUR_TIME_M_SEC (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() & 0xffffffff)

namespace {
    /**
     * void decrease_check(int32_t server_time, int32_t client_time, int32_t latency_threshold, bitrate_t &bt)
     * @brief measure latency and decide decrease bitrate or not
     * @param server_time          - time from server
     * @param client_time          - current client time
     * @param latency_threshold    - latency threshold for bitrate decrease
     * @param bt                   - bitrate control structure
     */
    void decrease_check(int32_t server_time, int32_t client_time, int32_t latency_threshold, bitrate_t &bt) {
        /// Check if your latency bigger than defined threshold value, and if was there changes in defined period of time
        if (server_time - client_time > latency_threshold && (CUR_TIME_M_SEC - bt.last_changes > bt.changes_period)) {
            /// if condition has been occurred decrease bitrate
            /// bitrate - % < min_bitrate set bitrate as min bitrate
            if ((bt.bitrate - (bt.bitrate * (bt.percent_substruct / 100))) < bt.min_bitrate) {
                /// if we already have min_bitrate do nothing
                if (bt.bitrate != bt.min_bitrate) {
                    bt.bitrate = bt.min_bitrate;
                    bt.bitrate_guard.lock();
                    bt.changes = true;
                    bt.bitrate_guard.unlock();
                    bt.last_changes = CUR_TIME_M_SEC;
                    std::cerr << "bitrate has been set as min_bitrate " << std::endl;
                }
            }
            else {
                /// if bitrate - % > min_bitrate, subtract & from bitrate value
                bt.bitrate -= bt.bitrate * bt.percent_substruct / 100;
                bt.bitrate_guard.lock();
                bt.changes = true;
                bt.bitrate_guard.unlock();
                bt.last_changes = CUR_TIME_M_SEC;
                std::cerr << "bitrate has been decreased, current " << bt.bitrate << std::endl;
            }
            /// nullify stability counter
            bt.stability_cnt = 0;
        }
    }

    /**
     * void increase_check(bitrate_t &bt)
     * @brief measure stream stability and decide increase bitreate or not
     * @param bt - bitrate control structure
     */
    void increase_check(bitrate_t &bt) {
        /// Check if stability counter greater than zero and last bitrate changes was more than bt.changes_period milliseconds ago
       	if (bt.stability_cnt > 0 && (CUR_TIME_M_SEC - bt.last_changes > bt.changes_period)) {
            /// if bitrate + % greater than max_bitrate set bitrate as max_bitrate
            if (bt.bitrate + (bt.bitrate * bt.persent_add / 100) > bt.max_bitrate) {
                /// if we already have max_bitrate do nothing
                if (bt.bitrate != bt.max_bitrate) {
                    bt.bitrate = bt.max_bitrate;
                    bt.bitrate_guard.lock();
                    bt.changes = true;
                    bt.bitrate_guard.unlock();
                    bt.last_changes = CUR_TIME_M_SEC;
                    std::cerr << "bitrate has been set as max_bitrate " << std::endl;
                }
            } else {
                /// if bitrate + % < max_bitrate increase bitrate
                bt.bitrate += bt.bitrate * bt.persent_add / 100;
                bt.bitrate_guard.lock();
                bt.changes = true;
                bt.bitrate_guard.unlock();
                bt.last_changes= CUR_TIME_M_SEC;
                std::cerr << "bitrate has been increased, current " << bt.bitrate << std::endl;
            }
       }
    }
    /**
     * void check_latency(int32_t server_time, int32_t client_time, int32_t latency_threshold, bitrate_t &bt)
     * @brief main latency control function encapsulate decrease_check and increase_check inside
     * @param server_time          - time from server
     * @param client_time          - current client time
     * @param latency_threshold    - latency threshold for bitrate decrease
     * @param bt                   - bitrate control structure
     */
    void check_latency(int32_t server_time, int32_t client_time, int32_t latency_threshold, bitrate_t &bt) {
        /// check if we need to decrease bitrate
	   // std::cout << "Stream latency "<< server_time - client_time << " threshold " << latency_threshold 
	//	    << " paket stream stability [" << bt.stability_cnt 
	//	    << "], period diff [" << CUR_TIME_M_SEC- bt.last_changes << "], period[" << bt.changes_period << "]" <<std::endl; 
        decrease_check(server_time, client_time, latency_threshold, bt);
       /// check if we need to increase bitrate
        increase_check(bt);
        /// increase stability counter
        bt.stability_cnt++;
        /// if counter reached it limit, set it as 1
        if (bt.stability_cnt == std::numeric_limits<size_t>::max()) {
            bt.stability_cnt = 1;
        }
    }
}

/**
 * UdpControlConnectionCli::UdpControlConnectionCli(boost::asio::io_service &io)
 * @brief Udp control stream constructor
 * @param io - boost::asio::io_service object
 */
UdpControlConnectionCli::UdpControlConnectionCli(boost::asio::io_service &io)
: AbstractUdpControlConnection(io), sequence_(0), sock_in_{0}, poll_res_(0), last_sended_(0)
{}

/**
 * UdpControlConnectionCli::UdpControlConnectionCli(boost::asio::io_service &io, const std::string ip,
                                              const std::string port, const std::string &magic_string)
 * @param io - boost::asio::io_service object
 * @param ip - server ip address
 * @param port - server port
 * @param magic_string - word to transform in magic string
 */
UdpControlConnectionCli::UdpControlConnectionCli(boost::asio::io_service &io, const std::string ip,
                                                 const std::string port, const std::string &magic_string)
: AbstractUdpControlConnection(io), ip_(ip), port_(port) {
   udp::resolver resolver(get_io_service());                     /// initialize resolver
   udp::resolver::query query(udp::v4(), ip, port);              /// setup protocol + ip + port query for resolver
   get_remote_endpoint() = *resolver.resolve(query);             /// initialize endpoint with address resolver
   get_socket() = udp::socket(get_io_service());                 /// initialize socket
   set_magic_string(magic_string);                               /// generate magic string
   set_connection_init(true);                                    /// set connection initialization flag to true
}

/**
 * void UdpControlConnectionCli::init_stream(const std::string ip, const std::string port, const std::string &magic_string)
 * @brief initialize stream if constructor with only io object was used
 * @param ip - server ip
 * @param port - server port
 * @param magic_string - word to transform in magic string
 */
void UdpControlConnectionCli::init_stream(const std::string ip, const std::string port, const std::string &magic_string) {
   if(!get_connection_init()){
      ip_ = ip;
      port_ = port;
      udp::resolver resolver(get_io_service());                   /// initialize resolver
      udp::resolver::query query(udp::v4(), ip_, port_);          /// setup protocol + ip + port query for resolver
      get_remote_endpoint() = *resolver.resolve(query);           /// initialize endpoint with address resolver
      get_socket() = udp::socket(get_io_service());               /// initialize socket
      set_magic_string(magic_string);                             /// generate magic string
      set_connection_init(true);                                  /// set connection initialization flag to true
   }
}

/**
 * void UdpControlConnectionCli::open_connection()
 * @brief establish connection
 */
void UdpControlConnectionCli::open_connection() {
   /// open connection
   get_socket().open(udp::v4());
   if (!get_socket().is_open()) {
       throw "UdpControlStream: failed to open socket";             /// temporary exception stub
   }
}

/**
 * void UdpControlConnectionCli::init_message()
 * @brief sends first identification message to server
 */
void UdpControlConnectionCli::init_message() {
   // boost buffer for transport
   ctl_buff_t magic_buf = {0};
   // nullify buffer
   bzero(magic_buf.data(), magic_buf.size());
   // fill part of buffer with magic string
   memcpy(magic_buf.data(), get_magic_string().c_str(), get_magic_string().size());
   last_sended_ = CUR_TIME_M_SEC;
   // send magic string to server
   get_socket().send_to(boost::asio::buffer(magic_buf), get_remote_endpoint());
}


/**
 * void UdpControlConnectionCli::run(std::mutex &mutex, bool &reconnect, int32_t  latency_threshold, bitrate_t &bt)
 * @brief launch client event loop
 * @param mutex - protection to bitrate control struct and reconnect
 * @param reconnect - flag to reconnect
 * @param latency_threshold - set threshold to measure latency
 * @param bt - bitrate control structure
 */
void UdpControlConnectionCli::run(std::mutex &mutex, bool &reconnect, int32_t  latency_threshold, bitrate_t &bt) {
   sock_in_[0].fd = get_socket().native_handle();
   sock_in_[0].events = POLLIN;

   /// send initial message to server for client recognition
   init_message();
   while (true) {
       boost::system::error_code err;
      /// check last time when packet was received
      /// if connection was lost for 10 seconds
      /// reconnect  to server
      if (CUR_TIME_M_SEC -  last_sended_ > 10000) {
         std::cerr << "Connection lost." << std::endl;
         mutex.lock();                /// lock mutex on flag to change value
         reconnect = true;            /// trigger video stream to reconnect
         mutex.unlock();              /// unlock mutex
         break;
      }
      /// polling socket for incoming connection
      poll_res_ = poll(sock_in_, 1, 300);
      if (poll_res_ > 0) {
         /// prepare buffer
         get_request_buff().fill(0);
         /// receive data
         get_socket().receive_from(boost::asio::buffer(get_request_buff()), get_remote_endpoint(), 0, err);
         if (err) {
             throw "UdpControlConnectionCli: failed in function run()(receive_from)";
         }
         /// check magic string
         if (std::string(get_request_buff().data(), get_magic_string().size()) == get_magic_string()) {
            /// extract control struct from buffer
            ctl_pkt *received_pkt = (ctl_pkt *) (get_request_buff().data() + get_magic_string().size());
            /// measure latency and change bitrate
            check_latency(CUR_TIME_M_SEC, last_sended_, latency_threshold, bt);

            /// debug info
            //std::cout << "last reseved time - prev received time " <<  CUR_TIME_M_SEC - received_pkt->last_received_time
              //       << std::endl;
            /// measure how many packets we lost
            if (received_pkt->last_received_id - sequence_ > 5) {
               std::cerr << "Dropped frames need to change bitrate" << std::endl;
            }
            /// debug info
           // std::cout << "sequence " << static_cast<int>(sequence_) << " sequence from server "
             //        << static_cast<int>(received_pkt->last_received_id) << std::endl;
            /// memorize last sending time
            last_sended_ = CUR_TIME_M_SEC;
            /// iterate sequence
            sequence_++;
            /// fill control struct with sequence
            /// last_receive time = 0 (server doesn't need this)
            ctl_pkt res = {0};
            res.last_received_id = sequence_;
            res.last_received_time = 0;
            /// create buffer to store client magic + structure
            get_res_buff().fill(0);
            /// fill buffer
            memcpy(get_res_buff().data(), get_magic_string().c_str(), get_magic_string().size());
            memcpy(get_res_buff().data() + get_magic_string().size(), &res, sizeof(ctl_pkt));
            /// send buffer to server
            get_socket().send_to(boost::asio::buffer(get_res_buff()), get_remote_endpoint(), 0, err);
            if (err) {
                throw "UdpControlConnectionCli: failed to send in run";
            }
            /// sleep for 3 sec
            ::usleep(300000);
         }
      }
   }
}
