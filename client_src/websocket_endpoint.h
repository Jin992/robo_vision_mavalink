//
// Created by roda on 27.03.19.
//

#ifndef WEBSOCKET_TEST_WEBSOCKET_ENDPOINT_H
#define WEBSOCKET_TEST_WEBSOCKET_ENDPOINT_H

#include "client_src/connection_metadata.h"
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

class websocket_endpoint {
public:
    websocket_endpoint ();
    ~websocket_endpoint();

    int connect(std::string const &); //(std::string const & uri)
    void close(websocketpp::close::status::value, std::string); //(websocketpp::close::status::value code, std::string reason)
    void send(void *, size_t); //(void *ptr, size_t size)
    std::string get_status() {return m_metadata_ptr->get_status();}

private:
    typedef websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread;
    connection_metadata::ptr m_metadata_ptr;
    client m_endpoint;
    thread m_thread;
    int m_next_id;
};


#endif //WEBSOCKET_TEST_WEBSOCKET_ENDPOINT_H
