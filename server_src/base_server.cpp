//
// Created by roda on 27.03.19.
//

#include <client_src/websocket_endpoint.h>
#include "base_server.h"


/**
 * base_server::base_server()
 * @brief base_server constructor
 */
base_server::base_server() : is_connected(false), counter(0), id_counter(0), m_handler(nullptr) {
    m_server.init_asio();
    m_server.set_open_handler(bind(&base_server::on_open,this,::_1));
    m_server.set_close_handler(bind(&base_server::on_close,this,::_1));
    m_server.set_validate_handler(bind(&base_server::on_validate,this,::_1));
    m_server.set_message_handler(bind(&base_server::on_message,this, ::_1,::_2));
    m_server.set_http_handler(bind(&base_server::on_http, this, ::_1));
}

/**
 * void base_server::on_http(connection_hdl hdl)
 * @brief websocketpp http requests handler
 * @param hdl - connection handler weak_pointer
 */
void base_server::on_http(connection_hdl hdl) {
    connection_ptr con = m_server.get_con_from_hdl(hdl);

    con->defer_http_response();
    con->set_body("Request " + con->get_request().get_uri());
    con->set_status(websocketpp::http::status_code::ok);
    con->send_http_response();
}

/**
 * void base_server::set_msg_hdl(msg_handler &hdl)
 * @brief Setter for wepsocketpp message hangler
 * @param hdl - reference to message handler function
 */
void base_server::set_msg_hdl(msg_handler &hdl){
    m_handler = hdl;
}

/**
 * void base_server::on_open(connection_hdl hdl)
 * @brief websocketpp init connection handler
 * @param hdl - connection handler weak_pointer
 */
void base_server::on_open(connection_hdl hdl) {
    connection_ptr con = m_server.get_con_from_hdl(hdl);
    con->id = id_counter++;
    m_connections.emplace_back(con);
}

/**
 * void base_server::on_close(connection_hdl hdl)
 * @brief websocketpp connection close handler
 * @param hdl - connection handler weak_pointer
 */
void base_server::on_close(connection_hdl hdl) {
    auto it = std::remove_if(m_connections.begin(), m_connections.end(), [this, hdl](connection_ptr ptr) {
        if (m_server.get_con_from_hdl(hdl)->id == ptr->id){
            return true;
        }
        return false;
    });
    if (it != m_connections.end()) {
        std::cout << (*it)->video_stream.size() << " frames received" << std::endl;
        m_connections.erase(it, m_connections.end());
    }
}

/**
 * void base_server::on_message(connection_hdl hdl, server::message_ptr msg)
 * @brief websocketpp incoming message handler
 * @param hdl - connection handler weak_pointer
 * @param msg - server::message_ptr ptr to object that store msg
 */
void base_server::on_message(connection_hdl hdl, server::message_ptr msg) {
    hdl.lock();
    if (m_handler != nullptr) {
        m_handler(msg);
    }
//    if (msg->get_opcode() == websocketpp::frame::opcode::binary) {
//        /*auto it = std::find_if(m_connections.begin(), m_connections.end(),[this, hdl](connection_ptr ptr) {
//            if (m_server.get_con_from_hdl(hdl)->id == ptr->id){
//                return true;
//            }
//            return false;
//        });
//        if (it != m_connections.end()) {
//            //uint8_t *tmp = new uint8_t[msg->get_payload().size()]();
//            //memcpy(tmp, msg->get_payload().data(), msg->get_payload().size());
//            write(1, msg->get_payload().data(), msg->get_payload().size());
//            //(*it)->video_stream.push(std::make_pair(tmp, msg->get_payload().size()));
//            action = true;
//        }*/
//        write(1, msg->get_payload().data(), msg->get_payload().size());
//    }
}



bool base_server::on_validate(connection_hdl hdl) {
    hdl.lock();
    return 1;
}

/**
 * void base_server::run(uint16_t port)
 * @brief Run method launch server eventloop
 * @param port - server port
 */
void base_server::run(uint16_t port) {
    m_server.listen(port);
    m_server.start_accept();
    m_server.run();
}