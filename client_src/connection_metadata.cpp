//
// Created by roda on 27.03.19.
//

#include "connection_metadata.h"

connection_metadata::connection_metadata(websocketpp::connection_hdl hdl, std::string uri)
        : m_hdl(hdl)
        , m_status("Connecting")
        , m_uri(uri)
        , m_server("N/A")
{}

/**
 * void connection_metadata::on_open(client * c, websocketpp::connection_hdl hdl)
 * @brief handle newly established connection
 * @param c - client
 * @param hdl - connection handler (weak_ptr)
 */
void connection_metadata::on_open(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Open";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
    std::cout << "Opened~~~~~~~~~~~~~~~~~~" << std::endl;
}

/**
 * void connection_metadata::on_fail(client * c, websocketpp::connection_hdl hdl)
 * @brief handle cases when error occurred in connection establishment
 * @param c - client
 * @param hdl - connection handler
 */
void connection_metadata::on_fail(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Failed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
    m_error_reason = con->get_ec().message();
    std::cerr << "Failed to connect to server" << std::endl;
}

/**
 * void connection_metadata::on_close(client * c, websocketpp::connection_hdl hdl)
 * @brief connection close handler
 * @param c - client
 * @param hdl - connection handler
 */
void connection_metadata::on_close(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Closed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "close code: " << con->get_remote_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_remote_close_code())
      << "), close reason: " << con->get_remote_close_reason();
    m_error_reason = s.str();
}

/**
 * void connection_metadata::on_message(websocketpp::connection_hdl, client::message_ptr msg)
 * @brief websocket message handler
 * @param msg - message
 * @param hdl - connection handler (weak_ptr)
 */
void connection_metadata::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
    // temporary spike to avoid unused variable warning
    if (msg->get_payload().empty()) {

    }
}

/**
 * websocketpp::connection_hdl connection_metadata::get_hdl() const
 * @return weak_ptr to connection handler
 */
websocketpp::connection_hdl connection_metadata::get_hdl() const {
    return m_hdl;
}

/**
 * std::string connection_metadata::get_status() const
 * @brief getter for connection status
 * @return connection status
 */
std::string connection_metadata::get_status() const {
    return m_status;
}


void connection_metadata::record_sent_message(std::string message) {
    m_messages.push_back(">> " + message);
}