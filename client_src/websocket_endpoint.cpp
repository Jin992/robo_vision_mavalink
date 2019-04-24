//
// Created by roda on 27.03.19.
//

#include "websocket_endpoint.h"

websocket_endpoint::websocket_endpoint () : m_next_id(0) {
    m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

    m_endpoint.init_asio();
    m_endpoint.start_perpetual();

    m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
}

websocket_endpoint::~websocket_endpoint() {
    m_endpoint.stop_perpetual();

    /*for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
        if (it->second->get_status() != "Open") {
            // Only close open connections
            continue;
        }
        std::cout << "> Closing connection " << it->second->get_id() << std::endl;
        websocketpp::lib::error_code ec;
        m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
        if (ec) {
            std::cout << "> Error closing connection " << it->second->get_id() << ": "
                      << ec.message() << std::endl;
        }
    }
    */
    websocketpp::lib::error_code ec;
    m_endpoint.close(m_metadata_ptr->get_hdl(), websocketpp::close::status::going_away, "", ec);
    m_thread->join();
}

int websocket_endpoint::connect(std::string const & uri) {
    websocketpp::lib::error_code ec;

    client::connection_ptr con = m_endpoint.get_connection(uri, ec);
    if (ec) {
        throw "websocket_endpoint [connect]: Failed connect to remote endpoint.";
    }

    m_metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(con->get_handle(), uri);

    con->set_open_handler(websocketpp::lib::bind(
            &connection_metadata::on_open,
            m_metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
    ));
    con->set_fail_handler(websocketpp::lib::bind(
            &connection_metadata::on_fail,
            m_metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
    ));
    con->set_close_handler(websocketpp::lib::bind(
            &connection_metadata::on_close,
            m_metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
    ));
    con->set_message_handler(websocketpp::lib::bind(
            &connection_metadata::on_message,
            m_metadata_ptr,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2
    ));

    m_endpoint.connect(con);

    return 0;
}

void websocket_endpoint::close(websocketpp::close::status::value code, std::string reason) {
    websocketpp::lib::error_code ec;

    m_endpoint.close(m_metadata_ptr->get_hdl(), code, reason, ec);
    if (ec) {
        throw ec.message();
    }
}

void websocket_endpoint::send(void *ptr, size_t size) {
    websocketpp::lib::error_code ec;

    m_endpoint.send(m_metadata_ptr->get_hdl(), ptr, size, websocketpp::frame::opcode::binary, ec);
    if (ec) {
        //throw ec.message();
    }
}
