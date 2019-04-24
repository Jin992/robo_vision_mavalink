#include "client_src/websocket_endpoint.h"
#include "Video/VideoCapture.h"
#include "UDP_control/UdpControlConnectionCli.h"
#include "bitrate.h"
#include <iostream>

#include "mav_link/serial_port.h"
#include "mav_link/autopilot_interface.h"
#include <stdio.h>

/// UDP Control
constexpr int dev_name_n    = 2;
constexpr int width_n       = 3;
constexpr int height_n      = 4;
constexpr int fps_n         = 5;
constexpr int bitrate_n     = 6;

#define PI           3.14159265358979323846  /* pi include need math.h in qt*/

/**
 * void watch_dog(websocket_endpoint &endpoint, VideoCapture &cap, char *address,int timer)
 * @brief Restart server VideoStream and Sequency&Latency servers when it needed
 * @param endpoint - client endpoint
 * @param cap - VideoCapture object (camera)
 * @param address - remote endpoint address
 * @param timer - reconnect retry timer
 */
void watch_dog(websocket_endpoint &endpoint, VideoCapture &cap, const std::string address, int timer) {
    while (endpoint.get_status() != "Open") {
        ::usleep(timer);
        // if still dont't open connect
        if (endpoint.get_status() != "Open") {
            endpoint.connect(address);
        }
        // if status is Opened init camera
        if (endpoint.get_status() == "Open") {
            if (cap.is_initialized()) {
                cap.video_free();
                cap.video_init();
            }
        }
    }
}


/**
 *  void control_stream_cli(const std::string ip, const std::string port, std::mutex &mutex, bool &reconnect, int32_t threshold, bitrate_t &bt)
 *  @brief  Udp control server
 *  @param  ip          - udp control server ip
 *  @param  port        - udp control server port
 *  @param  mutex       - reconnect control mutex
 *  @param  reconnect   - flag to initialize reconnect
 *  @param  threshold   - latency threshold for reconnect, measured in milliseconds
 *  @param  bt          - bitrate control structure
 */
void control_stream_cli(const std::string ip, const std::string port, std::mutex &mutex, bool &reconnect, int32_t threshold, bitrate_t &bt) {
    // io service object
    boost::asio::io_service io_service;
    /** Create control client object */
    UdpControlConnectionCli client(io_service);
    /** Initialize stream credentials */
    client.init_stream(ip, port, "Radion");
    /** Open socket */
    client.open_connection();
    /** Run event loop */
    client.run(mutex, reconnect, threshold, bt);
    /** Close connection */
    client.close_connection();
}

/**
 * void init_bitrate(bitrate_t &bt, int quality, int8_t increase, int8_t decrease, int period)
 * @brief Initialize bitrate_t structure
 * @param bt - bitrate structure
 * @param quality - video resolution height (1080, 720, 480, 320, 240)
 * @param increase - bitrate increase step value, measured in percents
 * @param decrease - bitrate decrease step value, measured in percents
 * @param period - period of increase/decrease step, measured in milliseconds
 */
void init_bitrate(bitrate_t &bt, int quality, int8_t increase, int8_t decrease, int period) {
    /// map of resolutions and their min/max bitrate
    std::map<int, std::pair<int, int>> quality_map = {
            {1080, {272000, 13000000}},
            {720,  {271000, 13000000}},
            {480,  {72000,  10000000}},
            {320,  {71400,  410000}},
            {240,  {67000, 400000}}
    };

    /// check for min/max value for quality
    auto it = quality_map.find(quality);
    bt.bitrate = it->second.first;          /// initial bitrate value(min value, start from minimum)
    bt.min_bitrate = it->second.first;      /// min bitrate value for current resolution
    bt.max_bitrate = it->second.second;     /// max bitrate for current resolution
    bt.persent_add = increase;              /// value of bitrate increase step, measured in percents
    bt.percent_substruct = decrease;        /// value of bitrate decrease step, measured in percents
    bt.changes_period = period;             /// period of increase/decrease step, measured in milliseconds
    bt.stability_cnt = 0;                   /// sequence of bandwidth stability
    bt.changes = false;                     /// switch to change bitrate
    bt.last_changes = CUR_TIME_M_SEC;
}

void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}

char *uart_name = (char*)"/dev/ttyS0";//USB0 from converter

int baudrate = 115200;

bool stop = false;

void
quit_handler( int sig )
{
    stop = true;
}

int result;
 std::chrono::time_point<std::chrono::system_clock> init = std::chrono::system_clock::now();	    
 
 
void auto_read(Autopilot_Interface &autopilot_interface) {
    autopilot_interface.read_messages();
    }

int main(int argc, char **argv) {
    if (argc != 7) {
        std::cout << "usage ./" << argv[0] << " server_ip  dev_name(/dev/video*) width height fps bitrate" << std::endl;
        return 1;
    }

    std::string dev  =   argv[dev_name_n]; // cam name
    /* Video params */
    uint16_t width   =   static_cast<uint16_t >(std::stol(argv[width_n], nullptr, 10));      // frame width
    uint16_t height  =   static_cast<uint16_t >(std::stol(argv[height_n], nullptr, 10));     // frame height
    uint16_t fps     =   static_cast<uint16_t >(std::stol(argv[fps_n], nullptr, 10));        // stream fps
    uint32_t bitrate =   static_cast<uint16_t >(std::stol(argv[bitrate_n], nullptr, 10));    // stream bitrate

    bitrate_t bt_s;
    /// initialize structure
    init_bitrate(bt_s, height, 10, 30, 10000);

    //------------------------------------------------ mavlink --------------------------------
    int fd;/*File Descriptor*/

	printf("\n +----------------------------------+");
	printf("\n |        Serial Port Read          |");
	printf("\n +----------------------------------+");

	/*------------------------------- Opening the Serial Port -------------------------------*/

	/* Change /dev/ttyUSB0 to the one corresponding to your system */

        	fd = open("/dev/ttyS0",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   					/* O_RDWR   - Read/Write access to serial port       */
								/* O_NOCTTY - No terminal will control the process   */
								/* Open in blocking mode,read will wait              */


        	if(fd == -1)						/* Error Checking */
            	   printf("\n  Error! in Opening ttyS0  ");
        	else
            	   printf("\n  ttyS0 Opened Successfully ");


    Serial_Port serial_port(uart_name, baudrate);
    
    Autopilot_Interface autopilot_interface(&serial_port);

    serial_port.start();
    
    char coma = ',';
    std::ofstream my_file("data.txt");

	   

    std::vector<std::string> strVec;


    //------------------------------------------------ end mavlink ----------------------------

    while(true) {
        try {
            // Endpoint object
            websocket_endpoint  endpoint;
            std::mutex          mtx;
            bool                reconnect = false;

            /// first attempt to connect
            int con_res = endpoint.connect(std::string("ws://" + std::string(argv[1]) + ":25090"));
            if (con_res < 0) {
                std::cerr << "Error" << std::endl;
            }
            /// initialize video cam object
            VideoCapture cap(dev, 1, width, height, fps, bitrate);
            /// try connect, if status !Open
//            watch_dog(endpoint, cap, std::string("ws://" + std::string(argv[1]) + ":25090"), 30000);

            /// check if camera initialized, if not initialize
            if (!cap.is_initialized()) {
                cap.video_init();
            }
            /// starting udp latency control stream
            std::thread udp_control(control_stream_cli, std::string(argv[1]), std::string("25085"), std::ref(mtx), std::ref(reconnect), 800, std::ref(bt_s));
            std::thread auto_serial(auto_read, std::ref(autopilot_interface));
            // Start camera loop
            cap.capture_video_stream(mtx, [&endpoint, reconnect, &autopilot_interface, &strVec, &my_file](void *buf, int size, int m_bt) {
                                            endpoint.send(buf, size);
    std::cout << "Some random text" << std::endl;
    // compare current and previous timestamps if diff > 1000 milliseconds(1 second)
    // show bitrate
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - init).count() >= 500) {
        autopilot_interface.read_messages();
	//autopilot_interface.write_send_message(&autopilot_interface, &strVec); //for debug
	std::cout << "lat "<< au_face.global_position_int.lat/1e7 << std::endl;
	std::cout << "lon "<< au_face.global_position_int.lon/1e7 << std::endl;
	std::cout << "lon "<< au_face.global_position_int.relative_alt/1000 << std::endl; //  /1000)
	std::cout << "fixtype "<< au_face.gps_raw_int.fix_type << std::endl;
	std::cout << "roll " << au_face.attitude.pitch*180/PI << std::endl;
	std::cout << "pitch " << au_face.attitude.roll*180/PI << std::endl;
        autopilot_interface.write_to_file_message(&autopilot_interface, &my_file);
        //strVec.clear(); 
        
        
        std::chrono::time_point<std::chrono::system_clock> init = std::chrono::system_clock::now();
    }
    std::cout << "bitrate in client lambda " << m_bt << std::endl;
					    ;},
                         reconnect, bt_s);
            /// release camera
            cap.video_free();
            /// check if we can join latency control stream
            if (udp_control.joinable()) {
                /// join stream
                udp_control.join();
            }
            /// close connection if we can
            endpoint.close(101, "");

        } catch (const char *err) {
        std::cout << err << std::endl;
        }
        catch (const std::string &err) {
            std::cout << err << std::endl;
        }
        catch(...) {
            std::cout << "some exception" << std::endl;
        }
    }

    return 0;
}
