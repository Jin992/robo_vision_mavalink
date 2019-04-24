/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <iomanip>   //to get time for local time
#include "mavlink_control.h"
#include "autopilot_interface.h"

#include <ctime>

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

#include <fstream>  //to read and write to csv files
#include <iostream>

#include <thread>
#include <pthread.h>
#include <chrono>
#include <vector>
#include <signal.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define PI           3.14159265358979323846
//============================= auopilot interface ========================================
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
//pthread read_tid;


//Serial_Port serial_port(uart_name, baudrate);
//Autopilot_Interface autopilot_interface(&serial_port);
//========================================= ASIO =================================
//chrono::steady_clock sc;   // create an object of `steady_clock` class
static double rec_frame_count;
boost::system::error_code errorrr;

static bool write_to_file_start = false;
double server_got_bytes = 0;
bool keepGoing = true;
void shutdown(int)
{
	keepGoing = false;
	
}
std::size_t bytesAccum = 0;
void justReceive(boost::system::error_code ec, std::size_t bytesReceived,
    boost::asio::ip::tcp::socket &socket, std::vector<unsigned char> &buffer)
{

	//auto start = sc.now();     // start timer


	bytesAccum += bytesReceived;

	auto end = buffer.begin() + bytesReceived;
	for (auto it = buffer.begin(); it != end; ++it)
	{
		if (*it == 'e')
		{
			std::printf("server got: %lu\n", bytesAccum);
			//rec_frame_count++;
			//std::cout << "frames " << rec_frame_count << std::endl;
			write_to_file_start = true;
			server_got_bytes = ((double)bytesAccum/1000000)*10;

			bytesAccum = 0;
		}

	}

	socket.async_receive(
	    boost::asio::buffer(buffer, 2048),
	    0,
	    boost::bind(justReceive, _1, _2, boost::ref(socket), 
	                                     boost::ref(buffer)));

	//auto endd = sc.now();       // end timer (starting & ending is done by measuring the time at the moment the process started & ended respectively)
	//auto time_span = static_cast<chrono::duration<double>>(endd - start);   // measure time span between start & end
	//cout<<"Operation took: "<<time_span.count()<<" seconds !!!";
}
//------------------------------------------------ just my first app ----------------------------------
using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

string read_(tcp::socket & socket) {
       boost::asio::streambuf buf;
       boost::asio::read_until( socket, buf, "\n" );
       string data = boost::asio::buffer_cast<const char*>(buf.data());
       return data;
}

//------------------------------------------------------------------------------------------------------


int main(int argc, char *argv[])
{
//======================================================== mavink processing part ======================================

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
            	   printf("\n  Error! in Opening ttyUSB0  ");
        	else
            	   printf("\n  ttyUSB0 Opened Successfully ");


    Serial_Port serial_port(uart_name, baudrate);
    //---------------------------------------------- QSerialPort --------------------------------


    //-------------------------------------------------------------------------------------------
    Autopilot_Interface autopilot_interface(&serial_port);

    serial_port.start();
    //autopilot_interface.start();

    //serial_port_quit = &serial_port;
    //autopilot_interface_quit = &autopilot_interface;
    //signal(SIGINT,quit_handler);

			
                //================================== TABLE GPS FIX SIGNAL MEANING ========================
                /*
            if (true)
            {
                printf("time: %d:\n",                autopilot_interface.attitude.time_boot_ms);
                printf("roll: %f:\n",                autopilot_interface.attitude.roll*180/M_PI);
                printf("pitch: %f:\n",               autopilot_interface.attitude.pitch*180/M_PI);
                printf("yaw: %f:\n",                 autopilot_interface.attitude.yaw*180/M_PI);

                printf("lat: %f:\n",                 (float)(autopilot_interface.gps_raw_int.lat/1e7));
                printf("lon: %f:\n",                 (float)(autopilot_interface.gps_raw_int.lon/1e7));
                printf("alt: %d:\n\n",               (autopilot_interface.gps_raw_int.alt/1000));
                // 1-NO_FIX 2-2D_FIX 3-3D_FIX
                printf("fix_type: %d:\n",            (autopilot_interface.gps_raw_int.fix_type));
             }
              */
                
                /*
                <enum name="GPS_FIX_TYPE">
                             <description>Type of GPS fix</description>
                             <entry value="0" name="GPS_FIX_TYPE_NO_GPS">
                                 <description>No GPS connected</description>
                             </entry>
                             <entry value="1" name="GPS_FIX_TYPE_NO_FIX">
                                 <description>No position information, GPS is connected</description>
                             </entry>
                             <entry value="2" name="GPS_FIX_TYPE_2D_FIX">
                                 <description>2D position</description>
                             </entry>
                             <entry value="3" name="GPS_FIX_TYPE_3D_FIX">
                                 <description>3D position</description>
                             </entry>
                             <entry value="4" name="GPS_FIX_TYPE_DGPS">
                                 <description>DGPS/SBAS aided 3D position</description>
                             </entry>
                             <entry value="5" name="GPS_FIX_TYPE_RTK_FLOAT">
                                 <description>RTK float, 3D position</description>
                             </entry>
                             <entry value="6" name="GPS_FIX_TYPE_RTK_FIXED">
                                 <description>RTK Fixed, 3D position</description>
                             </entry>
                             <entry value="7" name="GPS_FIX_TYPE_STATIC">
                                 <description>Static fixed, typically used for base stations</description>
                             </entry>
                       </enum>
                  */

    //---------------------------------------------- ASIO SERVER -----------------------------------------
    signal(SIGINT, shutdown);

	int *p = NULL;
	boost::asio::io_service io;
	boost::asio::io_service::work work(io);

	boost::thread t1(boost::bind(&boost::asio::io_service::run, &io));
	boost::thread t2(boost::bind(&boost::asio::io_service::run, &io));
	boost::thread t3(boost::bind(&boost::asio::io_service::run, &io));
	boost::thread t4(boost::bind(&boost::asio::io_service::run, &io));

	boost::asio::ip::tcp::acceptor acceptor(io,
	    boost::asio::ip::tcp::endpoint(
	        boost::asio::ip::address::from_string("172.22.72.137"), 1125));
        
        acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

	boost::asio::ip::tcp::socket socket(io);

	// accept 1 client
	std::vector<unsigned char> buffer(2048, 0);

	acceptor.async_accept(socket, [&socket, &buffer](boost::system::error_code ec)
	{
	    // options
	    socket.set_option(boost::asio::ip::tcp::no_delay(true)); 
	    socket.set_option(boost::asio::socket_base::receive_buffer_size(1920 * 1080 * 4));
	    socket.set_option(boost::asio::socket_base::send_buffer_size(1920 * 1080 * 4));

	    socket.async_receive(
	        boost::asio::buffer(buffer, 2048),
	        0,
	        boost::bind(justReceive, _1, _2, boost::ref(socket),
	                                         boost::ref(buffer)));
	});

    //result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    //std::thread t5(autopilot_interface.read_messages());


	//============================== so so form a csv file =========================
	    char coma = ',';
	    ofstream my_file("data3.txt");

	    //std::vector<double> myvector (7);
	    // double* p_myvector = myvector.data();    // *p_my_vector = 345;

	    std::vector<string> strVec;
	    //std::stringstream ss;
	   // std::string cur_date_time;
	 //==========================================================================

	while (keepGoing)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		autopilot_interface.read_messages();


		autopilot_interface.server_got_bytes = server_got_bytes;
		autopilot_interface.gps_fix_type = autopilot_interface.gps_raw_int.fix_type;
		server_got_bytes = 0;


		//fio.open("sample.txt", ios::trunc | ios::out | ios::in);    trunc mode clear file before open, in will append
//		  my_file.open("data.txt", ios::in|ios::out|ios::app);
//		  my_file << autopilot_interface.attitude.time_boot_ms << setprecision(10) << ", "
//	        	 << server_got_bytes << setprecision(10) <<", "
//				 << autopilot_interface.gps_raw_int.lat/1e7 << setprecision(10) << ", "
//	             << autopilot_interface.gps_raw_int.lon/1e7 << setprecision(10) << ", "
//	             << autopilot_interface.gps_raw_int.alt/1000 << setprecision(10) << ", "
//	             //<< autopilot_interface.attitude.yaw*180/M_PI << setprecision(10) << ", "
//				 << autopilot_interface.gps_raw_int.fix_type << setprecision(10) << ", "
//	             << "\n";
//		  my_file.close();

		// autopilot_interface.cur_date_time.push_back(cur_date_time);
		
		 autopilot_interface.write_send_message(&autopilot_interface, &strVec);
		 if (write_to_file_start){
		      //string message = read_(socket);
			  //std::cout << "download speed is " << message << std::endl;
			 autopilot_interface.write_to_file_message(&autopilot_interface, &my_file);
			 write_to_file_start = false;
		 }

		 //boost::asio::write( socket, boost::asio::buffer(std::to_string(autopilot_interface.gps_raw_int.lat)));

		 strVec.clear();  //flush a vector data, we don't accumulate it
		  //================================= debug message ===========================================
		  /*
		  	  	  	std::printf("time: %d:\n",                autopilot_interface.attitude.time_boot_ms);
					std::printf("roll: %f:\n",                autopilot_interface.attitude.roll*180/M_PI);
					std::printf("pitch: %f:\n",               autopilot_interface.attitude.pitch*180/M_PI);
					std::printf("yaw: %f:\n",                 autopilot_interface.attitude.yaw*180/M_PI);

					std::printf("lat: %f:\n",                 (float)(autopilot_interface.gps_raw_int.lat/1e7));
					std::printf("lon: %f:\n",                 (float)(autopilot_interface.gps_raw_int.lon/1e7));
					std::printf("alt: %d:\n\n",               (autopilot_interface.gps_raw_int.alt/1000));
					// 1-NO_FIX 2-2D_FIX 3-3D_FIX
					std::printf("fix_type: %d:\n",            (autopilot_interface.gps_raw_int.fix_type));
			*/
		  //============================================================================================

					//p_myvector[0] =  (double)autopilot_interface.attitude.roll*180/M_PI;
					//p_myvector[1] =	 (double)autopilot_interface.attitude.pitch*180/M_PI;
					//p_myvector[2] =	 (double)autopilot_interface.attitude.yaw*180/M_PI;
					//p_myvector[3] =	 (double)autopilot_interface.gps_raw_int.lat/1e7;
					//p_myvector[4] =	 (double)autopilot_interface.gps_raw_int.lon/1e7;
					//p_myvector[5] =	 (double)autopilot_interface.gps_raw_int.alt/1000;
					//p_myvector[6] =	 (double)autopilot_interface.gps_raw_int.fix_type;
		  /*
		  std::cout << "myvector contains:";
					  for (unsigned i=0; i<myvector.size(); ++i)
					    std::cout << ' ' << myvector[i];
					  std::cout << '\n';

					  std::cout << myvector[4] << std::endl;
		  */

//		            strVec.clear();  //flush a vector data, we don't accumulate it
//
//					//strVec.push_back(std::to_string(autopilot_interface.attitude.roll*180/M_PI));
//					//strVec.push_back(std::to_string(autopilot_interface.attitude.pitch*180/M_PI));
//					//strVec.push_back(std::to_string(autopilot_interface.attitude.yaw*180/M_PI));
//					strVec.push_back(std::to_string(autopilot_interface.gps_raw_int.lat/1e7));
//					strVec.push_back(std::to_string(autopilot_interface.gps_raw_int.lon/1e7));
//					strVec.push_back(std::to_string(autopilot_interface.gps_raw_int.alt/1000));
//					strVec.push_back(std::to_string(autopilot_interface.gps_raw_int.fix_type));
//
//					for (auto i = strVec.begin(); i != strVec.end(); ++i)
//					    std::cout << *i << ' ' << std::endl;;
//
//					std::cout << "Vector size " << strVec.size() << std::endl;

					// = std::to_string(autopilot_interface.gps_raw_int.lat/1e7);

	}         

	//=========================================  sending ===========================================
//	// options to test
//		socket.set_option(boost::asio::ip::tcp::no_delay(true));
//		socket.set_option(boost::asio::socket_base::receive_buffer_size(1920 * 1080 * 4));
//		socket.set_option(boost::asio::socket_base::send_buffer_size(1920 * 1080 * 4));
//
//		std::vector<unsigned char> buffer(1920 * 1080 * 4, 0);
//		buffer.back() = 'e';
//
//		std::chrono::time_point<std::chrono::system_clock> last =
//		    std::chrono::system_clock::now();
//
//		std::chrono::duration<double> delta = std::chrono::seconds(0);
//
//		std::size_t bytesSent = 0;
	//==================================================================================================
	io.stop();

	t1.join();
	t2.join();
	t3.join();
	t4.join();
    
	std::printf("server: goodbye\n");

//---------------------------------------------------------------------------------------------------------	

		//serial_port.stop();
        //autopilot_interface.stop();

        //std::printf("client: goodbye\n");


        return 0;
}
//---------------------------------- END MAVLINK + CAM + LOG ----------------------------------------


