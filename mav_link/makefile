all: mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lboost_filesystem -lboost_system -lboost_thread -lpthread

clean:
	 rm -rf *o mavlink_control
