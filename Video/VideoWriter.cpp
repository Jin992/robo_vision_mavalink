//
// Created by jin on 16.04.19.
//

#include "VideoWriter.h"
#include <ctime>
#include <iomanip>
#include <iostream>
#include <errno.h>
#include <cstring>

namespace {
    /**
     * std::string timestamp()
     * @brief generate current timestamp
     * @return - std::string with timestamp
     */
    std::string timestamp() {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer,sizeof(buffer),"%Y-%m-%d-%H:%M:%S",timeinfo);
        return std::string(buffer);
    }
}

/**
 * VideoWriter::VideoWriter()
 * @brief default constructor
 */
VideoWriter::VideoWriter()
: width_(0), height_(0)
{}

/**
 * VideoWriter::VideoWriter(uint8_t cam_no,uint16_t width, uint16_t height)
 * @brief Overloaded constructor
 * @param cam_no - camera system number
 * @param width  - resolution width
 * @param height - resolution height
 */
VideoWriter::VideoWriter(uint8_t cam_no,uint16_t width, uint16_t height)
: cam_no_(cam_no), width_(width), height_(height)
{}

/**
 * VideoWriter::_generate_name
 * @brief  generate name for video files
 * @return std::string name of new file
 */
std::string VideoWriter::_generate_name() {
    return timestamp() + "-cam" + std::to_string(cam_no_) + "-" + std::to_string(width_) + "x" + std::to_string(height_) + ".raw";
}

/**
 * VideoWriter::open
 * @brief create and open file to record stream
 */
void VideoWriter::open() {
    filename_ = _generate_name();
    std::cout << filename_ << std::endl;
    file_.open(filename_, std::ofstream::binary | std::ofstream::out);
    if (!file_.is_open()) {
        throw "VideoWriter: open: failed to open file";
    }
}

/**
 * VideoWriter::write(char *buffer, size_t b_size)
 * @brief write video stream to file
 * @param buffer
 * @param b_size
 */
void VideoWriter::write(char *buffer, size_t b_size) {
    file_.write(buffer, b_size);
}

void VideoWriter::close() {
    file_.close();
}