//
// Created by jin on 16.04.19.
//

#ifndef WB_VIDEOWRITER_H
#define WB_VIDEOWRITER_H


#include <string>
#include <fstream>

class VideoWriter {
public:
    VideoWriter();
    explicit VideoWriter(uint8_t cam_no, uint16_t width, uint16_t height);
    void write(char *buffer, size_t b_size);
    void open();
    void close();

private:
    std::string _generate_name();

private:
    uint8_t         cam_no_;
    uint16_t        width_;
    uint16_t        height_;
    std::string     filename_;
    std::ofstream   file_;




};


#endif //WB_VIDEOWRITER_H
