#include "VideoCapture.h"
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <err.h>
#include <pthread.h>
#include <time.h>
#include <iostream>
#include <numeric>

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

namespace {
    int xioctl(int fd, int request, void *arg) {
        int r;

        do {
            r = ioctl(fd, request, arg);
        } while(-1 == r && EINTR == errno);
        return r;
    }

    char *read_num(char *c, int *n) {
        while(*c >= '0' && *c <= '9') {
            *n = *n * 10 + *c - '0';
            c++;
        }
        return c;
    }
}

VideoCapture::VideoCapture()
: height_(480), width_(640), fps_(30 * 100),  bitrate_(0), current_cnt_(0), device_name_("/dev/video0"),
   fd(-1), io(IO_METHOD_USERPTR), fmt_(V4L2_PIX_FMT_RGB24), field(V4L2_FIELD_INTERLACED), cam_id_(0), initialized_(false)
{}

VideoCapture::VideoCapture(std::string  dev, uint8_t  cam_id, uint16_t width, uint16_t height, uint16_t fps, uint32_t bitrate)
: height_(height), width_(width), fps_(fps * 100), bitrate_(bitrate), current_cnt_(0), device_name_(dev),
  writer_(cam_id, width, height), cam_id_(cam_id), initialized_(false)
{
    writer_.open();
}

VideoCapture::~VideoCapture() {
    writer_.close();
}

void VideoCapture::set_params(std::string &dev, uint8_t cam_id, uint16_t width, uint16_t height, uint16_t fps, uint32_t bitrate) {
    device_name_ = dev;
    cam_id_ = cam_id;
    width_ = width;
    height_ = height;
    bitrate_ = bitrate;
    fps_ = fps * 100;
}


void VideoCapture::_measure_bitrate(int used_bytes, int &m_bt) {
    // save current frame bite value
    current_cnt_ += used_bytes;
    // get current timestamp
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    // compare current and previous timestamps if diff > 1000 milliseconds(1 second)
    // show bitrate
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - init_point_).count() >= 1000) {
        // some function to show bitrate currentlly print to stderr
        std::cerr << "Current bitrate : " << (float)(current_cnt_ * 8 * std::chrono::duration_cast<std::chrono::milliseconds>(now - init_point_).count() / 1000) / 1000000 << " Mbit/s    |" << std::endl;
        // after printing to output set counter to zero for next round
	m_bt = current_cnt_;
        current_cnt_ = 0;
        // set new timestamp
        init_point_ = std::chrono::system_clock::now();
    }
}

/* h264 packet output function*/
int VideoCapture::_process_image(int n, int used_bytes, std::function<void(void *, int, int)>func) {
    static int i = 0;
    i++;
    int32_t m_bt = 0;
    // memorize last buffer number
    buffer_last = n;
    // write frame to stdout
    //if (i > 100)
    // functor to process buffer
    _measure_bitrate(used_bytes, &m_bt);
    func(buffers[n].start, used_bytes, m_bt);
    //write to file

    //writer_.write(reinterpret_cast<char *>(buffers[n].start), used_bytes);
    // estimate bitrate per second
    //_measure_bitrate(used_bytes);

    if(needShot == 1) {
        needShot = 2;
    }
    return needShot ? 2 : 1;
}

/* Read data from camera driver */
int VideoCapture::_read_frame(std::function<void(void *, int, int)>func) {
    uint i, n = -1;
    int used = 0;
    // get data read method
    switch(io) {
        case IO_METHOD_READ: // directly read from driver buffer (with system call read(very slow))
            if(read(fd, buffers[0].start, buffers[0].length) == -1) {
                switch(errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */
                    default:
                        return -1;
                }
            }
            n = 0;
            break;
        case IO_METHOD_USERPTR: // store to user ptr
            bzero(&v4l2_buf, sizeof(struct v4l2_buffer));
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            v4l2_buf.memory = V4L2_MEMORY_USERPTR;
            if(xioctl(fd, VIDIOC_DQBUF, &v4l2_buf) == -1) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */
                    default:
                        return -1;
                }
            }
            for(i = 0; i < n_buffers; ++i)
                if(v4l2_buf.m.userptr == (unsigned long)buffers[i].start && v4l2_buf.length == buffers[i].length) {
                    n = i;
                    break;
                }
            if(xioctl(fd, VIDIOC_QBUF, &v4l2_buf) == -1) {
                return -1;
            }
            break;
        case IO_METHOD_MMAP: // direct access to camera driver buffer
        default:
            bzero(&v4l2_buf, sizeof(struct v4l2_buffer));
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            v4l2_buf.memory = V4L2_MEMORY_MMAP;
            if(xioctl(fd, VIDIOC_DQBUF, &v4l2_buf) == -1) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */
                    default:
                        return -1;
                }
            }

            n = v4l2_buf.index;
            used = v4l2_buf.bytesused;
            if(xioctl(fd, VIDIOC_QBUF, &v4l2_buf) == -1) {
                return -1;
            }
            break;
    }

    return n >= 0 ? _process_image(n, used, func) : 0;
}

/* set timeout to wait frame */
int VideoCapture::_shot(u_int32_t tout, std::function<void(void *, int, int)>func) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    /* Timeout. */
    tv.tv_sec = tout / 1000000;
    tv.tv_usec = tout % 1000000;
    r = select(fd + 1, &fds, NULL, NULL, (int)tout == -1 ? NULL : &tv);
    // error
    if(r <= 0) {
        // not error
        if(r < 0 && errno == EINTR) {
            r = 0;
        }
        if(!r || errno == EINTR) {
            return 0;
        }
        return r;
    }
    return _read_frame(func);
}

int VideoCapture::_stop_capturing() {
    enum v4l2_buf_type type;

    if(!start_ok) {
        goto l1;
    }
    switch(io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
        default:
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(fd, VIDIOC_STREAMOFF, &type);
            break;
    }
    l1:	start_ok = 0;
    return 1;
}

int VideoCapture::_start_capturing() {
    uint i;
    enum v4l2_buf_type type;

    switch(io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;
        case IO_METHOD_USERPTR:
            for(i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                bzero(&buf, sizeof(buf));
                buf.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory	= V4L2_MEMORY_USERPTR;
                buf.index	= i;
                buf.m.userptr	= (unsigned long)buffers[i].start;
                buf.length	= buffers[i].length;
                if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
                    error = 32 + 2;
                    break;
                }
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if(xioctl(fd, VIDIOC_STREAMON, &type) == -1){
                error = 32 + 2;
            }
            break;
        case IO_METHOD_MMAP:
        default:
            for(i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                bzero(&buf, sizeof(buf));
                buf.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory	= V4L2_MEMORY_MMAP;
                buf.index	= i;
                if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
                    error = 32 + 3;
                    break;
                }
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if(xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
                error = 32 + 4;
            }
            break;
    }
    start_ok = error ? 0 : 1;
    return error ? 0 : 1;
}

int VideoCapture::_uninit_device() {
    uint i;

    if(!init_ok) {
        goto l1;
    }
    switch(io) {
        case IO_METHOD_READ:
            free(buffers[0].start);
            break;
        case IO_METHOD_USERPTR:
            for(i = 0; i < n_buffers; ++i)
                free(buffers[i].start);
            break;
        case IO_METHOD_MMAP:
        default:
            for(i = 0; i < n_buffers; ++i)
                munmap(buffers[i].start, buffers[i].length);
            break;
    }
    free(buffers);
    buffers = 0;
    if(frame.start) {
        free(frame.start);
        frame.start = 0;
    }
    l1:	init_ok = 0;
    return 1;
}

int VideoCapture::_init_read(uint buffer_size) {
    buffers = (buffer *)calloc(1, sizeof(struct buffer));
    if(!buffers) {
        return 0;
    }
    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);
    if(!buffers[0].start) {
        free(buffers);
        buffers = 0;
        return 0;
    }
    return 1;
}

int VideoCapture::_init_mmap() {
    struct v4l2_requestbuffers req;

    bzero(&req, sizeof(req));
    req.count	= 4;
    req.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory	= V4L2_MEMORY_MMAP;

    if(xioctl(fd, VIDIOC_REQBUFS, &req) == -1){
        return 0;
    }
    if(req.count < 2){
        return 0;
    }
    buffers = (buffer *)calloc(req.count, sizeof(struct buffer));
    if(!buffers) {
        return 0;
    }

    int r = 1;
    for(n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        bzero(&buf, sizeof(buf));
        buf.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory	= V4L2_MEMORY_MMAP;
        buf.index	= n_buffers;
        if(xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            r = 0;
            break;
        }
        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(
                NULL /* start anywhere */,
                buf.length,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                fd, buf.m.offset
        );
        if(buffers[n_buffers].start == MAP_FAILED) {
            r = 0;
            break;
        }
    }
    if(!r) {
        // cleanup on error
        for(r = 0; r <= (int)n_buffers; r++) {
            if(buffers[r].start == MAP_FAILED) {
                continue;
            }
            munmap(buffers[r].start, buffers[r].length);
        }
        free(buffers);
        buffers = 0;
        return 0;
    }
    return 1;
}

int VideoCapture::_init_userp(uint buffer_size) {
    struct v4l2_requestbuffers req;
    uint page_size;

    page_size = getpagesize();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);
    bzero(&req, sizeof(req));
    req.count	= 4;
    req.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory	= V4L2_MEMORY_USERPTR;
    if(xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        return 0;
    }
    buffers = (buffer *)calloc(4, sizeof(struct buffer));
    if(!buffers) {
        return 0;
    }
    int r = 1;
    for(n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign(
                page_size,	/* boundary */
                buffer_size
        );
        if(!buffers[n_buffers].start) {
            r = 0;
            break;
        }
    }
    if(!r) {
        // cleanup on error
        for(r = 0; r <= (int)n_buffers; r++) {
            if(!buffers[r].start) {
                continue;
            }
            free(buffers[r].start);
        }
        free(buffers);
        buffers = 0;
        return 0;
    }
    return 1;
}

int VideoCapture::_init_device() {
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    uint min;

    if(error) {
        return 0;
    }
    error = 16 + 1;
    if(xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        return 0;
    }
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        error = 16 + 2;
        return 0;
    }
    switch(io) {
        case IO_METHOD_READ:
            if(!(cap.capabilities & V4L2_CAP_READWRITE)) {
                error = 16 + 3;
                return 0;
            }
            break;
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
        default:
            if(!(cap.capabilities & V4L2_CAP_STREAMING)) {
                error = 16 + 3;
                return 0;
            }
            break;
    }
    /* Select video input, video standard and tune here. */
    bzero(&cropcap, sizeof(cropcap));
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(!xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        if(xioctl(fd, VIDIOC_S_CROP, &crop) == -1) {
            switch(errno) {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    bzero(&fmt, sizeof(fmt));
    fmt.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width	= width_;
    fmt.fmt.pix.height	= height_;
    fmt.fmt.pix.pixelformat	= fmt_;
    fmt.fmt.pix.field	= field;
    if(xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        error = 16 + 4;
        return 0;
    }

    /* Note VIDIOC_S_FMT may change width and height. */
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if(fmt.fmt.pix.bytesperline < min) {
        fmt.fmt.pix.bytesperline = min;
    }
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if(fmt.fmt.pix.sizeimage < min) {
        fmt.fmt.pix.sizeimage = min;
    }
    bytesperline = fmt.fmt.pix.bytesperline;

    switch(io) {
        case IO_METHOD_READ:
            if(!_init_read(fmt.fmt.pix.sizeimage)) {
                error = 16 + 8;
                return 0;
            }
            break;
        case IO_METHOD_USERPTR:
            if(!_init_userp(fmt.fmt.pix.sizeimage)) {
                error = 16 + 8;
                return 0;
            }
            break;
        case IO_METHOD_MMAP:
        default:
            if(!_init_mmap()) {
                error = 16 + 8;
                return 0;
            }
            break;
    }
    error = 0;
    init_ok = 1;
    frame.length = raw ? buffers[0].length : width_ * height_ * 3;
    frame.start = malloc(frame.length);
    if(!frame.start) {
        _uninit_device();
        error = 16 + 9;
        return 0;
    }
    return 1;
}

int VideoCapture::_close_device() {
    int r = 1;
    if(fd != -1 && close(fd) == -1) {
        r = 0;
    }
    fd = -1;
    error = r ? 0 : 1;
    return r;
}

int VideoCapture::_open_device() {
    struct stat st;

    error = 1;
    if(stat(device_name_.c_str(), &st) == -1) {
        return 0;
    }
    if(!S_ISCHR(st.st_mode)) {
        return 0;
    }
    fd = open(device_name_.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);
    if(fd == -1) {
        return 0;
    }
    error = 0;
    return 1;
}

int VideoCapture::video_H264_set_bitrate(int bitrate) {
    // this Logitech cam not support
    if(!strcmp(name_.c_str(), "UVC Camera (046d:0828)")) {
        return -1;
    }
    uvcx_bitrate_layers_t b;
    b.wLayerID = 0;
    b.dwAverageBitrate = bitrate;
    b.dwPeakBitrate = bitrate;
    struct uvc_xu_control_query xu;
    xu.unit = H264_ctrl_unit_id;
    xu.selector = UVCX_BITRATE_LAYERS;
    xu.query = SET_CUR;
    xu.size = sizeof(b);
    xu.data = (unsigned char*)&b;
    int ret = xioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        warn("ioctl(UVCIOC_CTRL_QUERY)");
    }
    return ret;
}

int VideoCapture::video_H264_set_framerate(int framerate) {
    uvcx_framerate_config_t r;
    r.wLayerID = 0;
    r.dwFrameInterval = framerate;
    struct uvc_xu_control_query xu;
    xu.unit = H264_ctrl_unit_id;
    xu.selector = UVCX_FRAMERATE_CONFIG;
    xu.query = SET_CUR;
    xu.size = sizeof(r);
    xu.data = (unsigned char*)&r;
    int ret = xioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        warn("ioctl(UVCIOC_CTRL_QUERY)");
    }
    return ret;
}

int VideoCapture::video_H264_get_key_frame(int type) {
    u_int16_t len;
    struct uvc_xu_control_query xu;
    xu.unit = H264_ctrl_unit_id;
    xu.selector = UVCX_PICTURE_TYPE_CONTROL;
    xu.query = GET_LEN;
    xu.size = sizeof(len);
    xu.data = (unsigned char*)&len;
    int ret = xioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        warn("video_H264_get_key_frame: ioctl(UVCIOC_CTRL_QUERY)");
    }

    uvcx_picture_type_control_t c;
    c.wLayerID = 0;
    c.wPicType = type;
    xu.query = SET_CUR;
    if(len == sizeof(c)) {
        xu.size = sizeof(c);
        xu.data = (unsigned char*)&c;
    } else if(len == 2) {
        len = type;
    }
    ret = xioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        warn("video_H264_get_key_frame: ioctl(UVCIOC_CTRL_QUERY)");
    }
    return ret;
}

void VideoCapture::video_set_power_line_freq(int freq) {
    // anti flicker
    struct v4l2_queryctrl qctrl;
    struct v4l2_control ctrl;
    bzero(&qctrl, sizeof(qctrl));
    qctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
    if(xioctl(fd, VIDIOC_QUERYCTRL, &qctrl) != -1) {
        ctrl.id = qctrl.id;
        if(freq < 0) {
            ctrl.value = PL_FREQ;
        }
        else {
            ctrl.value = freq;
        }
        if(xioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
            warn("ioctl VIDIOC_S_CTRL");
        }
    }
}

int VideoCapture::video_init(void) {
    struct stat st;

    PL_FREQ = !stat("/D/60HZ", &st) ? V4L2_CID_POWER_LINE_FREQUENCY_60HZ : V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    FILE *f = fopen("/sys/class/video4linux/video0/name", "r");
    constexpr int MAX_STR = 10 * 1024;
    if(f) {
        char buf[MAX_STR];
        fgets(buf, MAX_STR, f);
        char *c = buf;
        while(*c && *c != 10) c++;
        *c = 0;
        fclose(f);
        name_ = buf;
        fprintf(stderr, "video %s is '%s'\n", device_name_.c_str(), buf);
    }
    f = fopen("/sys/class/video4linux/video0/device/uevent", "r");
    if(f) {
        char buf[MAX_STR];
        int l = fread(buf, 1, MAX_STR - 1, f);
        fclose(f);
        buf[l] = 0;
        fprintf(stderr, "uevent is '%s'\n", buf);
        uevent_ = buf;
        char *c = strstr(buf, "DEVICE=/proc/bus/usb/");
        if(c) {
            c = read_num(c + 21, &busnum);
        }
        if(c) {
            c = read_num(c + 1, &devnum);
        }
        fprintf(stderr, "busnum %d devnum %d\n", busnum, devnum);
    }
    //io = IO_METHOD_READ;
    io = IO_METHOD_MMAP;
    //io = IO_METHOD_USERPTR;

    fmt_ = V4L2_PIX_FMT_H264;
    int r = _open_device();
    if(!r) {
        fprintf(stderr, "open_device\n");
        exit(1);
    }

    struct v4l2hw info;
    bzero(&info, sizeof(info));
    info.busnum = busnum;
    info.devnum = devnum;
    if(v4l2_init(fd, &info) == -1) {
        err(1, "not a hardware encoding camera");
    }
    fprintf(stderr, "busnum=%d devnum=%d unit_ID=%d\n", info.busnum, info.devnum, info.unitId);
    H264_ctrl_unit_id = info.unitId;
    int ret;
    uvcx_video_config_probe_commit_t probeMax;
    uvcx_video_config_probe_commit_t probeCur;
    memset(&probeMax, 0, sizeof(probeMax));
    memset(&probeCur, 0, sizeof(probeCur));
    u_int16_t len = 0;
    struct uvc_xu_control_query xu;
    memset(&xu, 0, sizeof (xu));
    xu.unit = info.unitId;
    xu.selector = UVCX_VIDEO_CONFIG_PROBE;
    xu.query = GET_LEN;
    xu.size = sizeof(len);
    xu.data = (unsigned char*)&len;
    ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0 || (len != sizeof(probeMax) && len != 42)) {
        err(1, "ioctl(UVCIOC_CTRL_QUERY) 1");
    }
    xu.unit = info.unitId;
    xu.selector = UVCX_VIDEO_CONFIG_PROBE;
    xu.query = GET_MAX;
    xu.size = len; //sizeof(probeMax);
    xu.data = (unsigned char*)&probeMax;
    ret = xioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        err(1, "ioctl(UVCIOC_CTRL_QUERY) 2");
    }
    xu.unit = info.unitId;
    xu.selector = UVCX_VIDEO_CONFIG_PROBE;
    xu.query = GET_CUR;
    xu.size = len; //sizeof(probeCur);
    xu.data = (unsigned char*)&probeCur;
    ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        err(1, "ioctl(UVCIOC_CTRL_QUERY) 3");
    }
#define FPS2PERIOD(fps) ((int32_t)(1e9/fps))
    // Set parameters
    memset(&probeCur, 0, sizeof(probeCur));
    probeCur.bmHints = 0;
    probeCur.dwFrameInterval = FPS2PERIOD(3000);
    probeCur.bmHints |= BMHINTS_FRAME_INTERVAL;
    probeCur.wIFramePeriod = 500;
    probeCur.bmHints |= BMHINTS_IFRAMEPERIOD;
    probeCur.bRateControlMode = RATECONTROL_CBR;
    probeCur.bmHints |= BMHINTS_RATECONTROL;
    probeCur.dwBitRate = 1; /// probe bitrate
    probeCur.bmHints |= BMHINTS_BITRATE;
    probeCur.bUsageType = USAGETYPE_REALTIME;
    probeCur.bmHints |= BMHINTS_USAGE;

    probeCur.bmHints |= BMHINTS_ENTROPY;
    probeCur.bEntropyCABAC = ENTROPY_CABAC;
    probeCur.bmHints |= BMHINTS_PROFILE;
    probeCur.wProfile = PROFILE_HIGH;

    xu.unit = info.unitId;
    xu.selector = UVCX_VIDEO_CONFIG_COMMIT;
    xu.query = SET_CUR;
    xu.size = len; //sizeof(probeCur);
    xu.data = (unsigned char*)&probeCur;
    ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        err(1, "ioctl(UVCIOC_CTRL_QUERY) 4");
    }

    xu.unit = info.unitId;
    xu.selector = UVCX_VIDEO_CONFIG_PROBE;
    xu.query = GET_CUR;
    xu.size = len; //sizeof(probeCur);
    xu.data = (unsigned char*)&probeCur;
    memset(&probeCur, 0, sizeof(probeCur));
    ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if(ret < 0) {
        err(1, "ioctl(UVCIOC_CTRL_QUERY) 5");
    }
    if(probeCur.wWidth != 0 && probeCur.wHeight != 0) {
        printf("probe ok, should submit\n");
    }

    video_set_power_line_freq(-1);
    r = _init_device();
    if(!r) {
        err(1, "init_device");
    }
    initialized_ = true;
    return 0;
}

bool VideoCapture::is_initialized() const {
    return initialized_;
}

int VideoCapture::video_free(void) {
    int r = _stop_capturing();
    if(!r) {
        err(1, "stop_capturing");
    }
    r = _uninit_device();
    if(!r) {
        err(1, "uninit_device");
    }
    r = _close_device();
    if(!r) {
        err(1, "close_device");
    }
    initialized_ = false;
    return 0;
}

void VideoCapture::capture_video_stream(std::mutex &mutex, std::function<void(void *, int, int)>func, bool &stop, bitrate_t &bitrate) {
    int r = _start_capturing();
    if(!r) {
        err(1, "start_capturing");
    }
    struct timespec t, tf;
    char bad_cam = !strcmp(name_.c_str(), "UVC Camera (046d:0828)") ? 1 : 0;
    /*char need_set_pl_freq = */bad_cam && PL_FREQ == V4L2_CID_POWER_LINE_FREQUENCY_50HZ ? 1 : 0;
    //int32_t pts_diff = 0;
    //char live = 0;
    int i = 1;
    //video_H264_set_framerate(FPS2PERIOD(500));

    while(true) {
        mutex.lock();
        if (stop) {
            break;
        }
        mutex.unlock();
        if (clock_gettime(CLOCK_MONOTONIC_RAW, &t)) {
            err(1, "CLOCK_MONOTONIC_RAW");
        }
        if (need_key_frame) {
            need_key_frame = 0;
            tf = t;
            // 0=I-Frame, 1=IDR, 2=IDR+SPS+PPS
            fprintf(stderr, "video_H264_get_key_frame(IDR+SPS+PPS)\n");
            if (video_H264_get_key_frame(2) < 0) {
                fprintf(stderr, "video_H264_get_key_frame by stop/start capturing\n");
                _stop_capturing();
                _start_capturing();
            }
        }

        bitrate.bitrate_guard.lock();
        if (bitrate.changes) {
            video_H264_set_bitrate(bitrate.bitrate);
            bitrate.changes = false;
            std::cerr << "Bitrate changed to " << bitrate.bitrate <<std::endl;
        }
        bitrate.bitrate_guard.unlock();

        i++;
        mutex.lock();
        if (stop) {
            break;
        }
        mutex.unlock();
        r = _shot(1000000, func);
        //if (!r) continue;
        if (r < 0) {
            err(1, "shot");
        }
    }
}
