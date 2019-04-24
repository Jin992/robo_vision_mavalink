#ifndef __LIBV4L2_HW__
#define __LIBV4L2_HW__

/**
 * v4l2hw - video for linux 2 hardware
 */
struct v4l2hw {
  int fd;         /// file descriptor
  dev_t rdev;     /// device Id
  int busnum;     /// device bus num
  int devnum;     /// device num
  enum {
    NONE = 0,
    SKYPE_UVC,
    QUANTA,
    USB_UVC
  } type;
  int8_t unitId;  /// generated by system
};

int v4l2_init(int fd, struct v4l2hw* info);


#endif