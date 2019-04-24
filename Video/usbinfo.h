#ifndef __USBINFO_H__
#define __USBINFO_H__

#include "libv4l2-hw.h"

struct interface_xu_descriptor {
  int8_t bLenght;
  int8_t bDescriptorType;
  int8_t bDescriptorSubType;
  int8_t bUnitID;
  u_int8_t guidExtensionCode[16];
  int8_t bNumControls;
  int8_t bNrInPins;
  int8_t bControlSize;
  int8_t iExtension;
} __attribute__ ((__packed__));
  

// http://www.ietf.org/rfc/rfc4122.txt
#define UUID_BE(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7) {		\
    ((a) >> 24) & 0xff, ((a) >> 16) & 0xff, ((a) >> 8) & 0xff, (a) & 0xff, \
      ((b) >> 8) & 0xff, (b) & 0xff,					\
      ((c) >> 8) & 0xff, (c) & 0xff,					\
      (d0), (d1), (d2), (d3), (d4), (d5), (d6), (d7) }

#define UUID_LE(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7) {              \
    (a) & 0xff, ((a) >> 8) & 0xff, ((a) >> 16) & 0xff, ((a) >> 24) & 0xff, \
      (b) & 0xff, ((b) >> 8) & 0xff,                                    \
      (c) & 0xff, ((c) >> 8) & 0xff,                                    \
      (d0), (d1), (d2), (d3), (d4), (d5), (d6), (d7) }

// from USB_Video_Class_1.1.pdf
#define USB_CC_VIDEO		0x0e
#define USB_SC_VIDEOCONTROL	1
#define USB_CS_DEVICE		0x21
#define USB_CS_INTERFACE	0x24

#define WARNING_LOG(...) {fprintf(stderr, "WW "); fprintf(stderr, __VA_ARGS__);}
#define ERROR_LOG(...) {fprintf(stderr, "EE "); fprintf(stderr, __VA_ARGS__);}


#endif
