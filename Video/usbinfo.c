#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifndef MIPS
#include <libudev.h>
#endif
#include <libusb-1.0/libusb.h>
#include "usbinfo.h"

unsigned char GUID_UVCX_H264_XU[16] = UUID_LE(0xA29E7641, 0xDE04, 0x47E3, 0x8B, 0x2B, 0xF4, 0x34, 0x1A, 0xFF, 0x00, 0x3B);

void dump_videocontrol_interface(const unsigned char *buf, struct v4l2hw* info) {
  //buf[2] = VC Interface Descriptor Subtype
  if(buf[2] == 0x06) {  /* EXTENSION_UNIT */
    struct interface_xu_descriptor* xu_desc = (struct interface_xu_descriptor*)buf;
    xu_desc->bControlSize = *(&(xu_desc->bControlSize) + xu_desc->bNrInPins);
    xu_desc->iExtension = *(&(xu_desc->iExtension) + xu_desc->bNrInPins + xu_desc->bControlSize);
    assert(sizeof (*xu_desc) + xu_desc->bNrInPins + xu_desc->bControlSize == xu_desc->bLenght);
    if(strncmp((const char*)xu_desc->guidExtensionCode, (const char*)&GUID_UVCX_H264_XU, 16) == 0) {
      info->unitId = xu_desc->bUnitID;
      info->type = USB_UVC;
    }
  }
}

void dump_altsetting(const struct libusb_interface_descriptor *interface, struct v4l2hw* info) {
  const unsigned char *buf;
  unsigned size;
  
  /* avoid re-ordering or hiding descriptors for display */
  if(interface->extra_length) {
    size = interface->extra_length;
    buf = interface->extra;
    // buf[0] = bLenght
    // buf[1] = bDescriptorType
    while(size >= 2 * sizeof(u_int8_t)) {
      if(buf[0] < 2) break;
      
      /* This is the polite way to provide class specific
       * descriptors: explicitly tagged, using common class
       * spec conventions.
       */
      if(buf[1] == USB_CS_DEVICE || buf[1] == USB_CS_INTERFACE) {
    	  if(interface->bInterfaceClass == USB_CC_VIDEO && interface->bInterfaceSubClass == USB_SC_VIDEOCONTROL) {
    		  dump_videocontrol_interface(buf, info);
    	  }
      }
      size -= buf[0];
      buf += buf[0];
    }
  }
}

void dumpdev(libusb_device *dev, struct v4l2hw* info) {
  struct libusb_device_descriptor desc;
  int i, j, k, ret;

  libusb_get_device_descriptor(dev, &desc);
  if(desc.bNumConfigurations) {
    struct libusb_config_descriptor *config;
    for(i = 0; i < desc.bNumConfigurations; ++i) {
      ret = libusb_get_config_descriptor(dev, i, &config);
      if(ret) {
    	  WARNING_LOG("Couldn't get configuration descriptor %d, some information will be missing\n", i);
    	  continue;
      }
      for(j = 0 ; j < config->bNumInterfaces ; j++) {
    	  const struct libusb_interface *interface = &config->interface[j];
    	  for(k = 0; k < interface->num_altsetting; k++)
    		  dump_altsetting(&interface->altsetting[k], info);
      }
      libusb_free_config_descriptor(config);
    }
  }
}

int v4l2_init(int fd, struct v4l2hw *info) {
  struct stat sinfo;
  if(fstat(fd, &sinfo) == -1) {
    ERROR_LOG("fstat: %s", strerror(errno));
    return -1;
  }
  info->fd = fd;
  info->rdev = sinfo.st_rdev;
  info->type = NONE;
#ifndef MIPS
  struct udev* udevCtx = udev_new();
  struct udev_device* udevDevice = udev_device_new_from_devnum(udevCtx, 'c', sinfo.st_rdev);

  if (udevDevice == NULL) {
    ERROR_LOG("udev_device_new_from_devnum: %s", strerror(errno));
    return -1;
  }

  struct udev_device* parent = udev_device_get_parent_with_subsystem_devtype(udevDevice, "usb", "usb_device");
  
  if (parent == NULL) {
    ERROR_LOG("udev_device_get_parent_with_subsystem_devtype");
    udev_device_unref(udevDevice);
    udev_unref(udevCtx);
  }
  info->busnum = atoi(udev_device_get_sysattr_value(parent, "busnum"));
  info->devnum = atoi(udev_device_get_sysattr_value(parent, "devnum"));
  printf("busnum=%d devnum=%d\n", info->busnum, info->devnum);
#endif
  // discover devices
  libusb_device **list;
  libusb_context *ctx = NULL; //a libusb session
  libusb_init(&ctx); //initialize a library session

  ssize_t cnt = libusb_get_device_list(ctx, &list);
  fprintf(stderr, "libusb_get_device_list=%d\n", cnt);
  int i;
  if(cnt < 0) {
    ERROR_LOG("no usb device found");
    cnt = 0;
  }
  libusb_device *found = NULL;
  fprintf(stderr, "info busnum=%d devnum=%d\n", info->busnum, info->devnum);
  for(i = 0; i < cnt; i++) {
    libusb_device *device = list[i];
    fprintf(stderr, "dev=%d busnum=%d devnum=%d\n", i, libusb_get_bus_number(device), libusb_get_device_address(device));
    //struct libusb_device_descriptor ddesc;
    //int r = libusb_get_device_descriptor(device, &ddesc);
    //if(r) continue;
    if(info->busnum == libusb_get_bus_number(device) && info->devnum == libusb_get_device_address(device)) {
      found = device;
      fprintf(stderr, "found=%d\n\n\n", i);
      break;
    }
  }
  if(found) dumpdev(found, info);
  libusb_free_device_list(list, 1);
  libusb_exit(ctx);
#ifndef MIPS
  udev_device_unref(udevDevice);
  udev_unref(udevCtx);
#endif
  if(info->type != NONE) return 0;
  return -1;
}
