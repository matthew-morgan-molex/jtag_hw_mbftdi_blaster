#include <string>
#include <libusb-1.0/libusb.h>
#include "debug.h"

#define FTDI_VENDOR_ID 0x0403
#define FTDI_PRODUCT_ID 0x6011

void usb_detach_ftdi_sio(void)
{
    libusb_context* ctx = NULL;
    libusb_device** list;
    ssize_t         count;
    int             result;
    
    result = libusb_init(&ctx);
    if (result < 0)
    {
        return;
    }

    count = libusb_get_device_list(ctx, &list);
    if (count < 0)
    {
        libusb_exit(ctx);
        return;
    }

    for (int i = 0; i < count; i++)
    {
        struct libusb_device_descriptor desc;
        result = libusb_get_device_descriptor(list[i], &desc);
        if (result < 0)
        {
            continue;
        }

        if ((desc.idVendor == FTDI_VENDOR_ID) && (desc.idProduct == FTDI_PRODUCT_ID))
        {
            struct libusb_device *dev = list[i];
            struct libusb_device_handle *handle = NULL;
            unsigned char serial_number[32];
            std::string serial_str;
            result = libusb_open(dev, &handle);

            if (result < 0)
            {
                continue;
            }
            result = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial_number, sizeof(serial_number));
            if (result < 0)
            {
                continue;
            }
            serial_str = std::string((char*)serial_number);
            if ((serial_str.substr(0, 4) == std::string("ACVP")) ||
                (serial_str.substr(0, 3) == std::string("KCB")))
            {
                printd("do detach %s\n", serial_str.c_str());
                result = libusb_detach_kernel_driver(handle, 1);
                printd("detatch returned %d\n", result);
            }
            libusb_close(handle);
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
}
