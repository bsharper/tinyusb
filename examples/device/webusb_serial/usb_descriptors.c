#include "tusb.h"
#include <string.h>

#ifndef WEBUSB_URL
#define WEBUSB_URL "brianharper.us/drone/index.html"
#endif

// Vendor ID/Product ID - using TinyUSB default example VID/PID (for testing)
#define USB_VID   0xCAFE
#define USB_PID   0x4EE7

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
// Device descriptor
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0210, // at least 2.1 for BOS/WebUSB
  .bDeviceClass       = 0x00,   // per-interface
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// WebUSB Landing Page
//--------------------------------------------------------------------+

// URL descriptor created at runtime in control callback

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum {
  ITF_NUM_VENDOR, ITF_NUM_TOTAL
};

#define EPNUM_VENDOR_OUT    0x01
#define EPNUM_VENDOR_IN     0x81

uint8_t const desc_configuration[] = {
  // Config header
  // bLength, bDescriptorType, wTotalLength (LSB, MSB), bNumInterfaces, bConfigurationValue, iConfiguration, bmAttributes, bMaxPower
  9, TUSB_DESC_CONFIGURATION, (uint8_t)((9+9+7+7) & 0xFF), (uint8_t)(((9+9+7+7) >> 8) & 0xFF), 1, 1, 0x00, 0x80, 50,

  // Interface descriptor (vendor)
  9, TUSB_DESC_INTERFACE, ITF_NUM_VENDOR, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0x00,

  // Endpoint OUT
  7, TUSB_DESC_ENDPOINT, EPNUM_VENDOR_OUT, 0x02, 0x40, 0x00, 0x00,
  // Endpoint IN
  7, TUSB_DESC_ENDPOINT, EPNUM_VENDOR_IN,  0x02, 0x40, 0x00, 0x00,
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

char const* string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 }, // 0: supported language is English (0x0409)
  "Racoon Worx! without the !",                  // 1: Manufacturer
  "Drone Config WebUSB",       // 2: Product
  "0001",                      // 3: Serial
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count;

  if ( index == 0 )
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;
    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = (uint8_t) strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (uint8_t)(2*chr_count + 2);

  return _desc_str;
}

//--------------------------------------------------------------------+
// BOS Descriptor for WebUSB
//--------------------------------------------------------------------+

// WebUSB BOS descriptor: 1 device capability (WebUSB)
// Note: bVendorCode and iLandingPage are 1 byte each (not 2)
#define WEBUSB_BOS_TOTAL_LEN   (5 + 0x18)
#define WEBUSB_VENDOR_CODE     0x01
#define WEBUSB_LANDING_PAGE    0x01

uint8_t const desc_bos[] = {
  // BOS descriptor header
  0x05, TUSB_DESC_BOS, U16_TO_U8S_LE(WEBUSB_BOS_TOTAL_LEN), 0x01,

  // WebUSB Platform Capability Descriptor (length 0x18)
  0x18,                    // bLength
  TUSB_DESC_DEVICE_CAPABILITY, // bDescriptorType (Device Capability)
  0x05,                    // bDevCapabilityType (Platform)
  0x00,                    // bReserved
  // PlatformCapabilityUUID = {3408B638-09A9-47A0-8BFD-A0768815B665}
  0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
  0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
  U16_TO_U8S_LE(0x0100),   // bcdVersion 1.00
  WEBUSB_VENDOR_CODE,      // bVendorCode
  WEBUSB_LANDING_PAGE      // iLandingPage
};

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}

//--------------------------------------------------------------------+
// Vendor control request to get landing page
//--------------------------------------------------------------------+

// Make URL descriptor
// bLength, bDescriptorType (URL=0x03), bScheme (1=https), URL bytes
static uint8_t _url_desc[3 + sizeof(WEBUSB_URL)];
static bool url_desc_init = false;

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  (void) rhport; (void) stage;

  // Only handle at SETUP stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
      request->bRequest == WEBUSB_VENDOR_CODE)
  {
    if (!url_desc_init) {
      size_t url_len = strlen(WEBUSB_URL);
      _url_desc[0] = (uint8_t)(3 + url_len); // bLength
      _url_desc[1] = 0x03;                   // bDescriptorType: URL
      _url_desc[2] = 0x01;                   // bScheme: https
      memcpy(&_url_desc[3], WEBUSB_URL, url_len);
      url_desc_init = true;
    }
    return tud_control_xfer(rhport, request, _url_desc, _url_desc[0]);
  }

  // Stall other vendor requests
  return false;
}
