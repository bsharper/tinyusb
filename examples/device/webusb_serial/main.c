#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "stm32f1xx.h"

// ---- Flash layout params for STM32F103C8 (Blue Pill) ----
// Flash base for STM32F1 is typically 0x08000000
#ifndef FLASH_BASE_ADDR
#define FLASH_BASE_ADDR 0x08000000UL
#endif

#ifndef FLASH_SIZE_BYTES
#define FLASH_SIZE_BYTES (64*1024UL) // 64KiB typical for STM32F103C8
#endif

#define PAGE_SIZE 1024UL
#define LAST_PAGE_ADDR (FLASH_BASE_ADDR + FLASH_SIZE_BYTES - PAGE_SIZE)
// Max payload we can store/read after MAGIC in the last page
#define MAX_DATA_LEN (PAGE_SIZE - sizeof(MAGIC))

static const uint32_t MAGIC = 0x44434F4E; // 'N' 'O' 'C' 'D' ("DCON" backwards endian)

static const char *CONTROLLER_VERSION = "0.2.0";
static const char *CONTROLLER_NAME = "Racoon Worx Drone Config Controller";

// Binary protocol definitions
#define CMD_FORMAT  0x01
#define CMD_READ    0x02
#define CMD_WRITE   0x03
#define CMD_DUMP    0x04
#define CMD_INFO    0x05
// LED control commands
#define CMD_LED_ON  0x06
#define CMD_LED_OFF 0x07

#define RESP_OK     0x00
#define RESP_ERROR  0x01
#define RESP_EMPTY  0x02
// Asynchronous event packet (device-initiated)
#define RESP_EVENT  0x10

// Packet buffer
// RX buffer must hold the largest possible incoming packet (header + payload)
#define RX_BUFSZ (3 + MAX_DATA_LEN)
static uint8_t rxbuf[RX_BUFSZ];
static size_t rxlen = 0;

// Prototypes
static void webusb_task(void);
static void handle_binary_packet(void);
static void send_response(uint8_t status, const uint8_t* data, uint16_t data_len);
static void send_event(uint8_t event_code, const uint8_t* data, uint16_t data_len);
static bool is_ascii_bytes(const uint8_t* data, size_t len);
static void reset_comm_state(void);
// LED pulse helpers
static void led_pulse(uint32_t dur_ms);
static void led_tick(void);

// Binary command prototypes
static void cmd_format_binary(void);
static void cmd_read_binary(void);
static void cmd_write_binary(const uint8_t* data, uint16_t data_len);
static void cmd_dump_binary(void);
static void cmd_info_binary(void);
static void cmd_led_on(void);
static void cmd_led_off(void);
// LED pulse state
static volatile uint32_t led_pulse_until_ms = 0;

// Flash helpers for STM32F1
// NOTE: This example uses the HAL-like registers via CMSIS headers provided by TinyUSB BSP.
// For safety and simplicity, we run from flash and only erase/program the last page.

static inline void flash_unlock(void) {
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123U;
    FLASH->KEYR = 0xCDEF89ABU;
  }
}

static inline void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static bool flash_wait_busy(void) {
  uint32_t to = 1000000;
  while (FLASH->SR & FLASH_SR_BSY) {
    if (!to--) return false;
  }
  return true;
}

static bool flash_erase_last_page(void) {
  flash_unlock();
  if (!flash_wait_busy()) return false;
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = LAST_PAGE_ADDR;
  FLASH->CR |= FLASH_CR_STRT;
  if (!flash_wait_busy()) return false;
  FLASH->CR &= ~FLASH_CR_PER;
  flash_lock();
  return true;
}

static bool flash_program_halfword(uint32_t addr, uint16_t data) {
  if (!flash_wait_busy()) return false;
  FLASH->CR |= FLASH_CR_PG;
  *(volatile uint16_t*)addr = data;
  bool ok = flash_wait_busy();
  FLASH->CR &= ~FLASH_CR_PG;
  return ok;
}

static bool flash_write(uint32_t addr, const void* src, size_t len) {
  // Program in half-words
  const uint8_t* p = (const uint8_t*)src;
  for (size_t i = 0; i < len; i += 2) {
    uint16_t hw = p[i];
    if (i + 1 < len) hw |= ((uint16_t)p[i+1]) << 8;
    if (!flash_program_halfword(addr + i, hw)) return false;
  }
  return true;
}

static bool is_magic_present(void) {
  return *(volatile uint32_t*)LAST_PAGE_ADDR == MAGIC;
}

static void init_empty_page(void) {
  // Erase and write magic only
  if (flash_erase_last_page()) {
    flash_unlock();
    flash_write(LAST_PAGE_ADDR, &MAGIC, sizeof(MAGIC));
    flash_lock();
  }
}


// Binary command implementations
static void cmd_format_binary(void) {
  if (!flash_erase_last_page()) {
    send_response(RESP_ERROR, (const uint8_t*)"ERASE_FAILED", 12);
    return;
  }
  flash_unlock();
  if (!flash_write(LAST_PAGE_ADDR, &MAGIC, sizeof(MAGIC))) {
    send_response(RESP_ERROR, (const uint8_t*)"WRITE_MAGIC_FAILED", 18);
    flash_lock();
    return;
  }
  flash_lock();
  send_response(RESP_OK, (const uint8_t*)"FORMATTED", 9);
  // Emit async event: format done
  uint8_t ev = 0x03; // EV_FORMAT
  send_event(ev, NULL, 0);
}

static void cmd_read_binary(void) {
  // Indicate activity with a visible LED pulse
  led_pulse(300);
  if (!is_magic_present()) {
    send_response(RESP_EMPTY, NULL, 0);
    return;
  }
  const uint8_t* p = (const uint8_t*)(LAST_PAGE_ADDR + sizeof(MAGIC));
  // Find first 0xFF (erased) byte to bound output, but allow 0x00 and newlines
  size_t raw_len = 0;
  for (size_t i = 0; i < MAX_DATA_LEN; i++) {
    uint8_t b = p[i];
    if (b == 0xFF) break;  // Only stop at erased flash, not null bytes or newlines
    raw_len++;
  }

  // Stream directly from flash without extra buffering
  send_response(RESP_OK, p, (uint16_t)raw_len);
}

static void cmd_write_binary(const uint8_t* data, uint16_t data_len) {
  // Indicate activity with a visible LED pulse
  led_pulse(300);
  // Reject non-ASCII in data
  if (!is_ascii_bytes(data, data_len)) {
    send_response(RESP_ERROR, (const uint8_t*)"NON_ASCII", 9);
    return;
  }
  
  // Truncate data if it's too long instead of rejecting
  uint16_t max_len = PAGE_SIZE - sizeof(MAGIC);
  uint16_t write_len = data_len;
  bool truncated = false;
  
  if (data_len > max_len) {
    write_len = max_len;
    truncated = true;
  }
  
  // Ensure page initialized
  if (!is_magic_present()) {
    init_empty_page();
  }
  flash_unlock();
  bool ok = flash_write(LAST_PAGE_ADDR + sizeof(MAGIC), data, write_len);
  flash_lock();
  
  if (ok) {
    if (truncated) {
      send_response(RESP_OK, (const uint8_t*)"TRUNCATED", 9);
    } else {
      send_response(RESP_OK, (const uint8_t*)"WROTE", 5);
    }
    // Emit async event: write done (len, truncated)
    uint8_t payload[4];
    payload[0] = 0x02; // EV_WRITE
    payload[1] = (uint8_t)(write_len & 0xFF);
    payload[2] = (uint8_t)((write_len >> 8) & 0xFF);
    payload[3] = truncated ? 1 : 0;
    send_response(RESP_EVENT, payload, sizeof(payload));
  } else {
    send_response(RESP_ERROR, (const uint8_t*)"WRITE_FAILED", 12);
  }
}

static void cmd_dump_binary(void) {
  // Indicate activity with a visible LED pulse
  led_pulse(300);
  // Send raw binary contents of the last flash page (1KB)
  const uint8_t* p = (const uint8_t*)LAST_PAGE_ADDR;
  send_response(RESP_OK, p, PAGE_SIZE);
}

static void cmd_info_binary(void) {
  char buf[96];
  int len = snprintf(buf, sizeof(buf), "FLASH_BASE=0x%08lX SIZE=%lu PAGE=%lu LAST=0x%08lX",
           (unsigned long)FLASH_BASE_ADDR, (unsigned long)FLASH_SIZE_BYTES,
           (unsigned long)PAGE_SIZE, (unsigned long)LAST_PAGE_ADDR);
  send_response(RESP_OK, (const uint8_t*)buf, len > 0 ? len : 0);
}

// LED command implementations
static void cmd_led_on(void) {
  board_led_write(true);
  send_response(RESP_OK, (const uint8_t*)"LED_ON", 6);
}

static void cmd_led_off(void) {
  board_led_write(false);
  send_response(RESP_OK, (const uint8_t*)"LED_OFF", 7);
}

int main(void) {
  board_init();

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  // Ensure LED starts off
  board_led_write(false);

  while (1) {
    tud_task(); // device
    webusb_task();
    led_tick();
  }
}

static void webusb_task(void) {
  if (!tud_vendor_available()) return;

  uint8_t buf[64];
  uint32_t count = tud_vendor_read(buf, sizeof(buf));
  
  for (uint32_t i = 0; i < count; i++) {
    if (rxlen < sizeof(rxbuf)) {
      rxbuf[rxlen++] = buf[i];
      
      // Check if we have a complete packet (minimum 3 bytes: cmd + length)
      if (rxlen >= 3) {
        uint16_t data_len = (uint16_t)(rxbuf[1] | (rxbuf[2] << 8));
        uint16_t expected_packet_len = 3 + data_len;
        
        if (rxlen >= expected_packet_len) {
          handle_binary_packet();
          // Shift remaining data if any
          if (rxlen > expected_packet_len) {
            memmove(rxbuf, rxbuf + expected_packet_len, rxlen - expected_packet_len);
          }
          rxlen -= expected_packet_len;
        }
      }
    } else {
      // Buffer overflow, reset
      rxlen = 0;
    }
  }
}

static void handle_binary_packet(void) {
  if (rxlen < 3) return;  // Need at least command + length
  
  uint8_t cmd = rxbuf[0];
  uint16_t data_len = (uint16_t)(rxbuf[1] | (rxbuf[2] << 8));
  uint8_t* data = (data_len > 0) ? &rxbuf[3] : NULL;
  
  switch (cmd) {
    case CMD_FORMAT:
      cmd_format_binary();
      break;
    case CMD_READ:
      cmd_read_binary();
      break;
    case CMD_WRITE:
      if (data_len > 0 && data != NULL) {
        cmd_write_binary(data, data_len);
      } else {
        send_response(RESP_ERROR, (const uint8_t*)"NO_DATA", 7);
      }
      break;
    case CMD_DUMP:
      cmd_dump_binary();
      break;
    case CMD_INFO:
      cmd_info_binary();
      break;
    case CMD_LED_ON:
      cmd_led_on();
      break;
    case CMD_LED_OFF:
      cmd_led_off();
      break;
    default:
      send_response(RESP_ERROR, (const uint8_t*)"UNKNOWN_CMD", 11);
      break;
  }
}

static void send_response(uint8_t status, const uint8_t* data, uint16_t data_len) {
  // Write header first
  uint8_t hdr[3];
  hdr[0] = status;
  hdr[1] = (uint8_t)(data_len & 0xFF);
  hdr[2] = (uint8_t)((data_len >> 8) & 0xFF);

  // Ensure there is space, retry if FIFO full
  while (tud_vendor_write_available() < sizeof(hdr)) {
    tud_task();
  }
  (void) tud_vendor_write(hdr, sizeof(hdr));
  tud_vendor_write_flush();

  // Stream payload in chunks to handle buffers/EPSIZE limits
  uint32_t sent = 0;
  while (sent < data_len) {
    uint32_t avail = tud_vendor_write_available();
    if (avail == 0) {
      tud_task();
      continue;
    }
    uint32_t remaining = (uint32_t)data_len - sent;
    uint32_t chunk = remaining < avail ? remaining : avail;
    if (chunk == 0) {
      tud_task();
      continue;
    }
    uint32_t wrote = tud_vendor_write(data + sent, chunk);
    sent += wrote;
    tud_vendor_write_flush();
  }
}

// Send an asynchronous event packet: first byte is event code, rest is data
static void send_event(uint8_t event_code, const uint8_t* data, uint16_t data_len) {
  // Send RESP_EVENT with payload: [event_code][data...]
  uint16_t total_len = (uint16_t)(1 + (data ? data_len : 0));
  uint8_t hdr[3] = { RESP_EVENT, (uint8_t)(total_len & 0xFF), (uint8_t)((total_len >> 8) & 0xFF) };

  while (tud_vendor_write_available() < sizeof(hdr)) {
    tud_task();
  }
  (void) tud_vendor_write(hdr, sizeof(hdr));
  tud_vendor_write_flush();

  // Write event code
  while (tud_vendor_write_available() < 1) {
    tud_task();
  }
  tud_vendor_write(&event_code, 1);
  tud_vendor_write_flush();

  // Stream any payload
  uint32_t sent = 0;
  uint32_t len = data ? data_len : 0;
  while (sent < len) {
    uint32_t avail = tud_vendor_write_available();
    if (avail == 0) { tud_task(); continue; }
    uint32_t remaining = len - sent;
    uint32_t chunk = remaining < avail ? remaining : avail;
    uint32_t wrote = tud_vendor_write(data + sent, chunk);
    sent += wrote;
    tud_vendor_write_flush();
  }
}


static bool is_ascii_bytes(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] > 0x7F) return false;
  }
  return true;
}

// Reset communication state (called on new connection)
static void reset_comm_state(void) {
  rxlen = 0;
  memset(rxbuf, 0, sizeof(rxbuf));
}

static inline void protocol_reset(void) {
  rxlen = 0;
}

// TinyUSB Callbacks ---------------------------------------------------------

void tud_mount_cb(void) {
  // Reset communication state when device is connected
  protocol_reset();
  reset_comm_state();
  // Emit async event: connected
  uint8_t ev = 0x01; // EV_CONNECTED
  char info[96];
  int n = snprintf(info, sizeof(info), "%s (%s)", CONTROLLER_NAME, CONTROLLER_VERSION);
  if (n < 0) n = 0;
  send_event(ev, (const uint8_t*)info, (uint16_t)n);
}

void tud_umount_cb(void) { }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void) { }

// Invoked when received new data (not used, we poll with tud_vendor_available())
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
  (void) itf; (void) buffer; (void) bufsize;
}

// --- LED pulse implementation ---
static void led_pulse(uint32_t dur_ms) {
  board_led_write(true);
  led_pulse_until_ms = board_millis() + dur_ms;
}

static void led_tick(void) {
  uint32_t until = led_pulse_until_ms;
  if (until == 0) return;
  // handle wrap-around safely
  if ((int32_t)(board_millis() - until) >= 0) {
    board_led_write(false);
    led_pulse_until_ms = 0;
  }
}
