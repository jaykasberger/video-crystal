// LovyanGFX driver config for the Elecrow CrowPanel Advance 2.4" HMI (ESP32-S3).
// This file is project-local — each project has its own copy, so sharing
// the LovyanGFX library across projects with different hardware is fine.
//
// Derived from Elecrow's official LovyanGFX_Driver.h in
//   https://github.com/Elecrow-RD/CrowPanel-Advance-2.4-HMI-ESP32-S3-AI-Powered-IPS-Touch-Screen-320x240
// (example/example_code2.4/lesson-03/2_4LVGL)

#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  // Touch driver intentionally omitted: it would claim I2C port 0 on pins
  // 15/16, which we need free for Wire (BNO085 IMU on the external I2C
  // socket). Re-add Touch_FT5x06 here if/when touch input is wired back in.

 public:
  LGFX() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 80000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = 42;
      cfg.pin_mosi    = 39;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = 41;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 40;
      cfg.pin_rst          = -1;
      cfg.pin_busy         = -1;
      cfg.memory_width     = 240;
      cfg.memory_height    = 320;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 3;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};
