#include <Arduino.h>
#include <Wire.h>
#include <M5Unified.h>

static M5Canvas canvas_hist;

struct rect_t {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  void set(int16_t x_, int16_t y_, int16_t w_, int16_t h_) {
    x = x_;
    y = y_;
    w = w_;
    h = h_;
  }
};

static rect_t camera_area;
static rect_t histogram_area;
static int crosshair_x = 16; // Initial crosshair position (center of the screen)
static int crosshair_y = 12; // Initial crosshair position (center of the screen)
static constexpr int crosshair_size = 5; // Size of the crosshair
static constexpr size_t hist_count = 128;
static uint16_t histgram[hist_count] = {0};
static int16_t prev_x[hist_count] = { 0 };
static bool prev_line[hist_count] = { true };
static uint32_t frame_counter = 0;
static uint32_t fps = 0;

static constexpr int i2c_addr = 0x32;
static constexpr int pin_sda = 21;
static constexpr int pin_scl = 22;

#pragma pack(push)
#pragma pack(1)
struct temperature_info_t {
  uint16_t temp;
  uint8_t x;
  uint8_t y;
};
struct temp_data_t {
  int8_t refresh_control;
  uint8_t subpage;
  uint16_t med_temp;
  uint16_t avg_temp;
  temperature_info_t diff_info;
  temperature_info_t min_info;
  temperature_info_t max_info;
  uint16_t data[16 * 24];
};
#pragma pack(pop)

temp_data_t tempdatas[2];
int idx_recv = 0;
int idx_draw = 0;

static constexpr const uint32_t color_table[] = {
  0x0000FFu, 0x0003FFu, 0x0006FFu, 0x0009FFu, 0x000CFFu, 0x000FFFu, 0x0012FFu, 0x0016FFu,
  0x0019FEu, 0x001CFEu, 0x001FFEu, 0x0022FDu, 0x0025FDu, 0x0028FCu, 0x002BFCu, 0x002FFBu,
  0x0032FBu, 0x0035FAu, 0x0038F9u, 0x003BF9u, 0x003EF8u, 0x0041F7u, 0x0044F6u, 0x0047F6u,
  0x004AF5u, 0x004DF4u, 0x0050F3u, 0x0053F2u, 0x0056F1u, 0x0059F0u, 0x005CEFu, 0x005FEEu,
  0x0062ECu, 0x0065EBu, 0x0068EAu, 0x006AE9u, 0x006DE7u, 0x0070E6u, 0x0073E5u, 0x0076E3u,
  0x0079E2u, 0x007BE0u, 0x007EDFu, 0x0081DDu, 0x0084DCu, 0x0086DAu, 0x0089D8u, 0x008CD7u,
  0x008ED5u, 0x0091D3u, 0x0093D1u, 0x0096CFu, 0x0098CEu, 0x009BCCu, 0x009DCAu, 0x00A0C8u,
  0x00A2C6u, 0x00A5C4u, 0x00A7C2u, 0x00AAC0u, 0x00ACBEu, 0x00AEBCu, 0x00B1B9u, 0x00B3B7u,
  0x00B5B5u, 0x00B7B3u, 0x00B9B1u, 0x00BCAEu, 0x00BEACu, 0x00C0AAu, 0x00C2A7u, 0x00C4A5u,
  0x00C6A2u, 0x00C8A0u, 0x00CA9Du, 0x00CC9Bu, 0x00CE98u, 0x00CF96u, 0x00D193u, 0x00D391u,
  0x00D58Eu, 0x00D78Cu, 0x00D889u, 0x00DA86u, 0x00DC84u, 0x00DD81u, 0x00DF7Eu, 0x00E07Bu,
  0x00E279u, 0x00E376u, 0x00E573u, 0x00E670u, 0x00E76Du, 0x00E96Au, 0x00EA68u, 0x00EB65u,
  0x00EC62u, 0x00EE5Fu, 0x00EF5Cu, 0x00F059u, 0x00F156u, 0x00F253u, 0x00F350u, 0x00F44Du,
  0x00F54Au, 0x00F647u, 0x00F644u, 0x00F741u, 0x00F83Eu, 0x00F93Bu, 0x00F938u, 0x00FA35u,
  0x00FB32u, 0x00FB2Fu, 0x00FC2Bu, 0x00FC28u, 0x00FD25u, 0x00FD22u, 0x00FE1Fu, 0x00FE1Cu,
  0x00FE19u, 0x00FF16u, 0x00FF12u, 0x00FF0Fu, 0x00FF0Cu, 0x00FF09u, 0x00FF06u, 0x00FF03u,
  0x03FF00u, 0x06FF00u, 0x09FF00u, 0x0CFF00u, 0x0FFF00u, 0x12FF00u, 0x16FF00u, 0x19FE00u,
  0x1CFE00u, 0x1FFE00u, 0x22FD00u, 0x25FD00u, 0x28FC00u, 0x2BFC00u, 0x2FFB00u, 0x32FB00u,
  0x35FA00u, 0x38F900u, 0x3BF900u, 0x3EF800u, 0x41F700u, 0x44F600u, 0x47F600u, 0x4AF500u,
  0x4DF400u, 0x50F300u, 0x53F200u, 0x56F100u, 0x59F000u, 0x5CEF00u, 0x5FEE00u, 0x62EC00u,
  0x65EB00u, 0x68EA00u, 0x6AE900u, 0x6DE700u, 0x70E600u, 0x73E500u, 0x76E300u, 0x79E200u,
  0x7BE000u, 0x7EDF00u, 0x81DD00u, 0x84DC00u, 0x86DA00u, 0x89D800u, 0x8CD700u, 0x8ED500u,
  0x91D300u, 0x93D100u, 0x96CF00u, 0x98CE00u, 0x9BCC00u, 0x9DCA00u, 0xA0C800u, 0xA2C600u,
  0xA5C400u, 0xA7C200u, 0xAAC000u, 0xACBE00u, 0xAEBC00u, 0xB1B900u, 0xB3B700u, 0xB5B500u,
  0xB7B300u, 0xB9B100u, 0xBCAE00u, 0xBEAC00u, 0xC0AA00u, 0xC2A700u, 0xC4A500u, 0xC6A200u,
  0xC8A000u, 0xCA9D00u, 0xCC9B00u, 0xCE9800u, 0xCF9600u, 0xD19300u, 0xD39100u, 0xD58E00u,
  0xD78C00u, 0xD88900u, 0xDA8600u, 0xDC8400u, 0xDD8100u, 0xDF7E00u, 0xE07B00u, 0xE27900u,
  0xE37600u, 0xE57300u, 0xE67000u, 0xE76D00u, 0xE96A00u, 0xEA6800u, 0xEB6500u, 0xEC6200u,
  0xEE5F00u, 0xEF5C00u, 0xF05900u, 0xF15600u, 0xF25300u, 0xF35000u, 0xF44D00u, 0xF54A00u,
  0xF64700u, 0xF64400u, 0xF74100u, 0xF83E00u, 0xF93B00u, 0xF93800u, 0xFA3500u, 0xFB3200u,
  0xFB2F00u, 0xFC2B00u, 0xFC2800u, 0xFD2500u, 0xFD2200u, 0xFE1F00u, 0xFE1C00u, 0xFE1900u,
  0xFF1600u, 0xFF1200u, 0xFF0F00u, 0xFF0C00u, 0xFF0900u, 0xFF0600u, 0xFF0300u, 0xFF0000u,
};

static constexpr const int step_table[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, INT_MAX };

void recvTask(void*) {
  static constexpr uint8_t THERMAL2_REG_REFRESH_CTRL = 0x6E;
  uint32_t freq_i2c_default = 400000;
  uint32_t freq_i2c_read = 921600;

  for (;;) {
    M5.update();

    uint8_t send_data[2] = { THERMAL2_REG_REFRESH_CTRL, 2 };
    if (M5.Ex_I2C.start(i2c_addr, false, freq_i2c_default)
      && M5.Ex_I2C.write(send_data, 1)
      && M5.Ex_I2C.restart(i2c_addr, true, freq_i2c_read)
      && M5.Ex_I2C.read((uint8_t*)&tempdatas[idx_recv], 1)
      && tempdatas[idx_recv].refresh_control & 1) // has_new data none
    {
      if (M5.Ex_I2C.read(&((uint8_t*)&tempdatas[idx_recv])[1], sizeof(temp_data_t) - 1)) {
        idx_draw = idx_recv;
        idx_recv = !idx_recv;
      }
    }
    else {
      vTaskDelay(1);
    }
    M5.Ex_I2C.stop();
  }
}

void drawCrosshair() {
  int centerX = camera_area.x + camera_area.w / 2;
  int centerY = camera_area.y + camera_area.h / 2;
  int crosshairSize = 10;

  // Draw horizontal line
  M5.Display.drawLine(centerX - crosshairSize, centerY, centerX + crosshairSize, centerY, TFT_WHITE);
  // Draw vertical line
  M5.Display.drawLine(centerX, centerY - crosshairSize, centerX, centerY + crosshairSize, TFT_WHITE);
}

void drawToLCD(const temp_data_t* tempdata) {
  static int16_t values[32 * 24];
  static int min_temp;
  static int max_temp;

  if (tempdata->min_info.temp >= tempdata->max_info.temp) {
    return;
  }

  bool subpage = tempdata->subpage;

  if (subpage) {
    min_temp = tempdata->min_info.temp;
    max_temp = tempdata->max_info.temp;
  }

  int temp_diff = max_temp - min_temp;
  if (temp_diff == 0) return;
  int div = (1 << 24) / temp_diff;

  int centerX = camera_area.x + camera_area.w / 2;
  int centerY = camera_area.y + camera_area.h / 2;
  int crosshairTemp = 0;
  int sumTemp = 0;
  int count = 0;

  for (int y = 0; y < 24; ++y) {
    bool shift = (subpage ^ y) & 1;
    auto src = &tempdata->data[y * 16];

    auto value = &values[y * 32];
    for (int x = 0; x < 16; ++x) {
      value[(x << 1) + shift] = src[x];
    }
  }

  for (int y = 0; y < 24; ++y) {
    auto vy0 = camera_area.y + (camera_area.h * y / 24);
    auto vy1 = camera_area.y + (camera_area.h * (y + 1) / 24);
    bool shift = ((subpage ^ y) & 1);
    for (int x = 0; x < 16; ++x) {
      int xpos = (x << 1) + (shift);
      auto vx0 = camera_area.x + (camera_area.w * xpos >> 5);
      auto vx1 = camera_area.x + (camera_area.w * (xpos + 1) >> 5);
      int pn = xpos + y * 32;
      int temp = values[pn];

      temp = (temp - min_temp) * div >> 16;
      if (temp < 0) temp = 0;
      else if (temp > 255) temp = 255;
      histgram[temp >> 1]++;
      M5.Display.fillRect(vx0, vy0, vx1 - vx0, vy1 - vy0, color_table[temp]);

      // Check if the current pixel is within the 3x3 grid around the crosshair position
      if (abs(centerX - (vx0 + vx1) / 2) <= crosshair_size && abs(centerY - (vy0 + vy1) / 2) <= crosshair_size) {
        sumTemp += temp;
        count++;
      }
    }
  }

  ++frame_counter;
  static uint32_t prev_sec;
  uint32_t sec = millis() / 1000;
  if (prev_sec != sec) {
    prev_sec = sec;
    fps = frame_counter;
    frame_counter = 0;
  }

  if (subpage) {
    int prev_y_line = 0;
    int step_index = 0;
    while ((div * step_table[step_index] >> 8) < 32) { ++step_index; }
    int step = step_table[step_index];

    canvas_hist.setTextDatum(textdatum_t::bottom_right);
    canvas_hist.setTextPadding(0);
    int prev_temp = ((float)((((hist_count) << 17) / div) + min_temp) / 128 - 64);

    int hist_y = histogram_area.y + histogram_area.h - 1;
    int hist_x = histogram_area.x;

    M5.Display.setClipRect(histogram_area.x, histogram_area.y, histogram_area.w, histogram_area.h);
    for (int i = hist_count - 1; i >= -10; --i) {
      int temp = ((float)((((i + 1) << 17) / div) + min_temp) / 128 - 64);
      bool line = prev_temp / step != temp / step;
      uint32_t color = color_table[0];
      bool linedraw = line;
      if (i >= 0) {
        linedraw = prev_line[i] != line;
        prev_line[i] = line;
        int px = prev_x[i];
        int x = histgram[i];
        histgram[i] = 0;
        color = color_table[i << 1];
        uint32_t bgcolor = line ? ((color >> 1) & 0x7F7F7Fu) : ((color >> 2) & 0x3F3F3Fu);
        if (linedraw) {
          M5.Display.fillRect(hist_x + x, hist_y - i, histogram_area.w - x, 1, bgcolor);
        }
        canvas_hist.fillRect(x, hist_count - i - 1, histogram_area.w, 1, bgcolor);
        if (px != x) {
          prev_x[i] = x;
          M5.Display.fillRect(hist_x + x, hist_y - i, px - x, 1, x > px ? color : bgcolor);
          canvas_hist.fillRect(x, hist_count - i - 1, px - x, 1, x > px ? color : bgcolor);
        }
      }
      if (line) {
        int y = hist_count - i - 1;
        {
          uint32_t bgcolor = (color >> 2) & 0x3F3F3Fu;
          canvas_hist.setTextColor(color);
          canvas_hist.drawNumber(prev_temp, histogram_area.w, y);
        }
        prev_y_line = y + 1;
      }
      prev_temp = temp;
    }
    canvas_hist.pushSprite(&M5.Display, histogram_area.x, histogram_area.y);
    M5.Display.clearClipRect();
  }

  // Draw the crosshair
  drawCrosshair();

  // Calculate the average temperature
  int avgTemp = (count > 0) ? sumTemp / count : 0;

  // Display the average temperature at the crosshair position in Celsius
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextDatum(textdatum_t::top_center);
  M5.Display.drawString(String(avgTemp) + " °C", centerX, centerY + 15);
}

void setup(void) {
  M5.begin();
  M5.Power.setExtPower(true);
  M5.Ex_I2C.begin();

  M5.Display.setColorDepth(16);
  M5.Display.startWrite();
  M5.Display.print("Unit Thermal2 demo.");

  int_fast16_t width = M5.Display.width();
  int_fast16_t height = M5.Display.height();

  // Set camera_area to cover the entire screen
  camera_area.set(0, 0, width, height);
  // Optionally, set histogram_area to cover the entire screen or a part of it
  //histogram_area.set(0, 0, width, height);

  //canvas_hist.createSprite(histogram_area.w, histogram_area.h);

  delay(100);

  for (int i = 0; i < hist_count; ++i) {
    prev_line[i] = true;
    prev_x[i] = 0;
  }

  xTaskCreatePinnedToCore(recvTask, "recvTask", 8192, NULL, 5, NULL, PRO_CPU_NUM);
}

void loop(void) {
  drawToLCD(&tempdatas[idx_draw]);
  vTaskDelay(1); // Add a small delay to allow other tasks to run
}

