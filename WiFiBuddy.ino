#include <WiFi.h>
#include <TinyGPSPlus.h>
#include "driver/i2c.h"
#include <LovyanGFX.hpp>
#include <math.h>

// ── PIN DEFINITIONS ──────────────────────────────────────────────────────────

#define GPS_BAUDRATE  9600
#define GPS_RX_PIN    16
#define GPS_TX_PIN    17

#define I2C_SDA_PIN   6
#define I2C_SCL_PIN   7
#define I2C_PORT      I2C_NUM_0
#define I2C_FREQ_HZ   400000

#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C

#define BUTTON_PIN    1

#define TFT_CS        10
#define TFT_DC        9
#define TFT_MOSI      11
#define TFT_CLK       12
#define TFT_MISO      13
#define TFT_RST       14

// ── LOVYANGFX CONFIG ─────────────────────────────────────────────────────────

class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;

  LGFX() {
    auto bcfg = _bus.config();
    bcfg.spi_host    = VSPI_HOST;
    bcfg.spi_mode    = 0;
    bcfg.freq_write  = 40000000;
    bcfg.freq_read   = 16000000;
    bcfg.spi_3wire   = false;
    bcfg.use_lock    = true;
    bcfg.dma_channel = 1;
    bcfg.pin_sclk    = TFT_CLK;
    bcfg.pin_mosi    = TFT_MOSI;
    bcfg.pin_miso    = TFT_MISO;
    bcfg.pin_dc      = TFT_DC;
    _bus.config(bcfg);
    _panel.setBus(&_bus);

    auto pcfg = _panel.config();
    pcfg.pin_cs           = TFT_CS;
    pcfg.pin_rst          = TFT_RST;
    pcfg.pin_busy         = -1;
    pcfg.panel_width      = 240;
    pcfg.panel_height     = 320;
    pcfg.offset_x         = 0;
    pcfg.offset_y         = 0;
    pcfg.offset_rotation  = 0;
    pcfg.dummy_read_pixel = 8;
    pcfg.readable         = true;
    pcfg.invert           = false;
    pcfg.rgb_order        = false;
    pcfg.dlen_16bit       = false;
    pcfg.bus_shared       = false;
    _panel.config(pcfg);
    setPanel(&_panel);
  }
};

static LGFX tft;

// ── DATA STRUCTURES ──────────────────────────────────────────────────────────

struct WifiEntry {
  String  ssid;
  int32_t rssi;
};

#define MAX_WIFI 10
WifiEntry wifiData[MAX_WIFI];
int       wifiCount = 0;

double gpsLat       = 0.0;
double gpsLng       = 0.0;
double gpsSpeedKmph = 0.0;
double gpsAltitudeM = 0.0;
bool   gpsValid     = false;

float headingDeg = 0.0f;

// ── VIEW STATE ───────────────────────────────────────────────────────────────

enum View { VIEW_LIST = 0, VIEW_RADAR, VIEW_SCANNING };

View currentView     = VIEW_LIST;
View previousView    = VIEW_LIST;
View lastDrawnView   = (View)-1;  // force full redraw on first pass

// Radar sweep
static float         sweepAngle     = 0.0f;
static float         lastSweepAngle = -1.0f;
static unsigned long lastSweepMs    = 0;

// ── INTERNAL STATE ───────────────────────────────────────────────────────────

TinyGPSPlus gps;

static bool          scanInProgress  = false;
static unsigned long lastScanMs      = 0;
static unsigned long lastMpuMs       = 0;
static unsigned long lastRedrawMs    = 0;
static bool          lastButtonState = HIGH;

static float magAdjX = 1.0f;
static float magAdjY = 1.0f;
static float magAdjZ = 1.0f;

// ── I2C (ESP-IDF) ─────────────────────────────────────────────────────────────

static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(h, buf, 2, true);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r == ESP_OK;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, reg, true);
  i2c_master_start(h);
  i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, true);
  if (len > 1) i2c_master_read(h, out, len - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(h, out + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r == ESP_OK;
}

// ── IMU INIT & UPDATE ─────────────────────────────────────────────────────────

void initIMU() {
  i2c_config_t cfg = {};
  cfg.mode             = I2C_MODE_MASTER;
  cfg.sda_io_num       = I2C_SDA_PIN;
  cfg.scl_io_num       = I2C_SCL_PIN;
  cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ_HZ;
  i2c_param_config(I2C_PORT, &cfg);
  i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

  i2cWrite(MPU9250_ADDR, 0x6B, 0x00); delay(10);  // wake
  i2cWrite(MPU9250_ADDR, 0x37, 0x02); delay(10);  // bypass for AK8963

  // Read sensitivity adjustment from fuse ROM
  i2cWrite(AK8963_ADDR, 0x0A, 0x00); delay(10);
  i2cWrite(AK8963_ADDR, 0x0A, 0x0F); delay(10);
  uint8_t asa[3] = { 128, 128, 128 };
  if (i2cRead(AK8963_ADDR, 0x10, asa, 3)) {
    magAdjX = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
    magAdjY = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
    magAdjZ = ((float)(asa[2] - 128) / 256.0f) + 1.0f;
  }

  // Continuous measurement mode 2 (100 Hz, 16-bit)
  i2cWrite(AK8963_ADDR, 0x0A, 0x00); delay(10);
  i2cWrite(AK8963_ADDR, 0x0A, 0x16); delay(10);
}

void updateHeading() {
  uint8_t st1 = 0;
  if (!i2cRead(AK8963_ADDR, 0x02, &st1, 1) || !(st1 & 0x01)) return;
  uint8_t raw[7];
  if (!i2cRead(AK8963_ADDR, 0x03, raw, 7)) return;
  if (raw[6] & 0x08) return;  // overflow

  float fx = (float)(int16_t)((raw[1] << 8) | raw[0]) * magAdjX;
  float fy = (float)(int16_t)((raw[3] << 8) | raw[2]) * magAdjY;
  if (fx == 0.0f && fy == 0.0f) return;

  float a = 90.0f - (atan2f(fy, fx) * 180.0f / PI);
  if (a <   0.0f) a += 360.0f;
  if (a >= 360.0f) a -= 360.0f;
  headingDeg = a;
}

const char* toCardinal(float a) {
  if (a < 22.5f || a >= 337.5f) return "N";
  if (a <  67.5f) return "NE";
  if (a < 112.5f) return "E";
  if (a < 157.5f) return "SE";
  if (a < 202.5f) return "S";
  if (a < 247.5f) return "SW";
  if (a < 292.5f) return "W";
  return "NW";
}

// ── GPS ───────────────────────────────────────────────────────────────────────

void pollGPS() {
  while (Serial2.available()) gps.encode(Serial2.read());
  gpsValid = gps.location.isValid();
  if (gpsValid) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }
  if (gps.speed.isValid())    gpsSpeedKmph = gps.speed.kmph();
  if (gps.altitude.isValid()) gpsAltitudeM = gps.altitude.meters();
}

// ── WIFI ──────────────────────────────────────────────────────────────────────

void clearWiFi() {
  wifiCount = 0;
  for (int i = 0; i < MAX_WIFI; i++) { wifiData[i].ssid = ""; wifiData[i].rssi = -127; }
}

void sortWiFi() {
  for (int i = 0; i < wifiCount - 1; i++)
    for (int j = i + 1; j < wifiCount; j++)
      if (wifiData[j].rssi > wifiData[i].rssi) {
        WifiEntry t = wifiData[i]; wifiData[i] = wifiData[j]; wifiData[j] = t;
      }
}

void updateWiFi() {
  if (!scanInProgress) return;
  int result = WiFi.scanComplete();
  if (result == WIFI_SCAN_RUNNING || result == WIFI_SCAN_FAILED) return;

  clearWiFi();
  int limit = min(result, MAX_WIFI);
  for (int i = 0; i < limit; i++) {
    wifiData[i].ssid = WiFi.SSID(i).length() ? WiFi.SSID(i) : "(Hidden)";
    wifiData[i].rssi = WiFi.RSSI(i);
    wifiCount++;
  }
  sortWiFi();
  WiFi.scanDelete();
  scanInProgress = false;
  lastScanMs     = millis();

  // Return from scanning transition
  if (currentView == VIEW_SCANNING) currentView = previousView;
}

// ── DISPLAY HELPERS ──────────────────────────────────────────────────────────

int rssiToBars(int rssi) {
  if (rssi > -60) return 4;
  if (rssi > -70) return 3;
  if (rssi > -80) return 2;
  if (rssi > -90) return 1;
  return 0;
}

uint32_t rssiColor(int rssi) {
  if (rssi > -60) return tft.color565(0, 255, 102);
  if (rssi > -75) return tft.color565(255, 200, 0);
  return tft.color565(255, 50, 50);
}

// ── LIST VIEW ────────────────────────────────────────────────────────────────

void drawListView() {
  // Full redraw only on view switch
  if (lastDrawnView != VIEW_LIST) {
    tft.fillScreen(TFT_BLACK);
    lastDrawnView = VIEW_LIST;
  }

  uint32_t cyan = tft.color565(0, 246, 255);

  // Header
  tft.setTextSize(2);
  tft.setTextColor(cyan, TFT_BLACK);
  tft.setCursor(8, 6);
  tft.print("NETWORKS");

  tft.setTextSize(1);
  tft.setTextColor(tft.color565(0xCC, 0xCC, 0xCC), TFT_BLACK);
  String countStr = String(wifiCount) + " found";
  tft.setCursor(tft.width() - 8 - w, 10);
  tft.print(countStr);

  tft.drawLine(8, 26, tft.width() - 8, 26, tft.color565(0x44, 0x44, 0x44));

  // Network rows (up to 5)
  int startY    = 34;
  int rowHeight = 34;
  int display   = min(wifiCount, 5);

  for (int i = 0; i < display; i++) {
    int      y    = startY + i * rowHeight;
    uint32_t col  = rssiColor(wifiData[i].rssi);

    // Rank
    tft.setTextSize(1);
    tft.setTextColor(col, TFT_BLACK);
    tft.setCursor(8, y);
    tft.print(i + 1);

    // SSID
    tft.setCursor(20, y);
    String ssid = wifiData[i].ssid;
    if (ssid.length() > 12) ssid = ssid.substring(0, 11) + ".";
    tft.print(ssid);

    // RSSI
    tft.setCursor(20, y + 10);
    tft.setTextColor(tft.color565(0xCC, 0xCC, 0xCC), TFT_BLACK);
    tft.print(wifiData[i].rssi);
    tft.print(" dBm");

    // Signal bars
    int barX = tft.width() - 50;
    for (int b = 0; b < 4; b++) {
      uint32_t bc  = (b < rssiToBars(wifiData[i].rssi)) ? col : tft.color565(0x22, 0x22, 0x22);
      int      bh  = 6 + b * 3;
      int      by  = y + 4 + (4 - b) * 3;
      tft.fillRect(barX + b * 10, by, 8, bh, bc);
    }
  }

  // Clear empty rows
  for (int i = display; i < 5; i++)
    tft.fillRect(8, startY + i * rowHeight, tft.width() - 16, rowHeight - 2, TFT_BLACK);

  // GPS status line
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(0x77, 0x77, 0x77), TFT_BLACK);
  tft.setCursor(8, tft.height() - 20);
  if (gpsValid)
    tft.printf("%.5f, %.5f", gpsLat, gpsLng);
  else
    tft.print("GPS: no fix");
}

// ── RADAR VIEW ───────────────────────────────────────────────────────────────

#define RADAR_X  160
#define RADAR_Y  115
#define RADAR_R   90

void drawRadarBase() {
  tft.fillCircle(RADAR_X, RADAR_Y, RADAR_R, tft.color565(18, 18, 18));
  tft.drawCircle(RADAR_X, RADAR_Y,  30, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y,  60, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y,  90, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y, RADAR_R, tft.color565(80, 80, 80));
  tft.fillCircle(RADAR_X, RADAR_Y, 4, TFT_BLUE);
}

void drawNetworkDots() {
  // Place networks around radar based on RSSI distance, spread by index angle
  for (int i = 0; i < wifiCount; i++) {
    float dist  = constrain(map(wifiData[i].rssi, -100, -30, RADAR_R - 5, 5), 5, RADAR_R - 5);
    float angle = (360.0f / max(wifiCount, 1)) * i;
    float rad   = angle * DEG_TO_RAD;
    int   px    = RADAR_X + (int)(dist * sinf(rad));
    int   py    = RADAR_Y - (int)(dist * cosf(rad));
    uint32_t col = rssiColor(wifiData[i].rssi);
    tft.fillCircle(px, py, 4, col);
  }
}

void clearSweepLine() {
  if (lastSweepAngle < 0) return;
  float rad = lastSweepAngle * DEG_TO_RAD;
  int   x2  = RADAR_X + (int)(RADAR_R * sinf(rad));
  int   y2  = RADAR_Y - (int)(RADAR_R * cosf(rad));
  tft.drawLine(RADAR_X, RADAR_Y, x2, y2, tft.color565(18, 18, 18));
  // Restore rings
  tft.drawCircle(RADAR_X, RADAR_Y,  30, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y,  60, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y,  90, tft.color565(50, 70, 50));
  tft.drawCircle(RADAR_X, RADAR_Y, RADAR_R, tft.color565(80, 80, 80));
  // Restore dots
  drawNetworkDots();
}

void drawRadarView() {
  if (lastDrawnView != VIEW_RADAR) {
    tft.fillScreen(tft.color565(18, 18, 18));
    // Title bar
    tft.fillRect(0, 0, tft.width(), 28, tft.color565(0, 50, 0));
    tft.setTextColor(TFT_WHITE, tft.color565(0, 50, 0));
    tft.setTextSize(2);
    tft.setCursor(8, 6);
    tft.print("GPS RADAR");
    drawRadarBase();
    sweepAngle     = 0;
    lastSweepAngle = -1;
    lastDrawnView  = VIEW_RADAR;
  }

  // Sweep animation
  if (millis() - lastSweepMs >= 15) {
    lastSweepMs = millis();
    clearSweepLine();
    lastSweepAngle = sweepAngle;
    float rad = sweepAngle * DEG_TO_RAD;
    int x2 = RADAR_X + (int)(RADAR_R * sinf(rad));
    int y2 = RADAR_Y - (int)(RADAR_R * cosf(rad));
    tft.drawLine(RADAR_X, RADAR_Y, x2, y2, tft.color565(0, 200, 50));
    sweepAngle += 2.0f;
    if (sweepAngle >= 360.0f) { sweepAngle = 0; lastSweepAngle = -1; }
  }

  // Heading
  tft.fillRect(180, 0, tft.width() - 180, 28, tft.color565(0, 50, 0));
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(0x9C, 0xFB, 0x9C), tft.color565(0, 50, 0));
  tft.setCursor(182, 10);
  tft.printf("%.0f %s", headingDeg, toCardinal(headingDeg));

  // Bottom bar
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(0xCC, 0xCC, 0xCC), tft.color565(18, 18, 18));
  tft.setCursor(8, tft.height() - 14);
  tft.printf("Nets: %d", wifiCount);

  tft.setTextColor(tft.color565(0x77, 0x77, 0x77), tft.color565(18, 18, 18));
  tft.setCursor(10, 220);  // Position using cursor
  tft.print("Btn: List");
}

// ── SCANNING SCREEN ───────────────────────────────────────────────────────────

void drawScanningScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(tft.color565(0, 246, 255), TFT_BLACK);
  tft.setCursor(10, 150);  // Position using cursor
  tft.print("SCANNING...");
  lastDrawnView = VIEW_SCANNING;
}

// ── REDRAW DISPATCH ───────────────────────────────────────────────────────────

void redraw() {
  switch (currentView) {
    case VIEW_LIST:     drawListView();     break;
    case VIEW_RADAR:    drawRadarView();    break;
    case VIEW_SCANNING: drawScanningScreen(); break;
  }
}

// ── SETUP & LOOP ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  initIMU();

  tft.init();
  tft.setRotation(1);  // landscape

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  clearWiFi();

  lastScanMs = millis();
  redraw();
}

void loop() {
  pollGPS();

  // IMU heading every 120 ms
  if (millis() - lastMpuMs >= 120) {
    lastMpuMs = millis();
    updateHeading();
  }

  // Kick off async WiFi scan every 5 s (or immediately on startup)
  if (!scanInProgress && (wifiCount == 0 || millis() - lastScanMs >= 5000)) {
    previousView   = currentView;
    currentView    = VIEW_SCANNING;
    scanInProgress = true;
    WiFi.scanDelete();
    WiFi.scanNetworks(true, true);
  }

  updateWiFi();  // picks up completed scan, restores currentView

  // Button: toggle List <-> Radar (ignore during scanning)
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  if (pressed && !lastButtonState && currentView != VIEW_SCANNING) {
    currentView   = (currentView == VIEW_LIST) ? VIEW_RADAR : VIEW_LIST;
    lastDrawnView = (View)-1;  // force full redraw
  }
  lastButtonState = pressed;

  // Redraw at ~30 fps
  if (millis() - lastRedrawMs >= 33) {
    lastRedrawMs = millis();
    redraw();
  }

  delay(10);
}
