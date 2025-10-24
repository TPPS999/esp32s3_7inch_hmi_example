// ESP32-S3 HMI for IFS BMS - Configuration loaded from secrets.h
#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "ui.h"
#include <WaveshareCAN.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ModbusServerWiFi.h>
#include "secrets.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// WiFi credentials (from secrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Fallback AP credentials (from secrets.h)
const char* ap_ssid = WIFI_AP_SSID;
const char* ap_password = WIFI_AP_PASSWORD;

WaveshareCAN can;
lv_obj_t * statusLabel = NULL;  // Global label for status updates
lv_obj_t * canLabel = NULL;     // Label for CAN frame display

// HMI Node ID for CANopen heartbeat (from secrets.h)
#define HMI_NODE_ID CAN_NODE_ID

// Timing variables
unsigned long lastHeartbeatTime = 0;

// Modbus TCP Server
ModbusServerWiFi MBserver;

// Test holding registers (function code 3/6/16)
uint16_t testRegisters[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// WiFi Serial (Telnet server - from secrets.h)
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;
bool telnetConnected = false;

// Helper function to print to both Serial and Telnet
void logPrint(const char* message) {
  Serial.print(message);
  if (telnetConnected && telnetClient && telnetClient.connected()) {
    telnetClient.print(message);
  }
}

void logPrintln(const char* message) {
  Serial.println(message);
  if (telnetConnected && telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  }
}

void logPrintf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logPrint(buffer);
}

// Modbus worker function for Read Holding Registers (FC03)
ModbusMessage FC03(ModbusMessage request) {
  ModbusMessage response;
  uint16_t startAddress;
  uint16_t numRegisters;

  // Parse request
  request.get(2, startAddress);  // Register address
  request.get(4, numRegisters);  // Number of registers

  // Check if address range is valid
  if (startAddress + numRegisters > 10) {
    // Out of range - return exception
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  // Build response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(numRegisters * 2));

  // Add register values
  for (uint16_t i = 0; i < numRegisters; i++) {
    response.add(testRegisters[startAddress + i]);
  }

  return response;
}

// Modbus worker function for Write Single Register (FC06)
ModbusMessage FC06(ModbusMessage request) {
  ModbusMessage response;
  uint16_t address;
  uint16_t value;

  // Parse request
  request.get(2, address);
  request.get(4, value);

  // Check if address is valid
  if (address >= 10) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  // Write the value
  testRegisters[address] = value;

  // Echo request as response (standard for FC06)
  return ECHO_RESPONSE;
}

// Function to send HMI heartbeat (CANopen NMT heartbeat)
void sendHMIHeartbeat() {
  // CANopen Heartbeat: COB-ID = 0x700 + Node ID
  uint32_t cobId = 0x700 + HMI_NODE_ID;  // 0x700 + 0x88 = 0x788
  uint8_t heartbeatData[1];
  heartbeatData[0] = 0x05;  // NMT state: Operational (0x05)

  if (can.sendMessage(cobId, heartbeatData, 1)) {
    logPrintf("Heartbeat sent: ID=0x%X Data=0x05\n", cobId);
  }
}

// Function to process received CAN frames
void processCAN() {
  // Check if CAN frames are available
  if (can.available() > 0) {
    uint32_t id;
    bool extended;
    uint8_t data[8];
    uint8_t length;

    // Read the CAN frame
    int result = can.receiveMessage(&id, &extended, data, &length);

    if (result > 0) {
      // Print to serial/telnet for debugging
      logPrintf("CAN RX: ID=0x%03X Len=%d Data=", id, length);
      for (int i = 0; i < length; i++) {
        logPrintf("%02X ", data[i]);
      }
      logPrintln("");

      // Check for NMT reset commands
      if (id == 0x000 && length >= 2) {
        uint8_t nmtCommand = data[0];
        uint8_t nodeId = data[1];

        // NMT Command 0x81 = Reset Node
        if (nmtCommand == 0x81) {
          if (nodeId == HMI_NODE_ID) {
            // Reset ESP32 completely
            logPrintln("!!! NMT RESET ESP32 COMMAND RECEIVED !!!");
            lvgl_port_lock(-1);
            lv_label_set_text(canLabel, "NMT: Resetting ESP32...");
            lv_obj_set_style_text_color(canLabel, lv_color_hex(0xFF0000), 0);
            lvgl_port_unlock();
            delay(500);
            ESP.restart();
          } else if (nodeId == 0x00) {
            // Reset CAN layer only (broadcast reset)
            logPrintln("!!! NMT RESET CAN LAYER COMMAND RECEIVED !!!");
            lvgl_port_lock(-1);
            lv_label_set_text(canLabel, "NMT: Resetting CAN layer...");
            lv_obj_set_style_text_color(canLabel, lv_color_hex(0xFFFF00), 0);
            lvgl_port_unlock();
            delay(100);

            // Reinitialize CAN
            can.end();
            delay(100);
            can.begin(CAN_250KBPS);

            lvgl_port_lock(-1);
            lv_label_set_text(canLabel, "CAN: Layer reset complete");
            lv_obj_set_style_text_color(canLabel, lv_color_hex(0x00FF00), 0);
            lvgl_port_unlock();
            logPrintln("CAN layer reset complete");
          }
        }
      }

      // Update display with last received frame (every frame)
      lvgl_port_lock(-1);
      char buf[100];
      snprintf(buf, sizeof(buf), "CAN: 0x%03X [%d] ", (unsigned int)id, length);

      // Add data bytes
      char dataBuf[30];
      char* ptr = dataBuf;
      for (int i = 0; i < length && i < 8; i++) {
        ptr += snprintf(ptr, dataBuf + sizeof(dataBuf) - ptr, "%02X ", data[i]);
      }
      strncat(buf, dataBuf, sizeof(buf) - strlen(buf) - 1);

      lv_label_set_text(canLabel, buf);
      lv_obj_set_style_text_color(canLabel, lv_color_hex(0x00FFFF), 0);
      lvgl_port_unlock();
    }
  }
}

// Declare the function that will be called from C code
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


void setup() {
  Serial.begin(115200);

  // Initialize the TWAI hardware (from secrets.h)
  // Note: When CAN is active (USB_SEL=HIGH), native USB doesn't work for programming
  // This is hardware limitation - GPIO19/20 shared between USB and CAN via FSUSB42UMX
  //can.initIOExpander();
  can.begin(CAN_250KBPS);

  Serial.println("Initializing board");
  Board *board = new Board();
  board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
  auto lcd = board->getLCD();
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto lcd_bus = lcd->getBus();
  if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
    static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
  }
#endif
#endif

  // Add a small delay for peripherals to stabilize - important for reliability!
  delay(100);

  // Check for initialization success - important for reliability!
  if (!board->begin()) {
    Serial.println("Board initialization failed! Check connections and restart.");
    while (1) { delay(1000); }  // Halt with error indication
  }

  Serial.println("Initializing LVGL");
  lvgl_port_init(board->getLCD(), board->getTouch());

  Serial.println("Creating UI");
  /* Lock the mutex due to the LVGL APIs are not thread-safe */
  lvgl_port_lock(-1);

  // Fill background with black
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);

  // Create main title label (from secrets.h)
  lv_obj_t * label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "ESP32-S3 HMI\n" DISPLAY_VERSION);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(DISPLAY_COLOR), 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

  // Create status label (will be updated with WiFi info)
  statusLabel = lv_label_create(lv_scr_act());
  lv_label_set_text(statusLabel, "Connecting to WiFi...");
  lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(statusLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(statusLabel, LV_ALIGN_CENTER, 0, 0);

  // Create CAN status label at bottom
  canLabel = lv_label_create(lv_scr_act());
  lv_label_set_text(canLabel, "CAN: Initializing...");
  lv_obj_set_style_text_font(canLabel, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(canLabel, lv_color_hex(0x808080), 0);
  lv_obj_set_style_text_align(canLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(canLabel, LV_ALIGN_BOTTOM_MID, 0, -20);

  lvgl_port_unlock();

  Serial.println("UI created, starting WiFi connection");

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  lvgl_port_lock(-1);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Update status label with IP
    char buf[150];
    snprintf(buf, sizeof(buf), "WiFi Connected!\n\nSSID: %s\nIP: %s\n\nOTA: Ready", ssid, WiFi.localIP().toString().c_str());
    lv_label_set_text(statusLabel, buf);
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x00FF00), 0);
  } else {
    Serial.println("\nWiFi Connection Failed!");
    Serial.println("Starting Access Point mode...");

    // Start AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress IP = WiFi.softAPIP();

    Serial.print("AP IP Address: ");
    Serial.println(IP);

    // Update status label
    char buf[150];
    snprintf(buf, sizeof(buf), "AP Mode\n\nSSID: %s\nPassword: %s\nIP: %s\n\nOTA: Ready",
             ap_ssid, ap_password, IP.toString().c_str());
    lv_label_set_text(statusLabel, buf);
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xFFFF00), 0);
  }
  lvgl_port_unlock();

  // Configure OTA (from secrets.h)
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);

    // Update display during OTA
    lvgl_port_lock(-1);
    lv_label_set_text(statusLabel, "OTA Update\n\nStarting...");
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xFFFF00), 0);
    lvgl_port_unlock();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    lvgl_port_lock(-1);
    lv_label_set_text(statusLabel, "OTA Update\n\nComplete!\nRebooting...");
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x00FF00), 0);
    lvgl_port_unlock();
    delay(1000);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    Serial.printf("Progress: %u%%\r", percent);

    // Update display every 10%
    static unsigned int lastPercent = 0;
    if (percent / 10 != lastPercent / 10) {
      lvgl_port_lock(-1);
      char buf[50];
      snprintf(buf, sizeof(buf), "OTA Update\n\n%u%%", percent);
      lv_label_set_text(statusLabel, buf);
      lvgl_port_unlock();
      lastPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    String errorMsg = "OTA Update\n\nError: ";
    if (error == OTA_AUTH_ERROR) errorMsg += "Auth Failed";
    else if (error == OTA_BEGIN_ERROR) errorMsg += "Begin Failed";
    else if (error == OTA_CONNECT_ERROR) errorMsg += "Connect Failed";
    else if (error == OTA_RECEIVE_ERROR) errorMsg += "Receive Failed";
    else if (error == OTA_END_ERROR) errorMsg += "End Failed";

    lvgl_port_lock(-1);
    lv_label_set_text(statusLabel, errorMsg.c_str());
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xFF0000), 0);
    lvgl_port_unlock();

    Serial.println(errorMsg);
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("Hostname: ");
  Serial.print(OTA_HOSTNAME);
  Serial.print(", IP: ");
  Serial.println(WiFi.localIP());

  // Initialize Modbus TCP Server (from secrets.h)
  Serial.println("Starting Modbus TCP Server...");

  // Register worker functions
  MBserver.registerWorker(MODBUS_SERVER_ID, READ_HOLD_REGISTER, &FC03);
  MBserver.registerWorker(MODBUS_SERVER_ID, WRITE_HOLD_REGISTER, &FC06);

  // Start server (config from secrets.h)
  MBserver.start(MODBUS_PORT, MODBUS_MAX_CLIENTS, MODBUS_TIMEOUT_MS);

  // Initialize test registers with some values
  testRegisters[0] = 1234;  // Test value 1
  testRegisters[1] = 5678;  // Test value 2
  testRegisters[2] = 9999;  // Test value 3

  Serial.print("Modbus TCP Server started on port ");
  Serial.println(MODBUS_PORT);
  Serial.print("  Server ID: ");
  Serial.println(MODBUS_SERVER_ID);
  Serial.println("  Function codes: 03 (Read Holding), 06 (Write Single)");
  Serial.println("  Test registers: 0-9 (10 registers)");

  // Start Telnet server for remote serial debugging (from secrets.h)
  #if TELNET_ENABLED
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.print("Telnet server started on port ");
  Serial.println(TELNET_PORT);
  Serial.print("  Connect with: telnet ");
  Serial.println(WiFi.localIP());
  #endif

  // Update CAN label after all services started
  lvgl_port_lock(-1);
  char canBuf[50];
  snprintf(canBuf, sizeof(canBuf), "CAN: Ready (Node 0x%02X)", CAN_NODE_ID);
  lv_label_set_text(canLabel, canBuf);
  lv_obj_set_style_text_color(canLabel, lv_color_hex(0x00FF00), 0);
  lvgl_port_unlock();

  Serial.println("\n=== Setup complete! ===");
  Serial.println("Services running:");
  Serial.print("  - OTA Update: port ");
  Serial.println(OTA_PORT);
  Serial.print("  - Modbus TCP: port ");
  Serial.println(MODBUS_PORT);
  #if TELNET_ENABLED
  Serial.print("  - Telnet Serial: port ");
  Serial.println(TELNET_PORT);
  #endif
  Serial.print("  - CAN Heartbeat: 0x");
  Serial.print(0x700 + CAN_NODE_ID, HEX);
  Serial.print(" @ ");
  Serial.print(CAN_HEARTBEAT_MS);
  Serial.println("ms");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Handle Telnet connections
  if (telnetServer.hasClient()) {
    // Check if we already have a client connected
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.flush();
      telnetConnected = true;
      logPrintln("\n*** Telnet client connected ***");
    } else {
      // Reject new client if one already connected
      WiFiClient newClient = telnetServer.available();
      newClient.stop();
    }
  }

  // Check if telnet client is still connected
  if (telnetConnected && (!telnetClient || !telnetClient.connected())) {
    logPrintln("\n*** Telnet client disconnected ***");
    telnetConnected = false;
    if (telnetClient) telnetClient.stop();
  }

  // CAN functions disabled to avoid bootloop - investigate later
  // TODO: Re-enable CAN heartbeat and processing after fixing bootloop issue
  /*
  // Send CAN heartbeat periodically (from secrets.h)
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= CAN_HEARTBEAT_MS) {
    sendHMIHeartbeat();
    lastHeartbeatTime = currentTime;
  }

  // Process incoming CAN frames
  processCAN();
  */

  // You don't need any special code in the loop to handle button clicks
  // The LVGL task handler (in lvgl_port_task) processes touch events
  // and calls the registered callbacks automatically

  // Your regular loop code goes here
  delay(10);  // Keep this short delay from your original working code
}