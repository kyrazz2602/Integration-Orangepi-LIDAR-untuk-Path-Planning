/**
 * AirGuard - ESP32 WiFiManager Firmware
 * 
 * This firmware runs on the ESP32 to handle WiFi configuration using tzapu's WiFiManager.
 * Once connected, it retrieves the SSID and password, then transmits them to the 
 * Arduino Mega via Serial.
 * 
 * Hardware Wiring:
 * - ESP32 TX (GPIO 1 / TX0)   -> Arduino Mega Serial3 RX (Pin 14)
 * - ESP32 RX (GPIO 3 / RX0)   -> Arduino Mega Serial3 TX (Pin 15)
 * - ESP32 GND                 -> Arduino Mega GND
 * 
 * Dependency:
 * - WiFiManager library by tzapu (install via Arduino Library Manager)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>

// Define Serial interface to Arduino Mega.
// By default, we use Serial (UART0), which is also connected to USB.
// If using separate pins, Serial2 (TX2/RX2 on GPIO 17/16) can be used.
#define ARDUINO_SERIAL Serial
#define SERIAL_BAUDRATE 115200

// Config portal SSID
const char* AP_SSID = "AirGuard_WiFi_Config";

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10000; // Send credentials every 10 seconds for robustness

void setup() {
  // Initialize Serial
  ARDUINO_SERIAL.begin(SERIAL_BAUDRATE);
  delay(1000);
  
  // Local debug print to IDE Monitor (if connected)
  ARDUINO_SERIAL.println("\n[ESP32] Initializing AirGuard WiFiManager...");

  WiFiManager wm;
  
  // Optional: Uncomment below line to clear saved WiFi credentials for testing
  // wm.resetSettings();

  // Set timeout for portal (if user doesn't connect, it will retry last saved credentials)
  wm.setConfigPortalTimeout(180); // 3 minutes timeout

  // Automatically start access point and configuration portal
  // It will block here until a connection is established or it times out
  if (!wm.autoConnect(AP_SSID)) {
    ARDUINO_SERIAL.println("[ESP32] Failed to connect to WiFi and portal timed out. Restarting...");
    delay(3000);
    ESP.restart();
  }

  // If we reach here, we are successfully connected to WiFi!
  ARDUINO_SERIAL.println("[ESP32] Successfully connected to WiFi!");
  
  // Immediately send the WiFi credentials to Arduino Mega
  sendWifiCredentials();
}

void loop() {
  // Periodically send WiFi credentials to Arduino Mega in case the Arduino/Orange Pi
  // starts up late, restarts, or loses serial sync.
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= SEND_INTERVAL) {
      lastSendTime = currentMillis;
      sendWifiCredentials();
    }
  } else {
    // If connection is lost, attempt to reconnect
    ARDUINO_SERIAL.println("[ESP32] WiFi connection lost. Retrying...");
    delay(5000);
  }
}

/**
 * Transmits WiFi credentials to Arduino Mega in the custom protocol format:
 * WIFI,<SSID>,<PASSWORD>\n
 */
void sendWifiCredentials() {
  String ssid = WiFi.SSID();
  String password = WiFi.psk();
  
  // Construct the command string
  String wifiCmd = "WIFI," + ssid + "," + password;
  
  // Send over the configured Serial port
  ARDUINO_SERIAL.println(wifiCmd);
}
