#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"

// -----------------------------------------------------------------------------
// Project settings
// -----------------------------------------------------------------------------

const char *FIRMWARE_NAME = "ESP32 Glider Controller";
const char *FIRMWARE_VERSION = "0.3.1";

// -----------------------------------------------------------------------------
// Board pin settings
// -----------------------------------------------------------------------------

#ifndef LED_BUILTIN
#define LED_BUILTIN 48
#endif

#define PUMP_ENABLE_PIN 4
#define PUMP_DIRECTION_PIN 5
#define PUMP_PWM_PIN 6

#define CAN_TX_PIN 17
#define CAN_RX_PIN 18

// -----------------------------------------------------------------------------
// WiFi access point settings
// -----------------------------------------------------------------------------

const char *apName = "ESP32_GLIDER";
WebServer server(80);

// -----------------------------------------------------------------------------
// PWM settings
// -----------------------------------------------------------------------------

const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 10;
const int pwmMaxDuty = 1023;

// -----------------------------------------------------------------------------
// System state
// -----------------------------------------------------------------------------

int pumpSpeedPercent = 0;
bool pumpEnabled = false;
bool pumpReverse = false;
bool canDriverInstalled = false;

uint32_t encoderValue = 0;
uint32_t canRxCount = 0;
uint32_t canTxCount = 0;

// -----------------------------------------------------------------------------
// Pump control
// -----------------------------------------------------------------------------

void applyPumpState()
{
  digitalWrite(PUMP_ENABLE_PIN, pumpEnabled ? HIGH : LOW);
  digitalWrite(PUMP_DIRECTION_PIN, pumpReverse ? LOW : HIGH);

  int duty = map(pumpSpeedPercent, 0, 100, 0, pwmMaxDuty);
  ledcWrite(pwmChannel, duty);
}

// -----------------------------------------------------------------------------
// CAN setup
// -----------------------------------------------------------------------------

bool startCAN()
{
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(
          (gpio_num_t)CAN_TX_PIN,
          (gpio_num_t)CAN_RX_PIN,
          TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("CAN driver install failed");
    return false;
  }

  if (twai_start() != ESP_OK)
  {
    Serial.println("CAN driver start failed");
    return false;
  }

  uint32_t alertsToEnable =
      TWAI_ALERT_RX_DATA |
      TWAI_ALERT_ERR_PASS |
      TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_RX_QUEUE_FULL;

  if (twai_reconfigure_alerts(alertsToEnable, NULL) != ESP_OK)
  {
    Serial.println("CAN alert configuration failed");
    return false;
  }

  canDriverInstalled = true;
  Serial.println("CAN driver started");
  return true;
}

String resetCAN()
{

  if (canDriverInstalled)
  {

    twai_stop();

    twai_driver_uninstall();

    canDriverInstalled = false;
  }

  if (startCAN())
  {

    return "CAN driver reset successfully";
  }

  return "CAN driver reset failed";
}
// -----------------------------------------------------------------------------
// CAN transmit
// -----------------------------------------------------------------------------

String sendCANTestMessage()
{
  if (!canDriverInstalled)
  {
    return "CAN driver not started";
  }

  twai_message_t message = {};

  message.extd = false;
  message.identifier = 0x01;
  message.data_length_code = 4;

  message.data[0] = 0x04;
  message.data[1] = 0x01;
  message.data[2] = 0x01;
  message.data[3] = 0x00;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
  {
    canTxCount++;
    Serial.println("CAN TEST message queued");
    return "CAN TEST sent: ID 0x01 DATA 04 01 01 00";
  }

  Serial.println("CAN TEST failed");
  return "CAN TEST failed to queue";
}

// -----------------------------------------------------------------------------
// CAN receive
// -----------------------------------------------------------------------------

void handleCANRxMessage(twai_message_t &message)
{
  canRxCount++;

  Serial.println("CAN message received");
  Serial.print("ID: 0x");
  Serial.println(message.identifier, HEX);

  Serial.print("DLC: ");
  Serial.println(message.data_length_code);

  Serial.print("Data: ");
  for (int i = 0; i < message.data_length_code; i++)
  {
    if (message.data[i] < 0x10)
      Serial.print("0");
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Seb's encoder decode logic.
  // Only decode when byte 2 indicates encoder data.
  if (message.data_length_code >= 7 && message.data[2] == 0x01)
  {
    encoderValue =
        message.data[3] |
        (message.data[4] << 8) |
        (message.data[5] << 16) |
        (message.data[6] << 24);

    Serial.print("Encoder value: ");
    Serial.println(encoderValue);
  }
}

// -----------------------------------------------------------------------------
// CAN polling
// -----------------------------------------------------------------------------

void pollCAN()
{
  if (!canDriverInstalled)
  {
    return;
  }

  uint32_t alertsTriggered;
  twai_read_alerts(&alertsTriggered, 0);

  twai_status_info_t twaiStatus;
  twai_get_status_info(&twaiStatus);

  if (alertsTriggered & TWAI_ALERT_ERR_PASS)
  {
    Serial.println("CAN alert: controller is error passive");
  }

  if (alertsTriggered & TWAI_ALERT_BUS_ERROR)
  {
    Serial.println("CAN alert: bus error");
    Serial.print("Bus error count: ");
    Serial.println(twaiStatus.bus_error_count);
  }

  if (alertsTriggered & TWAI_ALERT_RX_QUEUE_FULL)
  {
    Serial.println("CAN alert: RX queue full");
    Serial.print("RX buffered: ");
    Serial.println(twaiStatus.msgs_to_rx);
    Serial.print("RX missed: ");
    Serial.println(twaiStatus.rx_missed_count);
    Serial.print("RX overrun: ");
    Serial.println(twaiStatus.rx_overrun_count);
  }

  if (alertsTriggered & TWAI_ALERT_RX_DATA)
  {
    twai_message_t message;

    while (twai_receive(&message, 0) == ESP_OK)
    {
      handleCANRxMessage(message);
    }
  }
}

// -----------------------------------------------------------------------------
// Command handling
// -----------------------------------------------------------------------------

String handleCommand(String command)
{
  command.trim();
  command.toUpperCase();

  Serial.println("Command: " + command);

  if (command == "PING")
    return "PONG";

  if (command == "CAN TEST")
  {
    return sendCANTestMessage();
  }

  if (command == "CAN RESET")
  {
    return resetCAN();
  }

  if (command == "PUMP ON")
  {
    pumpEnabled = true;
    applyPumpState();
    return "Pump enabled";
  }

  if (command == "PUMP OFF")
  {
    pumpEnabled = false;
    applyPumpState();
    return "Pump disabled";
  }

  if (command == "DIR NORMAL")
  {
    pumpReverse = false;
    applyPumpState();
    return "Direction set to NORMAL";
  }

  if (command == "DIR REVERSE")
  {
    pumpReverse = true;
    applyPumpState();
    return "Direction set to REVERSE";
  }

  if (command.startsWith("SPEED "))
  {
    int speed = command.substring(6).toInt();
    speed = constrain(speed, 0, 100);
    pumpSpeedPercent = speed;
    applyPumpState();
    return "Speed set to " + String(speed) + "%";
  }

  if (command == "STATUS")
  {
    return "Firmware: " + String(FIRMWARE_NAME) +
           "\nVersion: " + String(FIRMWARE_VERSION) +
           "\nWiFi AP: " + String(apName) +
           "\nConnected clients: " + String(WiFi.softAPgetStationNum()) +
           "\nPump: " + String(pumpEnabled ? "ON" : "OFF") +
           "\nDirection: " + String(pumpReverse ? "REVERSE" : "NORMAL") +
           "\nSpeed: " + String(pumpSpeedPercent) + "%" +
           "\nCAN driver: " + String(canDriverInstalled ? "STARTED" : "NOT STARTED") +
           "\nCAN TX count: " + String(canTxCount) +
           "\nCAN RX count: " + String(canRxCount) +
           "\nEncoder value: " + String(encoderValue);
  }

  return "Unknown command: " + command;
}

// -----------------------------------------------------------------------------
// Web page
// -----------------------------------------------------------------------------

void handleRoot()
{
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Glider Controller</title>

  <style>
    body {
      font-family: Arial;
      margin: 0;
      padding: 24px;
      background: #111;
      color: #eee;
    }

    .card {
      max-width: 900px;
      margin: auto;
      background: #1b1b1b;
      padding: 24px;
      border-radius: 12px;
      border: 1px solid #333;
    }

    button {
      font-size: 16px;
      padding: 10px 16px;
      margin: 6px;
      cursor: pointer;
    }

    .command-row {
      margin-top: 20px;
      display: flex;
      gap: 10px;
    }

    input {
      flex: 1;
      font-size: 16px;
      padding: 10px;
    }

    #log {
      background: #000;
      color: #00ff66;
      padding: 14px;
      margin-top: 24px;
      height: 300px;
      overflow-y: auto;
      white-space: pre-wrap;
      font-family: monospace;
      border: 1px solid #444;
      border-radius: 8px;
    }

    h3 {
      margin-top: 24px;
      margin-bottom: 8px;
    }
  </style>
</head>

<body>
  <div class="card">
    <h1>ESP32 Glider Controller</h1>

    <button onclick="sendCommand('PING')">PING</button>
    <button onclick="sendCommand('STATUS')">STATUS</button>

    <h3>Pump Controls</h3>
    <button onclick="sendCommand('PUMP ON')">PUMP ON</button>
    <button onclick="sendCommand('PUMP OFF')">PUMP OFF</button>
    <button onclick="sendCommand('DIR NORMAL')">NORMAL</button>
    <button onclick="sendCommand('DIR REVERSE')">REVERSE</button>

    <h3>Speed</h3>
    <button onclick="sendCommand('SPEED 0')">0%</button>
    <button onclick="sendCommand('SPEED 25')">25%</button>
    <button onclick="sendCommand('SPEED 50')">50%</button>
    <button onclick="sendCommand('SPEED 75')">75%</button>
    <button onclick="sendCommand('SPEED 100')">100%</button>

    <h3>CAN Bus</h3>

<button onclick="sendCommand('CAN TEST')">CAN TEST</button>
<button onclick="sendCommand('CAN RESET')">CAN RESET</button>
<button onclick="sendCommand('STATUS')">CAN STATUS</button>

    <div class="command-row">
      <input id="cmd" type="text" placeholder="Example: CAN TEST">
      <button onclick="sendCustom()">Send</button>
    </div>

    <div id="log">ESP32 glider console ready.</div>
  </div>

  <script>
    function appendLog(text) {
      const log = document.getElementById('log');
      log.textContent += String.fromCharCode(10) + text;
      log.scrollTop = log.scrollHeight;
    }

    function sendCommand(cmd) {
      appendLog("> " + cmd);

      fetch('/command?cmd=' + encodeURIComponent(cmd))
        .then(response => response.text())
        .then(data => appendLog(data))
        .catch(error => appendLog("Error: " + error));
    }

    function sendCustom() {
      const input = document.getElementById('cmd');
      const cmd = input.value.trim();

      if (cmd.length > 0) {
        sendCommand(cmd);
        input.value = "";
      }
    }

    document.getElementById('cmd').addEventListener('keydown', function(event) {
      if (event.key === 'Enter') sendCustom();
    });
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

// -----------------------------------------------------------------------------
// Web server
// -----------------------------------------------------------------------------

void handleWebCommand()
{
  if (!server.hasArg("cmd"))
  {
    server.send(400, "text/plain", "Missing cmd argument");
    return;
  }

  server.send(200, "text/plain", handleCommand(server.arg("cmd")));
}

void startWebServer()
{
  server.on("/", handleRoot);
  server.on("/command", handleWebCommand);
  server.begin();
  Serial.println("Web server started");
}

// -----------------------------------------------------------------------------
// WiFi setup
// -----------------------------------------------------------------------------

void startAccessPoint()
{
  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(apName);

  Serial.println();
  Serial.print("SoftAP started: ");
  Serial.println(apStarted ? "YES" : "NO");
  Serial.println("Network name: " + String(apName));
  Serial.println("Security: OPEN");
  Serial.print("Open browser to: ");
  Serial.println(WiFi.softAPIP());
}

// -----------------------------------------------------------------------------
// Startup banner
// -----------------------------------------------------------------------------

void printStartupBanner()
{
  Serial.println();
  Serial.println("=================================");
  Serial.println(FIRMWARE_NAME);
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("=================================");
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  printStartupBanner();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(PUMP_DIRECTION_PIN, OUTPUT);

  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(PUMP_PWM_PIN, pwmChannel);

  applyPumpState();

  startAccessPoint();
  startWebServer();
  startCAN();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop()
{
  server.handleClient();
  pollCAN();
}