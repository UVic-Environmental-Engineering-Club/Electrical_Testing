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

#define PUMP_ENABLE_PIN 4    // Pump Enable
#define PUMP_DIRECTION_PIN 5 // Pump Direction
#define PUMP_PWM_PIN 6       // Pump Speed (PWM -> RC filter -> 0-4V input)

#define CAN_TX_PIN 43 // CAN TX -> SN65HVD230 TXD
#define CAN_RX_PIN 44 // CAN RX -> SN65HVD230 RXD

//#define DRAW_WIRE_SENSOR_POLL_INTERVAL_MS 1000 // Interval for polling the draw-wire sensor over CAN
#define DRAW_WIRE_SENSOR_POLL_INTERVAL_MS 500
#define CONTROL_LOOP_INTERVAL_MS 500

// thomas code
#define BALLAST_MIN_POSITION 35761 // maximum position of the ballast
#define BALLAST_MAX_POSITION 24794 // minimum position of the ballast
//#define BALLAST_TARGET_FILL_PERCENT 25 // target position of the ballast -- this needs to be set by the user or some other logic
int ballest_tank_target_fill_percent = 50;

int ballastPostion = 0; // current position of the ballast -- this needs to be updated from the encoder value

int integral = 0; // integral term to accumulate error over time
int previousError = 0; // previous error value for derivative calculation

// PID control parameters -- NEEDS TO BE TUNED FOR THE SYSTEM
float kp = 1; // proportional gain
float ki = 0; // integral gain
float kd = 0; // derivative gain
int dt = CONTROL_LOOP_INTERVAL_MS; // time step in milliseconds

#define DEADBAND 40

int decendFillPercent = 75;
int ascendFillPercent = 25;

int decentTime = 30000; // time in milliseconds to decend to the target fill percent

int testRunning = false;

//#define empty_encoder_value 35761
//#define full_encoder value 24794

// -----------------------------------------------------------------------------
// Free RTOS task
// -----------------------------------------------------------------------------

TaskHandle_t pollDrawWireSensorHandle = NULL; // Handle for the CAN polling task
TaskHandle_t receiveCANMessageHandle = NULL; // Handle for the CAN receive task
TaskHandle_t testRoutineHandle = NULL; // Handle for the CAN receive task


// -----------------------------------------------------------------------------
// WiFi access point settings
// -----------------------------------------------------------------------------

const char *apName = "ESP32_GLIDER";

WebServer server(80); // Web server listening on HTTP port 80

// -----------------------------------------------------------------------------
// PWM settings
// -----------------------------------------------------------------------------

const int pwmChannel = 0;      // ESP32 PWM generator/channel number
const int pwmFrequency = 5000; // PWM frequency in Hz (pump controller supports 1-25 kHz)
const int pwmResolution = 10;  // PWM resolution in bits (10-bit = 0-1023)
#define pwmMinDuty 300
#define pwmMaxDuty 1023   // Maximum duty cycle value for 10-bit PWM

// -----------------------------------------------------------------------------
// System state
// -----------------------------------------------------------------------------

bool pumpEnabled = true;        // True = pump enabled, False = pump disabled
bool canDriverInstalled = false; // True when CAN/TWAI driver is running
uint32_t encoderValue = 0;       // Latest encoder value received from CAN bus
uint32_t canRxCount = 0;         // Total CAN messages received
uint32_t canTxCount = 0;         // Total CAN messages transmitted

// -----------------------------------------------------------------------------
// Pump control
// Updates the physical pump control outputs
// -----------------------------------------------------------------------------

void setPumpState(int pumpEnabled, int pumpPWM, int pumpReverse)
{
  // Enable/disable pump controller.
  // HIGH = enabled
  // LOW  = disabled
  digitalWrite(PUMP_ENABLE_PIN, pumpEnabled ? HIGH : LOW);

  // Set pump direction.
  // HIGH = normal direction
  // LOW  = reverse direction
  digitalWrite(PUMP_DIRECTION_PIN, pumpReverse ? LOW : HIGH);

  // Output PWM speed command to the pump controller.
  ledcWrite(pwmChannel, pumpPWM);

  //Serial.println("Speed: " + String(pumpPWM));
}

// -----------------------------------------------------------------------------
// CAN setup
// Initializes the ESP32 TWAI (CAN) controller.
// Must be called before transmitting or receiving CAN messages.
// -----------------------------------------------------------------------------

bool startCAN()
{
  // General CAN controller configuration.
  // Uses the specified TX/RX pins and normal operating mode.
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(
          (gpio_num_t)CAN_TX_PIN,
          (gpio_num_t)CAN_RX_PIN,
          TWAI_MODE_NORMAL);

  // CAN bus speed.
  // Draw-wire sensor currently expected to operate at 500 kbps.
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Accept all incoming CAN messages.
  // Filtering can be added later if required.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install CAN driver.
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("CAN driver install failed");
    return false;
  }

  // Start CAN controller.
  if (twai_start() != ESP_OK)
  {
    Serial.println("CAN driver start failed");
    return false;
  }

  // Configure CAN alerts that should generate notifications.
  uint32_t alertsToEnable =
      TWAI_ALERT_RX_DATA |      // New CAN message received
      TWAI_ALERT_ERR_PASS |     // Controller entered error-passive state
      TWAI_ALERT_BUS_ERROR |    // Bus error detected
      TWAI_ALERT_RX_QUEUE_FULL; // Receive queue overflow

  if (twai_reconfigure_alerts(alertsToEnable, NULL) != ESP_OK)
  {
    Serial.println("CAN alert configuration failed");
    return false;
  }

  // CAN system is now operational.
  canDriverInstalled = true;

  Serial.println("CAN driver started");
  return true;
}

void testRoutine(void *parameter)
{
  ballest_tank_target_fill_percent = decendFillPercent;
  delay(decentTime);
  ballest_tank_target_fill_percent = ascendFillPercent;
  //End task
  testRoutineHandle = NULL;
  vTaskDelete(NULL);
}
// -----------------------------------------------------------------------------
// CAN transmit
// -----------------------------------------------------------------------------

void pollDrawWireSensorTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    if (canDriverInstalled)
    {

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
        //Serial.println("CAN TEST message queued");
      }
      else
      {
        Serial.println("CAN TEST failed");
      }
    }
    else{
      Serial.println("CAN driver not installed, cannot poll draw-wire sensor");
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DRAW_WIRE_SENSOR_POLL_INTERVAL_MS)); // Periodic interval between messages
  }
}

// -----------------------------------------------------------------------------
// PID control
// -----------------------------------------------------------------------------
void pidBallastControl(int encoderValue, int targetValue, int integral, int previousError, int dt)
{
  int error = targetValue - encoderValue;

  //Serial.print("Encoder Value: " + String(encoderValue));
  //Serial.print(" - Target Value: " + String(targetValue));
  //Serial.println("Error: " + String(error));

  // proportional term
  int p = kp * error;

  // integral term
  integral += error * dt;
  //integral = constrain(integral, BALLAST_MIN_POSITION, BALLAST_MAX_POSITION); // limit integral to prevent windup
  int i = ki * integral;

  // derivative term
  int derivative = (error - previousError) / dt;
  int d = kd * derivative;

  // PID output
  int output = p + i + d;



  setPumpState((abs(error) >= DEADBAND),constrain(abs(output), pwmMinDuty, pwmMaxDuty),(output <= 0));

  //update previous error for next iteration
  previousError = error;
  
}
  
// -----------------------------------------------------------------------------
// CAN polling
// -----------------------------------------------------------------------------

void receiveCANMessageTask(void *parameter)
{
  for (;;){
    if (canDriverInstalled)
    {
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

        //while (twai_receive(&message, portMAX_DELAY) == ESP_OK)
        while (twai_receive(&message, 10) == ESP_OK){

          canRxCount++;
          /*
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
          */

          // Seb's encoder decode logic.
          // Only decode when byte 2 indicates encoder data.
          if (message.data_length_code >= 7 && message.data[2] == 0x01)
          {
            encoderValue =
                message.data[3] |
                (message.data[4] << 8) |
                (message.data[5] << 16) |
                (message.data[6] << 24);

            //Serial.print("Encoder value: ");
            //Serial.println(encoderValue);
            
            //Map the target fill percentage to a target encoder value
            int target_value = map(ballest_tank_target_fill_percent, 0, 100, BALLAST_MIN_POSITION, BALLAST_MAX_POSITION);
            Serial.println(ballest_tank_target_fill_percent);
            if(pumpEnabled)
            {
              pidBallastControl(encoderValue, target_value, integral, previousError, dt);
            }else
            {
              setPumpState(0,0,0);
            }
          }
        }
      }
    }
    else{
      Serial.println("CAN driver not installed, cannot receive messages");
    }
  }
}
// -----------------------------------------------------------------------------
// Command handling
// Processes commands received from the web interface.
// Returns a text response that is displayed in the web console.
// -----------------------------------------------------------------------------

String handleCommand(String command)
{
  // Remove leading/trailing spaces and convert to uppercase
  // so commands are not case-sensitive.
  command.trim();
  command.toUpperCase();

  Serial.println("Command: " + command);

  // Simple communication test.
  if (command == "PING")
    return "PONG";
  // Software emergency stop.
  // Disables pump and sets speed command to 0%.
  if (command == "KILL")
  {
    pumpEnabled = false;
    setPumpState(0,0,0);
    return "EMERGENCY STOP: pump disabled and speed set to 0%";
  }

  if (command == "START TEST")
  {
    if(testRoutineHandle == NULL)
    {
      xTaskCreatePinnedToCore(
        testRoutine,          // Function to implement the task
        "testRoutine",       // Name of the task
        4096,                // Stack size in words
        NULL,                // Task input parameter
        1,                   // Priority of the task
        &testRoutineHandle,  // Task handle.
        0);                  // Core where the task should run
      return "Test routine started: descent fill";
    }
    else{
      return "Test routine already running";
    }
  }

  // Enable pump output.
  if (command == "PUMP ON")
  {
    pumpEnabled = true;
    return "Pump enabled";
  }

  // Set pump speed percentage.
  // Example:
  //   SPEED 50
  if (command.startsWith("SPEED "))
  {
    int speed = command.substring(6).toInt();

    // Limit speed to valid range.
    ballest_tank_target_fill_percent = constrain(speed, 0, 100);

    //pumpSpeedPercent = speed;
    //applyPumpState();

    return "Speed set to " + String(speed) + "%";
  }

  // Command not recognized.
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
    <button onclick="sendCommand('KILL')">KILL</button>
    <button onclick="sendCommand('PUMP ON')">PUMP ON</button>

    <h3>Test Routine</h3>
    <button onclick="sendCommand('START TEST')">START TEST</button>

    <h3>Fill %</h3>
    <button onclick="sendCommand('SPEED 0')">0%</button>
    <button onclick="sendCommand('SPEED 25')">25%</button>
    <button onclick="sendCommand('SPEED 50')">50%</button>
    <button onclick="sendCommand('SPEED 75')">75%</button>
    <button onclick="sendCommand('SPEED 100')">100%</button>

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

    let lastEncoderValue = null;
    function pollEncoderValue() {
      fetch('/encoder')
        .then(response => response.json())
        .then(data => {
          if (data.encoderValue !== lastEncoderValue) {
            lastEncoderValue = data.encoderValue;
            appendLog('Encoder value: ' + data.encoderValue);
          }
        })
        .catch(error => {
          console.error('Encoder poll failed', error);
        });
    }

    setInterval(pollEncoderValue, 1000);
    pollEncoderValue();

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
// Web server handlers
// -----------------------------------------------------------------------------

// Process commands received from the web interface.
void handleWebCommand()
{
  // Verify command parameter exists.
  if (!server.hasArg("cmd"))
  {
    server.send(400, "text/plain", "Missing cmd argument");
    return;
  }

  // Execute command and return response.
  server.send(200, "text/plain", handleCommand(server.arg("cmd")));
}

// Return the current encoder value as JSON.
void handleEncoderValue()
{
  String payload = "{\"encoderValue\":" + String(encoderValue) + "}";
  server.send(200, "application/json", payload);
}

// Configure and start the web server.
void startWebServer()
{
  server.on("/", handleRoot);              // Main webpage
  server.on("/command", handleWebCommand); // Command endpoint
  server.on("/encoder", handleEncoderValue); // Encoder status endpoint

  server.begin();

  Serial.println("Web server started");
}

// -----------------------------------------------------------------------------
// WiFi Access Point setup
// -----------------------------------------------------------------------------

void startAccessPoint()
{
  // Start WiFi access point.
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(apName);

  // Print connection information.
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

  startAccessPoint();
  startWebServer();
  startCAN();

  // Create a FreeRTOS task for polling the draw-wire sensor over CAN.
  xTaskCreate(
      pollDrawWireSensorTask,   // Task function
      "PollDrawWireSensor",    // Name of the task (for debugging)
      4096,                    // Stack size in bytes
      NULL,                    // Task input parameter (not used)
      1,                       // Task priority (0 = lowest)
      &pollDrawWireSensorHandle // Task handle (for later reference)
  );

  // Create a FreeRTOS task for receiving CAN messages.
  xTaskCreate(
      receiveCANMessageTask,   // Task function
      "ReceiveCANMessage",    // Name of the task (for debugging)
      4096,                    // Stack size in bytes
      NULL,                    // Task input parameter (not used)
      1,                       // Task priority (0 = lowest)
      &receiveCANMessageHandle // Task handle (for later reference)
  );
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop()
{
  server.handleClient();
}