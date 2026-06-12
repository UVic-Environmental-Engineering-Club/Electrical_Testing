/* ESP32 TWAI receive example.
  Receive messages and sends them over serial.

  Connect a CAN bus transceiver to the RX/TX pins.
  For example: SN65HVD230

  TWAI_MODE_LISTEN_ONLY is used so that the TWAI controller will not influence the bus.

  The API gives other possible speeds and alerts:
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html

  Example output from a can bus message:
  -> Message received
  -> Message is in Standard Format
  -> ID: 604
  -> Byte: 0 = 00, 1 = 0f, 2 = 13, 3 = 02, 4 = 00, 5 = 00, 6 = 08, 7 = 00

  Example output with alerts:
  -> Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.
  -> Bus error count: 171
  -> Alert: The RX queue is full causing a received frame to be lost.
  -> RX buffered: 4  RX missed: 46 RX overrun 0

  created 05-11-2022 by Stephan Martin (designer2k2)
*/

#include "driver/twai.h"

#include "Arduino.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 5
#define TX_PIN 4
//#define RX_PIN 27
//#define TX_PIN 26

// Intervall:
#define POLLING_RATE_MS 1000
#define TRANSMIT_RATE_MS 500

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was send

uint32_t encoderValue = 0;

static void send_message() {
  // Send message

  // Configure message to transmit
  twai_message_t message;

    message.extd = false;  // Extended frame
    message.identifier = 1;
    message.data_length_code = 0x04;
    message.data[0] = 0x04;
    message.data[1] = 0x01;
    message.data[2] = 0x01;
    message.data[3] = 0x00;


  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to queue message for transmission\n");
  }
}

void setup() {
  // Start Serial:
  Serial.begin(115200);

  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

static void handle_rx_message(twai_message_t& message) {
  // Process received message
   if(message.data[2] == 0x01)
    {
        encoderValue = message.data[3] | (message.data[4] << 8) | (message.data[5] << 16) | (message.data[6] << 24);
    }


  Serial.printf("Encoder value: %d\n", encoderValue);
}

void loop() {
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }

    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    send_message();
  }
}