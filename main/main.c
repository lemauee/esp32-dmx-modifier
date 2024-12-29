#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

static const char *TAG = "main";

static uint8_t data[DMX_PACKET_SIZE] = {};  // Buffer to store DMX data

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  dmx_driver_install(dmx_num, &config, NULL, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  dmx_packet_t packet;

  while (1) {
    TickType_t now = xTaskGetTickCount();

    // Block until a packet is received
    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
      ESP_LOGI(TAG, "DMX received.");

      dmx_read(dmx_num, data, DMX_PACKET_SIZE);
      ESP_LOGI(TAG, "Start code: %02x, Size: %i",
                packet.sc, packet.size);
      ESP_LOG_BUFFER_HEX(TAG, data, 16);  // Log first 16 bytes

      // Send data and block until it's sent
      dmx_send_num(dmx_num, DMX_PACKET_SIZE);
      dmx_wait_sent(dmx_num, DMX_TIMEOUT_TICK);
      dmx_write(dmx_num, data, DMX_PACKET_SIZE);
      ESP_LOGI(TAG, "Sent first DMX value 0x%02x", data[1]);
    } else {
      // DMX timed out after having been previously connected
      ESP_LOGI(TAG, "No DMX reveived.");
    }
    // Only send a packet every ms
    vTaskDelayUntil(&now, pdMS_TO_TICKS(1000));
  }
}
