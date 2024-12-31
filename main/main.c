#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "freertos/task.h"

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

#define TX2_PIN 1  // the pin we are using to TX with
#define RX2_PIN 3  // the pin we are using to RX with
#define EN2_PIN 22  // the pin we are using to enable TX on the DMX transceiver

static const char *TAG = "main";

static uint8_t data_in[DMX_PACKET_SIZE] = {};  // Buffer to store DMX in data
static uint8_t data_out[DMX_PACKET_SIZE] = {};  // Buffer to store DMX out data

void app_main() {
  const dmx_port_t dmx_num_receive = DMX_NUM_1;
  const dmx_port_t dmx_num_send = DMX_NUM_0;
  dmx_config_t config_receive = DMX_CONFIG_DEFAULT;
  dmx_config_t config_send = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  dmx_driver_install(dmx_num_receive, &config_receive, NULL, personality_count);
  dmx_driver_install(dmx_num_send, &config_send, NULL, personality_count);
  dmx_set_pin(dmx_num_receive, TX_PIN, RX_PIN, EN_PIN);
  dmx_set_pin(dmx_num_send, TX2_PIN, RX2_PIN, EN2_PIN);

  dmx_packet_t packet;

  while (1) {
    TickType_t now = xTaskGetTickCount();

    // Block until a packet is received
    if (dmx_receive(dmx_num_receive, &packet, DMX_TIMEOUT_TICK)) {
      // ESP_LOGI(TAG, "DMX received.");

      dmx_read(dmx_num_receive, data_in, DMX_PACKET_SIZE);
      // ESP_LOGI(TAG, "Start code: %02x, Size: %i",
      //          packet.sc, packet.size);
      // ESP_LOG_BUFFER_HEX(TAG, data, 16);  // Log first 16 bytes

      // PAR
      data_out[5] = data_in[1]; // Par rechts
      data_out[6] = data_in[1]; // Par rechts

      // Wandarme
      data_out[12] = 255; // Schalter ein 
      data_out[16] = data_in[2]; // TODO: Bereich einschränken?

      // Deckenlampen
      data_out[13] = data_in[3];

      // Stern
      data_out[14] = data_in[4];

      // Ganglicht
      data_out[15] = data_in[5];

      // Baum
      data_out[3] = data_in[6];

      // Root Par

      // Theater Spot 60 WRGBW

      // Flatbeam Duo 
      // Lagerfeuer
      // TODO: let everything happen in float
      float scaling = ((float)(data_in[16]))/((float)255.F);
      data_out[100] = (uint8_t)((float)(64 + ((uint8_t)esp_random())/2)*scaling);
      data_out[101] = (uint8_t)((float)(64 + ((uint8_t)esp_random())/2)*scaling);
      data_out[102] = (uint8_t)((float)(64 + ((uint8_t)esp_random())/2)*scaling);
      data_out[103] = (uint8_t)((float)(64 + ((uint8_t)esp_random())/2)*scaling);

      // Send data and block until it's sent
      dmx_send_num(dmx_num_send, DMX_PACKET_SIZE);
      dmx_wait_sent(dmx_num_send, DMX_TIMEOUT_TICK);
      dmx_write(dmx_num_send, data_out, DMX_PACKET_SIZE);
      // ESP_LOGI(TAG, "Sent first DMX value 0x%02x", data[1]);
    } else {
      // DMX timed out after having been previously connected
      // ESP_LOGI(TAG, "No DMX reveived.");
    }
    // Only send a packet every 100 ms
    // vTaskDelayUntil(&now, pdMS_TO_TICKS(100));
  }
}
