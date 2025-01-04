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

int16_t clamp_i(int16_t num, int16_t min, int16_t max){
  const float t = num < min ? min : num;
  return t > max ? max : t;
}

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

  // Constants
  const int16_t fire_change_divider_red = 3; 
  const uint8_t fire_red_min = 50;
  const uint8_t fire_red_max = 255;
  const uint8_t fire_red_start = ((uint16_t)fire_red_min + (uint16_t)fire_red_max)/2;
  const int16_t fire_change_divider_green = 10; 
  const uint8_t fire_green_min = 10;
  const uint8_t fire_green_max = 50;
  const uint8_t fire_green_start = ((uint16_t)fire_green_min + (uint16_t)fire_green_max)/2;
  const int16_t fire_change_divider_blue = 20;
  const uint8_t fire_blue_min = 0;
  const uint8_t fire_blue_max = 0;
  const uint8_t fire_blue_start = ((uint16_t)fire_blue_min + (uint16_t)fire_blue_max)/2;
  const uint32_t fire_change_interval_ms = 400;

  // State variables
  int16_t fire_red = fire_red_start;
  int16_t fire_green = fire_green_start;
  int16_t fire_blue = fire_blue_start;
  TickType_t fire_last_change = xTaskGetTickCount();

  while (1) {
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

      // Root Par 6
      // L?
      const size_t start_root_par_l = 18;
      data_out[start_root_par_l] = data_in[7]; // red
      data_out[start_root_par_l+1] = data_in[8]; // green
      data_out[start_root_par_l+2] = data_in[9]; // blue
      data_out[start_root_par_l+3] = data_in[10]; // white
      data_out[start_root_par_l+4] = 0; // amber
      data_out[start_root_par_l+5] = 0; // lime?
      // R?
      const size_t start_root_par_r = 25;
      data_out[start_root_par_r] = data_out[start_root_par_l];
      data_out[start_root_par_r+1] = data_out[start_root_par_l+1];
      data_out[start_root_par_r+2] = data_out[start_root_par_l+2];
      data_out[start_root_par_r+3] = data_out[start_root_par_l+3];
      data_out[start_root_par_r+4] = data_out[start_root_par_l+4];
      data_out[start_root_par_r+5] = data_out[start_root_par_l+5];

      // Theater Spot 60 WRGBW
      // L?
      const size_t start_theater_l = 34;
      data_out[start_theater_l] = 255; // dimmer
      data_out[start_theater_l+1] = 255; // dimmer fine
      data_out[start_theater_l+2] = 0; // strobe
      data_out[start_theater_l+3] = data_in[11]; // red
      data_out[start_theater_l+4] = data_in[12]; // green
      data_out[start_theater_l+5] = data_in[13]; // blue
      data_out[start_theater_l+6] = data_in[14]; // white
      // R?
      const size_t start_theater_r = 50;
      data_out[start_theater_r] = data_out[start_theater_l];
      data_out[start_theater_r+1] = data_out[start_theater_l+1];
      data_out[start_theater_r+2] = data_out[start_theater_l+2];
      data_out[start_theater_r+3] = data_out[start_theater_l+3];
      data_out[start_theater_r+4] = data_out[start_theater_l+4];
      data_out[start_theater_r+5] = data_out[start_theater_l+5];
      data_out[start_theater_r+6] = data_out[start_theater_l+6];

      // Flatbeam Duo 
      // Lagerfeuer
      TickType_t now = xTaskGetTickCount();
      if((now - fire_last_change) > pdMS_TO_TICKS(fire_change_interval_ms)){
        fire_last_change = now;
        int8_t random_red = esp_random();
        int8_t random_green = esp_random();
        int8_t random_blue = esp_random();
        fire_red = clamp_i(fire_red + random_red/fire_change_divider_red,fire_red_min,fire_red_max);
        fire_green = clamp_i(fire_green + random_green/fire_change_divider_green,fire_green_min,fire_green_max);
        fire_blue = clamp_i(fire_blue + random_blue/fire_change_divider_blue,fire_blue_min,fire_blue_max);
      }

      float scaling = ((float)(data_in[16]))/((float)255.F);
      data_out[100] = (uint8_t)(((float)fire_red)*scaling); // red
      data_out[101] = (uint8_t)(((float)fire_green)*scaling); // green
      data_out[102] = (uint8_t)(((float)fire_blue)*scaling); // blue

      // White
      data_out[103] = data_in[15]; // white

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
