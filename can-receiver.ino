// Simple CAN receiver sketch

#include "driver/gpio.h"
#include "driver/twai.h"

#define GPIO_5V0_EN             GPIO_NUM_23
#define ESP_LED1                GPIO_NUM_22
#define ESP_LED2                GPIO_NUM_21
#define ESP_WAKE_IN1            GPIO_NUM_13
#define ESP_WAKE_IN2            GPIO_NUM_27
#define CAN_SILENT0             GPIO_NUM_4
#define CAN_SILENT1             GPIO_NUM_5
#define CAN_TERM_EN             GPIO_NUM_25
#define ESP_CAN_TX              GPIO_NUM_12
#define ESP_CAN_RX              GPIO_NUM_14

#define GPIO_LEVEL_HIGH         (1)
#define GPIO_LEVEL_LOW          (0)

void setup() {
  // setup serial
  Serial.begin(9600);
  Serial.println("CAN Receiver");

  // Enable 5V0 and LEDs
  esp_err_t err;
  gpio_config_t gpioConfig;
  gpioConfig.mode = GPIO_MODE_OUTPUT;
  gpioConfig.pin_bit_mask = (1ULL << GPIO_5V0_EN | 1ULL << ESP_LED1 | 1ULL << ESP_LED2 | 1ULL << CAN_SILENT0 | 1ULL << CAN_SILENT1 | 1ULL << CAN_TERM_EN);
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type = GPIO_INTR_DISABLE;
  err = gpio_config(&gpioConfig);
  ESP_ERROR_CHECK(err);
  bool pinState = false;
  gpio_set_level(GPIO_5V0_EN, GPIO_LEVEL_HIGH);
  gpio_set_level(ESP_LED1, pinState);
  gpio_set_level(CAN_SILENT0, 1);
  gpio_set_level(CAN_SILENT1, 0);
  gpio_set_level(CAN_TERM_EN, 1);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(ESP_CAN_TX, ESP_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
    twai_reconfigure_alerts(TWAI_ALERT_AND_LOG | TWAI_ALERT_ALL, NULL);
  } else {
    Serial.println("Failed to install driver");
    return;
  }
  // start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }
}

void loop() {
  // Read CAN message
  esp_err_t err;
  twai_message_t rxMessage;
  err = twai_receive(&rxMessage, pdMS_TO_TICKS(3000));
  if (err == ESP_OK)
  {
    Serial.println("Message received");
    Serial.print("ID is ");
    Serial.println(rxMessage.identifier);
    for (int i = 0; i < rxMessage.data_length_code; i++)
    {
      Serial.print("Data byte ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(rxMessage.data[i]);
    }
  }
  else
  {
    Serial.println("Failed to receive message.");
  }
}
