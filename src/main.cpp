#include <Arduino.h>

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))
#define delayMicroseconds(x) vTaskDelay(pdMS_TO_TICKS(x))

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "mqtt_client.hpp"
#include "secret.h"

TaskHandle_t xMqttLoopTaskHandle = NULL;

// 文字列バッファサイズ
constexpr size_t SERIAL1_BUFFER_SIZE = 64;
const uint8_t SARA_R410_PWR_ON = 2;

static SemaphoreHandle_t mqttMutex;

MqttClient mqtt(Serial1, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

// シリアル受信タスク
void SerialRxTask(void* pvParameters) {
  Serial.println("SerialRx task starting...");
  
  Serial.println("Serial1 initialized");

  for (;;) {
    if (Serial1.available() > 0) {
      char c = Serial1.read();
      if (c > 0) {  // Skip 0x00 bytes
        if (xQueueSend(serialRxQueue, &c, portMAX_DELAY) != pdPASS) {
          Serial.println("Failed to send to queue");
        }
      }
    }
    vTaskDelay(pdUS_TO_TICKS(1000));  // 短い遅延を追加
  }
}

void MqtttLoopTask(void* pvParameters) {

  // ModemTestTaskから初期化完了の通知を受け取る(データは無視)
  xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
  Serial.println("MqttLoop task starting...");

  for (;;) {
    // セマフォ
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(10)) == pdTRUE) 
    {
      mqtt.mqttLoop();
      xSemaphoreGive(mqttMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// モデムテストタスク
void ModemTestTask(void* pvParameters) 
{
  mqtt.registerTopic<uint64_t>("robot/heartbeat");
  uint64_t this_time = 0;

  vTaskDelay(pdMS_TO_TICKS(1000));
  mqtt.init();
  vTaskDelay(pdMS_TO_TICKS(1000));

  // xMqttLoopTaskHandleに通知
  xTaskNotify(xMqttLoopTaskHandle, 1, eNoAction);


  while(true)
  {
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) 
    {
      mqtt.publish("robot/heartbeat", std::to_string(millis()));
      mqtt.getLastValue<uint64_t>("robot/heartbeat", this_time);
      xSemaphoreGive(mqttMutex);
    }

    Serial.print("millis: ");
    Serial.print(millis());
    Serial.print(", receive: ");
    Serial.println(this_time);

    mqtt.mqttLoop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(SARA_R410_PWR_ON, arduino::INPUT);
  Serial.println("Setup starting...");

  // キューの作成
  serialRxQueue = xQueueCreate(SERIAL1_BUFFER_SIZE, sizeof(char));  
  // セマフォの作成
  serialMutex = xSemaphoreCreateMutex();
  mqttMutex = xSemaphoreCreateMutex();
  // タスクの作成と開始
  BaseType_t rxTask = xTaskCreate(SerialRxTask, "SerialRx", 1024, NULL, 2, NULL);
  BaseType_t mqttLoopTask = xTaskCreate(MqtttLoopTask, "MqttLoop", 1024, NULL, 1, &xMqttLoopTaskHandle);
  BaseType_t modemTask = xTaskCreate(ModemTestTask, "ModemTest", 1024, NULL, 3, NULL);

  if (modemTask != pdPASS || rxTask != pdPASS || mqttLoopTask != pdPASS || serialRxQueue == NULL || serialMutex == NULL || mqttMutex == NULL) {
    Serial.println("Error: Failed to create tasks");
    return;
  }

  Serial.println("Starting FreeRTOS scheduler...");
  vTaskStartScheduler();
  Serial.println("Error: Scheduler failed to start");
}

void loop() 
{
  vTaskDelay(portMAX_DELAY);
  Serial.println("Loop should not be here");
}