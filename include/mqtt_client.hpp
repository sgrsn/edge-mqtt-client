#pragma once
#define TINY_GSM_MODEM_SARAR4
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <any>

#define debug Serial

// トピック名と最後の値を保持する構造体
struct TopicData {
  std::string raw_data;
  std::any value;
  std::function<std::any(const std::string&)> converter;
};

// トピック名とTopicDataのマップ
static std::unordered_map<std::string, TopicData> topics_;

class MqttClient {
private:
  // HardwareSerial serial_;
  TinyGsm        modem_;
  TinyGsmClient client_;
  PubSubClient  mqtt_;

  const std::string apn;
  const std::string gprsUser;
  const std::string gprsPass;
  const char* broker;
  const uint16_t port;
  const char* clientId;
  const char* username;
  const char* password;

  uint32_t lastReconnectAttempt = 0;

  // 文字列をboolに変換
  static bool stringToBool(const std::string& str) {
    if (str.empty()) {
      return false;
    }
    return str == "true" || str == "1";
  }
  
  // 文字列をdoubleに変換
  static double stringToDouble(const std::string& str) {
    if (str.empty()) {
      return 0.0;
    }
    return std::stod(str);
  }
  
  // 文字列をintに変換
  static int stringToInt(const std::string& str) {
    if (str.empty()) {
      return 0;
    }
    return std::stoi(str);
  }

  static uint32_t stringToUint32(const std::string& str) {
    if (str.empty()) {
      return 0;
    }
    return std::stoul(str);
  }

  static uint64_t stringToUint64(const std::string& str) {
    if (str.empty()) {
      return 0;
    }
    return std::stoull(str);
  }

  // 型のサポート確認を行うテンプレート関数
  template<typename T>
  static bool isSupportedType() {
    return std::is_same_v<T, bool> ||
           std::is_same_v<T, double> ||
           std::is_same_v<T, int> ||
            std::is_same_v<T, uint32_t> ||
            std::is_same_v<T, uint64_t> ||
           std::is_same_v<T, std::string>;
  }

  // トピックの検証と未登録チェックを行うヘルパー関数
  auto findTopic(const std::string& topic_name) {
    auto it = topics_.find(topic_name);
    if (it == topics_.end()) {
      debug.print("Topic not registered: ");
      debug.println(topic_name.c_str());
    }
    return it;
  }

public:
  MqttClient(Stream& serial, const std::string& apn, const std::string& gprsUser, const std::string& gprsPass, const char* broker, uint16_t port, const char* clientId, const char* username, const char* password)
    : modem_(serial), client_(modem_), mqtt_(client_), apn(apn), gprsUser(gprsUser), gprsPass(gprsPass), broker(broker), port(port), clientId(clientId), username(username), password(password) {}

  void init()
  {
    // serial_.begin(115200, SERIAL_8N1, RX, TX);

    // WiFi.softAP("MyESP32", "12345678");
    // debug.begin();

    debug.println("Initializing modem...");

    // Serial setup
    // serial_.begin(115200, SERIAL_8N1, RX, TX);

    // Modem setup
    modem_.restart();
    delay(100);
    String modemInfo = modem_.getModemInfo();

    debug.print("Modem: ");
    debug.println(modemInfo);

    // Set modem baud rate and UART pins if you need
    // modem_.setBaud(115200);
    // debug.println("Modem baud rate set to 115200");
    // while(1);

    // Network setup
    if (!modem_.waitForNetwork()) {
      delay(2000);
      return;
    }
    if (!modem_.isNetworkConnected()) {
      delay(2000);
      return;
    }

    // GPRS connection parameters are usually set after network registration
    if (!modem_.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str())) {
      delay(2000);
      return;
    }
    if (!modem_.isGprsConnected()) { 
      delay(2000);
      return;
    }

    // MQTT Broker setup
    mqtt_.setServer(broker, port);
    mqtt_.setCallback(mqttCallback);

    debug.println("Setup completed");
  }

  void mqttLoop()
  {
    // Make sure we're still registered on the network
    if (!modem_.isNetworkConnected()) {
      debug.println("Network disconnected");
      if (!modem_.waitForNetwork(180000L, true)) {
        delay(10000);
        return;
      }

      // and make sure GPRS/EPS is still connected
      if (!modem_.isGprsConnected()) {
        debug.println("GPRS disconnected");
        if (!modem_.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str())) {
          delay(10000);
          return;
        }
      }
    }

    if (!mqtt_.connected()) {
      debug.println("MQTT disconnected");
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) { lastReconnectAttempt = 0; }
      }
      delay(100);
      return;
    }

    mqtt_.loop();
  }

  template<typename T>
  void registerTopic(const std::string& topic_name) {
    // static_assert(isSupportedType<T>(), "Unsupported type for MQTT topic");

    if (!isSupportedType<T>()) {
      debug.print("Unsupported type for MQTT topic: ");
      debug.println(topic_name.c_str());
      return;
    }
    
    TopicData data;
    data.raw_data = "";
    data.value = std::any();
    
    // 型に応じた変換関数を設定
    if constexpr (std::is_same_v<T, bool>) {
      data.converter = [](const std::string& s) { return std::any(stringToBool(s)); };
    }
    else if constexpr (std::is_same_v<T, double>) {
      data.converter = [](const std::string& s) { return std::any(stringToDouble(s)); };
    }
    else if constexpr (std::is_same_v<T, int>) {
      data.converter = [](const std::string& s) { return std::any(stringToInt(s)); };
    }
    else if constexpr (std::is_same_v<T, uint32_t>) {
      data.converter = [](const std::string& s) { return std::any(stringToUint32(s)); };
    }
    else if constexpr (std::is_same_v<T, uint64_t>) {
      data.converter = [](const std::string& s) { return std::any(stringToUint64(s)); };
    }
    else if constexpr (std::is_same_v<T, std::string>) {
      data.converter = [](const std::string& s) { return std::any(s); };
    }
    
    topics_[topic_name] = data;
  }

  // テンプレート化したgetLastValue関数
  template<typename T>
  bool getLastValue(const std::string& topic_name, T& value) {
    auto it = findTopic(topic_name);
    if (it == topics_.end()) {
      debug.print("Topic not registered: ");
      debug.println(topic_name.c_str());
      return false;
    }
    
    if (it->second.raw_data.empty()) {
      return false;
    }
    else
    {
      it->second.value = it->second.converter(it->second.raw_data);
      value = std::any_cast<T>(it->second.value);
    }
    
    return true;
  }
    
  // MQTTコールバック関数
  static void mqttCallback(char* received_topic, byte* payload, unsigned int len) {
    std::string topic_name(received_topic);
    auto it = topics_.find(topic_name);
    if (it == topics_.end()) {
      return; // 未登録のトピックは無視
    }
    // ペイロードを文字列に変換
    std::string payload_str(reinterpret_cast<char*>(payload), len);
    it->second.raw_data = payload_str;
  }

  boolean mqttConnect() {
    boolean status = mqtt_.connect(clientId, username, password);
    if (status == false) {
      return false;
    }
    for (auto& [topic_name, _] : topics_) {
      mqtt_.subscribe(topic_name.c_str());
    }
    return mqtt_.connected();
  }

  boolean publish(const std::string& topic_name, const std::string& value) {
    return mqtt_.publish(topic_name.c_str(), value.c_str());
  }
};