#pragma once
#define TINY_GSM_MODEM_SARAR4
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <any>

// Serial.println()の代わり
#include "print_server.hpp"
DumpServer debug;

// 対応する型を列挙型で定義
enum class ValueType {
  Bool,
  Double,
  Int,
  String
};

// トピック名と最後の値を保持する構造体
struct TopicData {
  std::string raw_data;
  std::any value;
  ValueType type;
  std::function<std::any(const std::string&)> converter;
};

// トピック名とTopicDataのマップ
static std::unordered_map<std::string, TopicData> topics_;

class MqttClient {
private:
  HardwareSerial serial_;
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

  // 型に応じたValueTypeを返すテンプレート関数
  template<typename T>
  static ValueType getValueType() {
    if constexpr (std::is_same_v<T, bool>) {
      return ValueType::Bool;
    }
    else if constexpr (std::is_same_v<T, double>) {
      return ValueType::Double;
    }
    else if constexpr (std::is_same_v<T, int>) {
      return ValueType::Int;
    }
    else if constexpr (std::is_same_v<T, std::string>) {
      return ValueType::String;
    }
    else {
      // static_assert(always_false<T>::value, "Unsupported type");
      return ValueType::String;
    }
  }

public:
  MqttClient(const std::string& apn, const std::string& gprsUser, const std::string& gprsPass, const char* broker, uint16_t port, const char* clientId, const char* username, const char* password)
    : serial_(0), modem_(serial_), client_(modem_), mqtt_(client_), apn(apn), gprsUser(gprsUser), gprsPass(gprsPass), broker(broker), port(port), clientId(clientId), username(username), password(password) {}

  void init()
  {
    serial_.begin(115200, SERIAL_8N1, RX, TX);

    WiFi.softAP("MyESP32", "12345678");
    debug.begin();

    debug.println("Initializing modem...");

    // Serial setup
    serial_.begin(115200, SERIAL_8N1, RX, TX);
    delay(1000);

    // Modem setup
    modem_.restart();
    delay(1000);
    String modemInfo = modem_.getModemInfo();

    // Network setup
    if (!modem_.waitForNetwork()) {
      delay(10000);
      return;
    }
    if (!modem_.isNetworkConnected()) {
      delay(10000);
      return;
    }

    // GPRS connection parameters are usually set after network registration
    if (!modem_.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str())) {
      delay(10000);
      return;
    }
    if (!modem_.isGprsConnected()) { 
      delay(10000);
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
      if (!modem_.waitForNetwork(180000L, true)) {
        delay(10000);
        return;
      }

      // and make sure GPRS/EPS is still connected
      if (!modem_.isGprsConnected()) {
        if (!modem_.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str())) {
          delay(10000);
          return;
        }
      }
    }

    if (!mqtt_.connected()) {
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
    TopicData data;
    data.type = getValueType<T>();
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
    else if constexpr (std::is_same_v<T, std::string>) {
      data.converter = [](const std::string& s) { return std::any(s); };
    }
    else {
      // throw std::runtime_error("Unsupported type for topic: " + topic_name);
      debug.println("Unsupported type for topic: " + topic_name);
    }
    // mqtt_.subscribe(topic_name.c_str());
    topics_[topic_name] = data;
  }

  bool getLastValue(const std::string& topic_name, double& value) 
  {
    auto it = topics_.find(topic_name);
    
    if (it == topics_.end()) {
      // throw std::runtime_error("Topic not registered: " + topic_name);
      debug.println("Topic not registered: ", topic_name.c_str());
      return false;
    }

    value = stringToDouble(it->second.raw_data);
    return true;
  }

  bool getLastValue(const std::string& topic_name, std::string& value)
  {
    auto it = topics_.find(topic_name);
    if (it == topics_.end()) {
      // throw std::runtime_error("Topic not registered: " + topic_name);
      debug.println("Topic not registered: ", topic_name.c_str());
      return false;
    }
    value = it->second.raw_data;
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
    
    // 登録された変換関数を使用して値を変換し保存
    try {
      it->second.value = it->second.converter(payload_str);
    } catch (const std::exception& e) {
      // pass
    }
  }

  boolean mqttConnect() 
  {
    boolean status = mqtt_.connect(clientId, username, password);
    if (status == false) {
      return false;
    }
    for (auto& [topic_name, _] : topics_) {
      mqtt_.subscribe(topic_name.c_str());
    }
    return mqtt_.connected();
  }
};
