// DumpServer.h
#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <queue>
#include <string>
#include "SPIFFS.h"

class DumpServer {
private:
  AsyncWebServer server;
  std::queue<String> messageQueue;
  static const size_t MAX_QUEUE_SIZE = 100;
  String currentLine;
  
  // キューにメッセージを追加する内部メソッド
  void enqueueMessage(const String& msg) {
    if(currentLine.isEmpty()) {
      currentLine = msg;
      return;
    }

    if (messageQueue.size() >= MAX_QUEUE_SIZE) {
      messageQueue.pop();
    }
    messageQueue.push(currentLine);
    currentLine = msg;
  }
  
  // 可変引数の再帰の終了条件
  void printInternal(String& result) {}
  
  // 可変引数を処理する内部メソッド
  template<typename T, typename... Args>
  void printInternal(String& result, const T& first, const Args&... args) {
    result += String(first) + " ";
    printInternal(result, args...);
  }

public:
  DumpServer(uint16_t port = 80) : server(port) {}
  
  void begin() {
    // SPIFFSを初期化
    if(!SPIFFS.begin(true)){
      return;
    }

    // Serve static files
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/index.html", "text/html");
    });

    // SSE endpoint
    server.on("/events", HTTP_GET, [this](AsyncWebServerRequest *request){
      AsyncWebServerResponse *response = request->beginResponse(
        200, "text/event-stream",
        "data: Connected\n\n"
      );
      response->addHeader("Cache-Control", "no-cache");
      response->addHeader("Connection", "keep-alive");
      request->send(response);
    });

    // Periodic message sender
    server.on("/messages", HTTP_GET, [this](AsyncWebServerRequest *request){
      String messages;
      while (!messageQueue.empty()) {
        messages += messageQueue.front() + "\n";
        messageQueue.pop();
      }
      if (!currentLine.isEmpty()) {
        messages += currentLine;  // 現在の行も送信（改行なし）
        currentLine = "";
      }
      request->send(200, "text/plain", messages);
    });

    server.begin();        
  }

  // 単一引数のprint
  template<typename T>
  void print(const T& value) {
    currentLine += String(value);
  }

  // 複数引数に対応したprint
  template<typename T, typename... Args>
  void print(const T& first, const Args&... args) {
    String result = String(first);
    printInternal(result, args...);
    currentLine += result;
  }

  // 単一引数のprintln
  template<typename T>
  void println(const T& value) {
    currentLine += String(value);
    currentLine += "\n";
  }

  // 複数引数に対応したprintln
  template<typename... Args>
  void println(const Args&... args) {
    String result;
    printInternal(result, args...);
    currentLine += result;
    currentLine += "\n";
  }

  // printf スタイルメソッド
  void printf(const char* format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    currentLine += String(buf);
  }
};