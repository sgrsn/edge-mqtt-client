#pragma once
#include <Arduino.h>
#include <Wire.h>

// レジスタの数
#define NUM_REGISTERS 8
#define i2c_data_type_t int
#define I2C_DATA_SIZE sizeof(i2c_data_type_t)

// 前方宣言
class I2CSlave;

// グローバル変数
I2CSlave* globalI2CSlaveInstance = nullptr;

// 静的コールバック関数の前方宣言
void receiveEventWrapper(int byteCount);
void requestEventWrapper();

class I2CSlave 
{
 private:
  // レジスタ配列（int型×8）
  volatile i2c_data_type_t registers[NUM_REGISTERS] = {0};
  // 現在選択されているレジスタのインデックス
  volatile byte currentRegister = 0;
  // 通信状態の管理
  volatile bool registerSelected = false;

 public:
  I2CSlave() {
    // グローバルインスタンスにこのインスタンスを設定
    globalI2CSlaveInstance = this;
  }

  void init(uint8_t slave_addr)
  {   
    // I2Cスレーブの初期化
    Wire.begin(slave_addr);
    // データ受信イベントのコールバック関数を登録
    Wire.onReceive(receiveEventWrapper);
    // データ要求イベントのコールバック関数を登録
    Wire.onRequest(requestEventWrapper);
  }

  void setRegister(byte regAddr, i2c_data_type_t value) 
  {
    if (regAddr < NUM_REGISTERS) 
    {
      registers[regAddr] = value;
    }
  }

  i2c_data_type_t getRegister(byte regAddr) 
  {
    if (regAddr < NUM_REGISTERS) 
    {
      return registers[regAddr];
    }
    return 0;
  }

  // マスターからデータを受信したときに呼ばれる関数
  void handleReceive(int byteCount) 
  {
    if (Wire.available()) 
    {
      // 最初のバイトは常にレジスタアドレスとして扱う
      byte regAddr = Wire.read();
      
      // 有効なレジスタアドレスかチェック
      if (regAddr < NUM_REGISTERS) 
      {
        currentRegister = regAddr;
        
        // レジスタアドレスの後にデータがある場合は書き込み操作と判断
        if (Wire.available() >= 4) 
        {
          // int型（4バイト）のデータを受信
          byte buffer[4];
          for (int i = 0; i < I2C_DATA_SIZE; i++)
          {
            if (Wire.available()) {
              buffer[i] = Wire.read();
            }
          }
          
          // バイト配列からint型に変換
          i2c_data_type_t value = 0;
          for (int i = 0; i < I2C_DATA_SIZE; i++)
          {
            value |= buffer[i] << (8 * (I2C_DATA_SIZE - i - 1));
          }
          registers[currentRegister] = value;
          
        } else {
          registerSelected = true;
        }
      }
    }
  }

  // マスターからデータ送信要求があったときに呼ばれる関数
  void handleRequest() 
  {
    if (registerSelected && currentRegister < NUM_REGISTERS) 
    {
      // 選択されたレジスタの値を送信
      i2c_data_type_t value = registers[currentRegister];
      
      // int型の値をバイト配列に変換して送信
      byte buffer[I2C_DATA_SIZE];
      for (int i = 0; i < I2C_DATA_SIZE; i++)
      {
        buffer[i] = (value >> (8 * (I2C_DATA_SIZE - i - 1))) & 0xFF;
      }
      Wire.write(buffer, I2C_DATA_SIZE);

      registerSelected = false;  // 送信完了後にフラグをリセット
    }
  }
};

// 静的コールバック関数の実装
void receiveEventWrapper(int byteCount) {
  if (globalI2CSlaveInstance) {
    globalI2CSlaveInstance->handleReceive(byteCount);
  }
}

void requestEventWrapper() {
  if (globalI2CSlaveInstance) {
    globalI2CSlaveInstance->handleRequest();
  }
}