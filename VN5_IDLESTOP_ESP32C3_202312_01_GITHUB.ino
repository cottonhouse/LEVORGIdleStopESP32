// for ESP32C3 + CAN Receiver(VP230)
//
// Copyright (c) 2023 Cottonhouse
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php
//
// CAN Lib. https://github.com/collin80/esp32_can
//          https://github.com/collin80/can_common
// Rev.01 ESP32用のプログラムをESP32C3用に移植
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <esp32_can.h>
#include <EEPROM.h>
//#define DEBUG 1  // シリアルにModeを表示する場合コメントを外す(USBシリアル接続をしていないと動作しなくなる)
#define EEPROM_SIZE 64

// for WiFi
const char *ssid = "LEVORG-IDLESTOP";
const char *password = "p@ssw0rd1234"; // 強固なパスワード推奨
const IPAddress ip(192, 168, 2, 1);
const IPAddress subnet(255, 255, 255, 0);

// for EEPROM
unsigned char romstat;           // 前回電源が切れる直前に保存されていた設定データ
const unsigned int ROMADDR = 0;  // EEPROMのアドレス

// Main Loop
int STAT = 0;        // 動作ステータス 0..エンジン起動まで待つ  1..保存された設定の適用  2..変更があったら設定を保存
int pktSending = 0;  // パケット送信フェーズフラグ

// Timer
volatile unsigned int intr = 0;
void intrrup() {
  intr += 1;
}
Ticker blinker;

// EEPROM Reading
void getRomstat() {
  // Get EEPROM Data
  romstat = EEPROM.read(ROMADDR);
  #ifdef DEBUG
  Serial.print("EEPROM Read: ");
  Serial.print(romstat, HEX);
  #endif
  // 値は00(ON)か40(OFF)しか保存されないはずだが、念の為それ以外だったら0にする（保存途中に電源が切れたりして内容が破壊されることもあり得る)）
  if(romstat != 0x40 && romstat != 0) {
    romstat = 0;
  }
  romstat &= 0x40; // 上から2bit目取り出し これも要らないが念の為残しておく
  #ifdef DEBUG
  Serial.print(" -> ");
  Serial.println(romstat, HEX);
  #endif
}

// パケット送信
void PktSend(CAN_FRAME *rx_frame) {
  CAN_FRAME tx_frame;
  unsigned char chksum = 0;        // チェックサム計算用
  unsigned char sendbuf[8];        // 送信用バッファ
  static unsigned char SecNo = 0;  // シーケンシャルNo用

  tx_frame.id = 0x390;  //rx_frame.MsgID;
  tx_frame.length = rx_frame->length;
  tx_frame.rtr = 0;
  tx_frame.extended = false;
  tx_frame.data.uint8[1] = SecNo;  // シーケンシャルNo
  tx_frame.data.uint8[2] = rx_frame->data.uint8[2];
  tx_frame.data.uint8[3] = rx_frame->data.uint8[3];
  tx_frame.data.uint8[4] = rx_frame->data.uint8[4];
  tx_frame.data.uint8[5] = rx_frame->data.uint8[5];
  tx_frame.data.uint8[6] = 0x40 | rx_frame->data.uint8[6];  // SW ON
  tx_frame.data.uint8[7] = rx_frame->data.uint8[7];
  for (int i = 1; i < tx_frame.length; i++) {  // data.u8[1] から data.u8[7] を合計してチェックサムを計算
    chksum += tx_frame.data.uint8[i];
  }
  tx_frame.data.uint8[0] = chksum + 0x93;  // チェックサム
  CAN0.sendFrame(tx_frame);    // 送信

  SecNo = ++SecNo & 0x0F;  // シーケンシャルNoの加算(不要と思われるが、念の為変える)
  #ifdef DEBUG
  Serial.println("Send");
  #endif
}

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {
  }
  #endif

  //  for WiFi AP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  #ifdef DEBUG
  Serial.println(myIP);
  #endif

  // for OTA
  ArduinoOTA.
  onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    #ifdef DEBUG
    Serial.println("Start updating " + type);
    #endif
  })
  .onEnd([]() {
    #ifdef DEBUG
    Serial.println("\nEnd");
    #endif
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    #ifdef DEBUG
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  })
  .onError([](ota_error_t error) {
    #ifdef DEBUG
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
  });
  ArduinoOTA.begin();

  // CAN
  #ifdef DEBUG
  Serial.print("CAN initializing");
  #endif
  CAN0.setCANPins(GPIO_NUM_5, GPIO_NUM_4);
  #ifdef DEBUG
  CAN0.setDebuggingMode(true);
  #endif
  CAN0.begin(500000);
  CAN0.watchFor();  // 全て通すフィルタ これが無いとパケットが受信できない
  #ifdef DEBUG
  Serial.println("CAN init end");
  #endif

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Get EEPROM Data
  getRomstat();
  #ifdef DEBUG
  Serial.println("Mode:0");
  #endif

  // Timer
  blinker.attach(0.05, intrrup); // Remark for Test environment
}

void loop() {
  CAN_FRAME rx_frame;
  static unsigned char D4_174;  // D4の直前の値保存用
  static unsigned int refTimes = 0; // 設定反映の試行回数
  ArduinoOTA.handle();

  if (CAN0.read(rx_frame)) {
    if (rx_frame.rtr == 0) {
      switch (rx_frame.id) {
        case 0x174:  // アイストCU
          if ((rx_frame.data.uint8[2] & 0x08) == 0) {
            // エンジンがOFFだったら強制的にステータス0へ遷移(ACCから電源をとる前提。エンジンOFFでパケットが来なくなるようなので、処理的には不要)
            if (STAT) {      // STATが0でない時STATを0にする
              getRomstat();  // EEPROM読み込み
              STAT = 0;
              pktSending = 0;  // 送信中フラグOFF
              #ifdef DEBUG
              Serial.println("Mode:0");
              #endif
            }
            break;            // switch(id)抜ける
          } else {            // エンジンがONになった
            if (STAT == 0) {  // エンジンがONでステータスが0だったらステータス1に変遷
              STAT = 1;       // EEPROMに保存された値の復旧処理ステータスへ変遷
              refTimes = 0; // 送信試行回数クリア
              #ifdef DEBUG
              Serial.println("Mode:1");
              #endif
            }
          }
          switch (STAT) {                                     // ステータスによって違う処理を実行
            case 1:                                           // 保存された設定の適用
              if (romstat != (rx_frame.data.uint8[4] & 0x40)) {  // 保存された設定状態と現在の設定状態が異なっていたら、SW ONのパケットを送信する
                if (!pktSending) {
                  pktSending = 1;  // 送信中フラグON
                  #ifdef DEBUG
                  Serial.println("Mode:1Send");
                  Serial.print("  ROM: ");
                  Serial.println(romstat, HEX);
                  Serial.print("  D4 : ");
                  Serial.println(rx_frame.data.uint8[4], HEX);
                  #endif
                } else {
                // 念の為のルーチン（送信して設定が変更されても保存された前回値にならなかった場合（CAN解析では出てこなかった値になった場合）
                // 無限に送信してしまう事になるのでそれを防止）
                  if(refTimes > 4) { // 送信試行は5回まで
                    STAT = 2;
                    pktSending = 0;  // 送信中フラグOFF
                    D4_174 = romstat; // EEPROM保存データを格納しておくと、STAT2で現在設定がEEPROMに保存される
                    refTimes = 0; // 要らないけど気持ち悪いのでクリアしておく
                    #ifdef DEBUG
                    Serial.println("Mode:2"); 
                    #endif
                  }
                }
              } else {
                STAT = 2;                             // 保存された設定と同じだった or Sendの結果同じになったら、次のステータスへ（2=設定値の保存ステータス）
                pktSending = 0;                       // 送信中フラグOFF
                D4_174 = rx_frame.data.uint8[4] & 0x40;  // 前回受信データの保存
                #ifdef DEBUG
                Serial.println("Mode:2");
                #endif
              }
              break;
            case 2:                                          // 設定に変更があったら保存する
              if ((rx_frame.data.uint8[4] & 0x40) != D4_174) {  // 保存しておいた前回データと違う場合
                D4_174 = rx_frame.data.uint8[4] & 0x40;         // 前回データの保存
                EEPROM.write(ROMADDR, D4_174);               // EEPROMへ保存
                EEPROM.commit();                             // ESP32はCommitが必要
                #ifdef DEBUG
                Serial.print("Write EEPROM: ");
                Serial.println(D4_174, HEX);
                #endif
              }
              break;
            default:  // メモリリークとか無いように念の為
              break;
          }
          break;
        case 0x390:  // アイストSW
          // パケット送信
          if (pktSending) {                              // パケット送信中フラグがONの時
            if ((rx_frame.data.uint8[6] & 0x40) != 0x40) {  // 自分が送ったデータ含む SW ONのデータを受信した時は、パケットは送らない
              PktSend(&rx_frame);                        // パケット送信
              refTimes++; // 送信回数カウント
            }
          }
          break;
        default:  // メモリリークとか無いように念の為
          break;
      }
    }
    // エンジンOFFになった場合、モード変遷できない（174パケットが無いとエンジンOFFが判断できない）ので、タイマ利用
    intr = 0;  // タイマクリア（CANパケット受信時のみクリアされるので、受信できないときはタイマカウンタが進む)
  }
  if ((intr >= 2)) {  // 0.05-0.1秒程度受信がないとき、電源OFFとみなす
    if (STAT != 0) {
      getRomstat();
      STAT = 0;        // エンジン掛かるまで待つモード
      pktSending = 0;  // 送信中フラグOFF
      #ifdef DEBUG
      Serial.println("Mode:0");
      #endif
    }
    intr = 0;  // Clear Timer
  }
  delay(1);  // ESP32だとウォッチドッグタイマー用に必要かも
}
