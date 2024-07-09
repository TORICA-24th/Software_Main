 //Analog UARTの宣言
#define SerialAir Serial1
#define SerialUnder Serial2

//PIO UARTの宣言
SerialPIO SerialWireless(2, 3);
SerialPIO SerialSD_ICS(10, 11);

//GPS初期宣言
#include <Geometry.h>
using namespace Geometry;
using namespace BLA;
char TWE_BUF[256];

//SD送信用バッファ
char UART_SD[512];

//TORICA_UARTインスタンス化
#include <TORICA_UART.h>
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
TORICA_UART SD_ICS_UART(&SerialSD_ICS);

//動作確認用LED
const int L_Air = 18;
const int L_Under = 19;
const int L_ICS = 20;

//スピーカ出力端子設定
const int O_SPK = 8;

//ICS初期化
#include <TORICA_ICS.h>
TORICA_ICS ics(&SerialSD_ICS);

//ジャイロ処理用ライブラリ初期化
#include <TinyGPSPlus.h>
TinyGPSPlus gps;

//I2Cパッケージ読み込み
#include <Wire.h>

//BNO055インスタンス化
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);

//DPS310初期化
#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

enum {
  PLATFORM,
  TAKEOFF,
  HIGH_LEVEL,
  MID_LEVEL,
  LOW_LEVEL
} flight_phase = PLATFORM;

enum {
  FAST,
  NORMAL,
  SLOW
} speed_level = NORMAL;
// filtered:移動平均
// lake:対地高度, 無印:気圧基準海抜高度
// dps:気圧高度
// urm:超音波高度

const float const_platform_m = 10.6;

#include "TORICA_MoveAve.h"
// 対気速度
TORICA_MoveAve<5> filtered_airspeed_ms(10.2);

// 現在の気圧高度(気圧基準)
TORICA_MoveAve<5> filtered_main_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_under_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_air_dps_altitude_m(0);
// プラホの高度(気圧基準)
TORICA_MoveAve<50> main_dps_altitude_platform_m(0);
TORICA_MoveAve<50> under_dps_altitude_platform_m(0);
TORICA_MoveAve<50> air_dps_altitude_platform_m(0);

// 気圧センサを用いた信頼できる対地高度
// 3つの気圧高度にそれぞれ移動平均をとってプラホを10mとし，中央値をとった値
#include "QuickStats.h"
float dps_altitude_lake_array_m[3];
QuickStats dps_altitude_lake_m;

// 超音波高度(対地高度)
TORICA_MoveAve<3> filtered_under_urm_altitude_m(0.6);

#include "TORICA_MoveMedian.h"
// 気圧での対地高度と超音波での対地高度の差
// 100Hz(calculate)*4s = 400
TORICA_MoveMedian<400> altitude_dps_urm_offset_m(0);


// 気圧と超音波から推定した対地高度
float estimated_altitude_lake_m = const_platform_m;

//エアデータと機体下電装部の生存確認
bool air_is_alive = false;
bool under_is_alive = false;

// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;
volatile float data_main_bno_roll = 0;
volatile float data_main_bno_pitch = 0;
volatile float data_main_bno_yaw = 0;

volatile float data_main_dps_pressure_hPa = 0;
volatile float data_main_dps_temperature_deg = 0;
volatile float data_main_dps_altitude_m = 0;

volatile float data_under_dps_pressure_hPa = 0;
volatile float data_under_dps_temperature_deg = 0;
volatile float data_under_dps_altitude_m = 0;
volatile float data_under_urm_altitude_m = 0;

volatile float data_air_dps_pressure_hPa = 0;
volatile float data_air_dps_temperature_deg = 0;
volatile float data_air_dps_altitude_m = 0;
volatile float data_air_sdp_differentialPressure_Pa = 0;
volatile float data_air_sdp_airspeed_ms = 0;

volatile int data_ics_angle = 0;

volatile uint8_t data_main_gps_hour = 0;
volatile uint8_t data_main_gps_minute = 0;
volatile uint8_t data_main_gps_second = 0;
volatile uint8_t data_main_gps_centisecond = 0;
volatile double data_main_gps_latitude_deg = 0;
volatile double data_main_gps_longitude_deg = 0;
volatile double data_main_gps_altitude_m = 0;

// ----------------------------

struct repeating_timer st_timer;
bool timer_flag = false;

bool speaker(struct repeating_timer *t) {
  static int sound_freq = 440;
  static int last_flight_phase = -1;
  static int spk_flag = 0;
  static uint32_t speaker_last_change_time = millis();
  static uint32_t sound_duration = 100; // 音が出ている時間

  switch (speed_level) {
    case SLOW:
      sound_freq = 440;
      break;
    case NORMAL:
      sound_freq = 880;
      break;
    case FAST:
      sound_freq = 1320;
      break;
    default:
      break;
  }

  int interval;
  switch (flight_phase) {
    case PLATFORM:
      interval = 1000;
      break;  
    case HIGH_LEVEL:
      interval = 500;
      break;
    case MID_LEVEL:    
      interval = 250;
      break;
    case LOW_LEVEL:
      interval = 125;
      break;
    default:
      interval = 0;
      break;
  }

  uint32_t current_time = millis();
  uint32_t off_duration = interval - sound_duration;

  if (flight_phase != PLATFORM) {
    if (spk_flag == 0 && (current_time - speaker_last_change_time) > off_duration) {
      tone(O_SPK, sound_freq);
      speaker_last_change_time = current_time;
      spk_flag = 1;
    } else if (spk_flag == 1 && (current_time - speaker_last_change_time) > sound_duration) {
      noTone(O_SPK);
      speaker_last_change_time = current_time;
      spk_flag = 0;
    }
  }

  last_flight_phase = flight_phase;

  return true;
}

void setup() {
  // LED初期化
  pinMode(L_ICS, OUTPUT);
  pinMode(L_Under, OUTPUT);
  pinMode(L_Air, OUTPUT);

  // USBケーブルを差した時の起動猶予時間
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(400);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(100);
  }

  delay(2000);

  //UARTピン設定
  SerialUnder.setTX(4);
  SerialUnder.setRX(5);
  SerialAir.setTX(0);
  SerialAir.setRX(1);

  // UART初期化
  //SerialWireless.setFIFOSize(1024);
  SerialAir.setFIFOSize(1024);
  SerialUnder.setFIFOSize(1024);
  //SerialSD_ICS.setFIFOSize(1024);
    
  SerialWireless.begin(9600);  //GPS+TWELITE
  SerialSD_ICS.begin(115200);  //SD+ICS
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  Serial.begin(115200);
  SerialWireless.print("loading...\n\n");

  //I2C wire1ピン設定
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  // DPS310初期化
  Wire1.setClock(400000);

  if (!dps.begin_I2C(0x77, &Wire1)) {
    Serial.println("Failed to find DPS");
    while (1) {
      SerialWireless.println("Failed to find DPS");
      digitalWrite(L_Air, HIGH);
      delay(100);
      digitalWrite(L_Air, LOW);
      delay(100);
    }
  }
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
  Serial.println("DPS OK!");

  // BNO055初期化
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      digitalWrite(L_Under, HIGH);
      delay(100);
      digitalWrite(L_Under, LOW);
      delay(100);
    }
  }

  add_repeating_timer_us(1000000, speaker, NULL, &st_timer);
  
  // センサー・各基板の起動を待機
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(100);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(400);
  }
}

void loop() {
  uint32_t ISR_now_time = millis();
  static uint32_t ISR_last_time = 0;
  if (ISR_now_time - ISR_last_time >= 10) {
    ISR_last_time = millis();
    func_100Hz();
  }

  // テレメトリダウンリンク(タイミングも関数内調整)
  TWE_downlink();
  //スピーカー
  //speaker();


}


void func_100Hz() {
  //uint32_t time_us = micros();

  // GPS・操舵角・機体下電装部・エアデータ電装部読み取り
  polling_UART();

  // フライトフェーズ判断(メインの加速度も測定)
  determine_flight_phase();

  // 気圧センサ×3・超音波センサから高度推定
  calculate_altitude();

  // SDに記録(メインの気圧高度も測定)
  send_SD();

  // if (micros() - time_us > 9900) {  //MAX10000=100Hz
  //   Serial.print("ISR100Hz_overrun!!!");
  // }
  // Serial.print("ISR_us:");
  // Serial.println(micros() - time_us);
}


void polling_UART() {
  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(L_ICS, !digitalRead(L_ICS));
  }

  //UnderSide
  static unsigned long int last_under_time_ms = 0;
  int readnum = Under_UART.readUART();
  int under_data_num = 4;
  if (readnum == under_data_num) {
    last_under_time_ms = millis();
    digitalWrite(L_Under, !digitalRead(L_Under));
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    filtered_under_dps_altitude_m.add(data_under_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      under_dps_altitude_platform_m.add(data_under_dps_altitude_m);
    }
    filtered_under_urm_altitude_m.add(data_under_urm_altitude_m);
  }
  if (millis() - last_under_time_ms > 1000) {
    // 超音波高度のみ冗長系がないため，データが来なければ8mとして高度推定に渡す．
    // 測定範囲外のときは10mになり，9m以上でテイクオフ判断をするため故障時は8m
    // filtered_under_urm_altitude_m.add(8.0);
    // ToDo 明示的にis_aliveを作るべき．値の処理によって7変わる．
    under_is_alive = false;
  } else {
    under_is_alive = true;
  }

  //AirData
  static unsigned long int last_air_time_ms = 0;
  readnum = Air_UART.readUART();
  int air_data_num = 5;
  if (readnum == air_data_num) {
    last_air_time_ms = millis();
    digitalWrite(L_Air, !digitalRead(L_Air));
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_ms = Air_UART.UART_data[4];
    filtered_airspeed_ms.add(data_air_sdp_airspeed_ms);
    filtered_air_dps_altitude_m.add(data_air_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      air_dps_altitude_platform_m.add(data_air_dps_altitude_m);
    }
  }
  if (millis() - last_air_time_ms > 1000) {
    air_is_alive = false;
  } else {
    air_is_alive = true;
  }

  //GPS
  while (SerialWireless.available() > 0) {
    if (gps.encode(SerialWireless.read())) {
      data_main_gps_hour = gps.time.hour();
      data_main_gps_minute = gps.time.minute();
      data_main_gps_second = gps.time.second();
      data_main_gps_centisecond = gps.time.centisecond();
      data_main_gps_latitude_deg = gps.location.lat();
      data_main_gps_longitude_deg = gps.location.lng();
      data_main_gps_altitude_m = gps.altitude.meters();
    }
  }
}


void determine_flight_phase() {
  //発進判定のため，IMU測定はここで行う
  read_main_bno();

  static unsigned long int takeoff_time_ms = 0;
  switch (flight_phase) {
    case PLATFORM:
      {
        static int over_urm_range_count = 0;
        if (filtered_under_urm_altitude_m.get() > 9.0) {
          over_urm_range_count++;
        } else {
          over_urm_range_count = 0;
        }
        bool over_urm_range = false;
        // 超音波が測定不能な状態が2秒以上続いたとき
        if (over_urm_range_count >= 200) {
          over_urm_range = true;
        }
        // 気圧センサにより下降したと判断したとき
        bool descending = estimated_altitude_lake_m < 10.2;
        if ((over_urm_range || descending) && millis() > 15000) {
          flight_phase = TAKEOFF;
          takeoff_time_ms = millis();
        }
        if (over_urm_range && millis() > 15000) {
          SerialWireless.print("\n\nover_urm_range\n\n");
        }
        if (descending && millis() > 15000) {
          SerialWireless.print("\n\ndescending\n\n");
        }
      }
      break;
    case TAKEOFF:
      // (ダイブするか知らんけど)ダイブ後に水平飛行に移ったとき(超音波の測定値が信頼できる状態のとき)
      // TAKEOFFで3秒待機することでHIGH_LEVELからMID_LEVELに瞬時に移行することを防ぐ
      if (millis() - takeoff_time_ms > 3000) {
        flight_phase = HIGH_LEVEL;
      }
      break;
    case HIGH_LEVEL:
      // 超音波が測定できるようになったとき
      /*if (filtered_under_urm_altitude_m.get() < 5.0 && estimated_altitude_lake_m < 5.0) {
        flight_phase = MID_LEVEL;
      }*/
      if (filtered_under_urm_altitude_m.get() < 5.0) {
        flight_phase = MID_LEVEL;
      }
      break;
    case MID_LEVEL:
      // 高度が1m以下になったとき(高度を高頻度で読み上げる)
      if (estimated_altitude_lake_m < 1.0) {
        flight_phase = LOW_LEVEL;
      }
      break;
    case LOW_LEVEL:
      break;
    default:
      break;
  }
  //速度レベル判断
  if (data_air_sdp_airspeed_ms > 1.5) {
    speed_level = FAST;
  }
  else if (data_air_sdp_airspeed_ms > 1.0) {
    speed_level = NORMAL;
  }
  else {
    speed_level = SLOW;
  }
}


void calculate_altitude() {
  // 100Hzで関数呼び出し
  dps_altitude_lake_array_m[0] = filtered_main_dps_altitude_m.get() - main_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[1] = filtered_under_dps_altitude_m.get() - under_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[2] = filtered_air_dps_altitude_m.get() - air_dps_altitude_platform_m.get() + const_platform_m;
  estimated_altitude_lake_m = dps_altitude_lake_m.median(dps_altitude_lake_array_m, 3);

  // 関数は100Hzで呼び出される
  // 中央値が出始めるのに2秒，そこから3秒間で気圧から超音波に情報源を切り替え
  static int transition_count = 0;
  if (flight_phase == MID_LEVEL || flight_phase == LOW_LEVEL) {
    // 気圧センサが本来より低い値ならオフセットは正
    altitude_dps_urm_offset_m.add(filtered_under_urm_altitude_m.get() - estimated_altitude_lake_m);

    if (transition_count < 500) {
      transition_count++;
    }
    float ratio = 1;
    if (transition_count < 500) {
      ratio = 0;
    }
    if (transition_count > 200) {
      ratio = (float)(transition_count - 200) / 300.0;
    }

    // 気圧センサが本来より低い値なら正のオフセットを足す
    estimated_altitude_lake_m += altitude_dps_urm_offset_m.get() * ratio;
  }
}


void send_SD() {
  uint32_t time_ms = millis();
  static int loop_count = 0;
  if (loop_count == 0) {
    sprintf(UART_SD, "%d, %.2f,%.2f,%.2f,", time_ms,
            data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
  } else if (loop_count == 1) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,",
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz,
            data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
  } else if (loop_count == 2) {

    // SDに書き込む直前で測定
    read_main_dps();

    sprintf(UART_SD, "%.2f,%.2f,%d, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,",
            estimated_altitude_lake_m, altitude_dps_urm_offset_m.get(), flight_phase,
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m,
            data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m);

  } else if (loop_count == 3) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f, %d,",
            data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms,
            data_ics_angle);
  } else {
    sprintf(UART_SD, "%u,%u,%u.%u,%10.7lf,%10.7lf,%5.2lf\n",
            data_main_gps_hour, data_main_gps_minute, data_main_gps_second, data_main_gps_centisecond,
            data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m);
    loop_count = -1;
  }
  loop_count++;
  //バッファをクリアしてから新しいデータを書き込み
  SerialSD_ICS.flush();
  SerialAir.flush();
  SerialUnder.flush();
  SerialSD_ICS.print(UART_SD);
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);
}


/*int speaker() {
  static int sound_freq = 440;
  static int last_speed_level = -1;
  static int spk_flag = 0;
  static uint32_t speaker_last_change_time = millis();
  static uint32_t sound_duration = 30; // 音が出ている時間

  switch (flight_phase) {
    case PLATFORM:
      sound_freq = 0; // 停止
      break;
    case HIGH_LEVEL:
      sound_freq = 220;
      break;
    case MID_LEVEL:    
      sound_freq = 440;
      break;
    case LOW_LEVEL:
      sound_freq = 1760;
      break;
    default:
      return 0;
  }

  int interval;
  switch (speed_level) {
    case SLOW:
      interval = 900;
      break;
    case NORMAL:
      interval = 600;
      break;
    case FAST:
      interval = 300;
      break;
    default:
      interval = 0;
      break;
  }

  uint32_t current_time = millis();
  uint32_t off_duration = interval - sound_duration;

  if (flight_phase != PLATFORM) {
    if (spk_flag == 0 && (current_time - speaker_last_change_time) > off_duration) {
      tone(O_SPK, sound_freq);
      speaker_last_change_time = current_time;
      spk_flag = 1;
    } else if (spk_flag == 1 && (current_time - speaker_last_change_time) > sound_duration) {
      noTone(O_SPK);
      speaker_last_change_time = current_time;
      spk_flag = 0;
    }
  }

  last_speed_level = speed_level;
  return 0;
}*/

void read_main_bno() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  //imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);    magnet.x()
  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         gyro.x()
  //imu::Vector<3> ground_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); ground_acc.x()
  data_main_bno_accx_mss = accel.x();
  data_main_bno_accy_mss = accel.y();
  data_main_bno_accz_mss = accel.z();
  data_main_bno_qw = quat.w();
  data_main_bno_qx = quat.x();
  data_main_bno_qy = quat.y();
  data_main_bno_qz = quat.z();
  Quaternion qua(data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  EulerAngles euler(qua.to_rotation_matrix());
  data_main_bno_roll = -(euler.first() * 180 / 3.1415);
  data_main_bno_pitch = euler.second() * 180 / 3.1415;
  data_main_bno_yaw = euler.third() * 180 / 3.1415;
}

void read_main_dps() {
  if (!(dps.temperatureAvailable() && dps.pressureAvailable())) {
    return;
  }
  dps.getEvents(&temp_event, &pressure_event);
  data_main_dps_pressure_hPa = pressure_event.pressure;
  data_main_dps_temperature_deg = temp_event.temperature;
  data_main_dps_altitude_m = (powf(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
  filtered_main_dps_altitude_m.add(data_main_dps_altitude_m);
  if (flight_phase == PLATFORM) {
    main_dps_altitude_platform_m.add(data_main_dps_altitude_m);
  }
}


void TWE_downlink() {
  static uint8_t TWE_downlink_type = 0;
  static uint32_t TWE_last_send_time = millis() - 1000;
  if (TWE_downlink_type == 0 && millis() - TWE_last_send_time >= 2000) {
    SerialWireless.print("\n\n\n");
    SerialWireless.print("MAIN\n");
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n %+06.2f        %+06.2f  %+06.2f\n", data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
    SerialWireless.print(TWE_BUF);
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 1 && millis() - TWE_last_send_time >= 400) {
    sprintf(TWE_BUF, "pressure        temp    alt\n%+06.2f        %+06.2f  %+06.2f\n", data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 2 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("UNDER : ");
    if (under_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "sonic_alt\n%+06.2f\n", data_under_urm_altitude_m);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "%+06.2f\n", filtered_under_urm_altitude_m.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 3 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("AIR : ");
    if (air_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+09.3f       %+06.2f\n", data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "                %+06.2f\n", filtered_airspeed_ms.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 4 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("estimated_altitude_lake_m\n");
    sprintf(TWE_BUF, "%+08.3f\n", estimated_altitude_lake_m);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("Flight Phase\n");
    sprintf(TWE_BUF, "%d\n",flight_phase);
    SerialWireless.print(TWE_BUF);

   SerialWireless.print("Speed Level\n");
    sprintf(TWE_BUF, "%d\n",speed_level);
    SerialWireless.print(TWE_BUF);


    sprintf(TWE_BUF, "latitude:%10.7lf longitude:%10.7lf", data_main_gps_latitude_deg, data_main_gps_longitude_deg);
    SerialWireless.println(TWE_BUF);

    //Reset downlink type
    TWE_downlink_type = 0;
    TWE_last_send_time = millis();
  }

  /*static uint32_t TWE_last_send_time = millis();
    if(millis() - TWE_last_send_time > 1000){
      sprintf(TWE_BUF,"latitude:%10.7lf longitude:%10.7lf", data_main_gps_latitude_deg,data_main_gps_longitude_deg);
      SerialWireless.println(TWE_BUF);
      TWE_last_send_time = millis();
    }*/
}
