#include <Arduino.h>
#include <SparkFun_LIS2DH12.h>
#include <cmath>
#include <ulog.h>

#include "LoraModule.hpp"
#include "BattMon.h"
#include "config.h"

static SPARKFUN_LIS2DH12 accel;
static TwoWire accelWire(I2C2_SDA_PIN, I2C2_SCL_PIN);

static LoraModule lora(Serial1, UART1_TX_PIN, UART1_RX_PIN, LORA_M0_PIN, LORA_M1_PIN);
static int16_t sensorID;

void setup()
{
  // put your setup code here, to run once:
  Serial2.setTx(UART2_TX_PIN);
  Serial2.setRx(UART2_RX_PIN);
  Serial2.begin(115200);

  ulog_init();
  ulog_subscribe([](ulog_level_t severity, char *msg)
                 { Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg); }, ULOG_DEBUG_LEVEL);

  BattMon::begin();

  if (!lora.begin())
  {
    ULOG_ERROR("Lora module is fucked up");
    while (1)
      ;
  }
  ULOG_INFO("Lora module initialized");
  sensorID = lora.getAddr() - LORA_SENSOR_ADDRESS_BASE;

  accelWire.begin();
  if (!accel.begin(ACCEL_DEFAULT_ADR, accelWire))
  {
    ULOG_ERROR("Accelerometer is fucked up");
    while (1)
      ;
  }

  accel.setDataRate(LIS2DH12_ODR_100Hz);
  accel.setScale(LIS2DH12_8g);
  accel.setMode(LIS2DH12_NM_10bit);

  // Enable HPF to remove DC offset
  uint8_t a = 0x08;
  accel.platform_write(&accel, 0x21, &a, 1);

  ULOG_INFO("Accelerometer initialized");
}

inline uint8_t calculateChecksum(uint8_t *data, size_t size)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < size; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

void loop()
{
  // put your main code here, to run repeatedly:
  auto batteryVoltage = BattMon::getVoltage();

  // ULOG_DEBUG("Battery voltage: "); //%f is unavailable with Arduino
  // Serial2.print(batteryVoltage);
  // Serial2.println(" V");
  if (batteryVoltage > 0 && batteryVoltage < 3.4f)
  {
    BattMon::stop();
    ULOG_ERROR("Battery voltage is too low");
    lora.setMode(LoraModule::Mode::Sleep);
    HAL_PWR_EnterSTANDBYMode();
  }

  constexpr auto ACC_BUF_COUNT = 10;
  static uint32_t accBuf[ACC_BUF_COUNT] = {0};
  static uint8_t accBufIndex = 0;

  auto accX = accel.getRawX();
  auto accY = accel.getRawY();
  auto accZ = accel.getRawZ();

  accX /= 10;
  accY /= 10;
  accZ /= 10;
  auto accSquared = static_cast<uint32_t>(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

  accBuf[accBufIndex++] = accSquared;
  if (accBufIndex >= ACC_BUF_COUNT)
  {
    accBufIndex = 0;
  }

  uint32_t accAvr = 0;
  for (auto &acc : accBuf)
  {
    accAvr += acc;
  }

  // Wait for a while to avoid sending too many packets
  static auto lastSentDataTime = 0;
  static auto lastSentHeartBeatTime = 0;
  if (millis() - lastSentDataTime > LORA_DATA_MIN_INTERVAL)
    if (accAvr > ACC2_THRESHOLD)
    {
      ULOG_INFO("Collision detected: %d", accAvr);

      struct __attribute__((packed))
      {
        uint8_t header = 0x69;
        uint8_t ID = sensorID;
        uint32_t acc2; // Acceleration squared
        uint8_t checksum;
      } collisionData{
          .acc2 = accSquared,
      };

      collisionData.checksum = calculateChecksum(reinterpret_cast<uint8_t *>(&collisionData),
                                                 sizeof(collisionData) - 1);

      lora.sendP2P(LORA_DONGLE_ADDRESS, LORA_CHANNEL,
                   reinterpret_cast<uint8_t *>(&collisionData), sizeof(collisionData));

      lastSentDataTime = millis();
    }

  if (millis() - lastSentHeartBeatTime > LORA_HEARTBEAT_INTERVAL &&
      millis() - lastSentDataTime > LORA_DATA_MIN_INTERVAL) // No need to send heartbeat if data is sent recently
  {
    lora.sendP2P(LORA_DONGLE_ADDRESS, LORA_CHANNEL, LORA_HEARTBEAT_DATA);
    lastSentHeartBeatTime = millis();
  }

  delay(50);
}