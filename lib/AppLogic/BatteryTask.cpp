#include "BatteryTask.h"
#include "BatteryData.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <cmath>

namespace
{

  // --- INA226 direct access via M5.In_I2C ---
  // Bypasses M5Unified Power API for accurate readings.
  // See TODO.md for calibration derivation.

  constexpr uint8_t INA226_ADDR = 0x41;
  constexpr uint8_t REG_BUS_VOLTAGE = 0x02;
  constexpr uint8_t REG_CURRENT = 0x04;
  constexpr uint8_t REG_CALIBRATION = 0x05;

  // Calibration: shunt=0.005 Ohm, max_expected_current=8.192 A
  // currentLSB = 0.0003 A, cal = 0.00512 / (0.0003 * 0.005) = 3413
  constexpr uint16_t CAL_VALUE = 3413;
  constexpr float CURRENT_LSB_MA = 0.3f;     // 0.0003 A = 0.3 mA per LSB
  constexpr float BUS_VOLTAGE_LSB_MV = 1.25f; // 0.00125 V = 1.25 mV per LSB

  // --- 2S LiPo voltage-to-percentage lookup (Tab5 = 2-cell series) ---
  struct VoltagePoint
  {
    int mv;
    int percent;
  };

  constexpr VoltagePoint VOLTAGE_TABLE[] = {
      {8400, 100},  // 4.20V × 2
      {8120, 75},   // 4.06V × 2
      {7720, 50},   // 3.86V × 2
      {7400, 25},   // 3.70V × 2
      {7000, 0},    // 3.50V × 2
  };
  constexpr int VOLTAGE_TABLE_SIZE =
      sizeof(VOLTAGE_TABLE) / sizeof(VOLTAGE_TABLE[0]);

  int voltageToPercent(int mv)
  {
    if (mv >= VOLTAGE_TABLE[0].mv)
      return 100;
    if (mv <= VOLTAGE_TABLE[VOLTAGE_TABLE_SIZE - 1].mv)
      return 0;

    for (int i = 0; i < VOLTAGE_TABLE_SIZE - 1; i++)
    {
      if (mv >= VOLTAGE_TABLE[i + 1].mv)
      {
        int dv = VOLTAGE_TABLE[i].mv - VOLTAGE_TABLE[i + 1].mv;
        int dp = VOLTAGE_TABLE[i].percent - VOLTAGE_TABLE[i + 1].percent;
        return VOLTAGE_TABLE[i + 1].percent +
               (mv - VOLTAGE_TABLE[i + 1].mv) * dp / dv;
      }
    }
    return 0;
  }

  // --- I2C helpers ---

  bool writeINA226Reg(uint8_t reg, uint16_t value)
  {
    uint8_t buf[2] = {static_cast<uint8_t>(value >> 8),
                      static_cast<uint8_t>(value & 0xFF)};
    return M5.In_I2C.writeRegister(INA226_ADDR, reg, buf, 2, 400000);
  }

  bool readINA226Reg(uint8_t reg, uint16_t &out)
  {
    uint8_t buf[2] = {0, 0};
    if (!M5.In_I2C.readRegister(INA226_ADDR, reg, buf, 2, 400000))
    {
      return false;
    }
    out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return true;
  }

  // --- Task ---

  void batteryTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    BatteryData &data = ctx->batteryData();

    // Wait for M5.Power to stabilize after boot
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Override INA226 calibration register with correct value
    // (M5Unified sets it for max_expected_current=2.0, we need 8.192)
    if (writeINA226Reg(REG_CALIBRATION, CAL_VALUE))
    {
      Serial.printf("BatteryTask: INA226 calibration set to %d\n", CAL_VALUE);
    }
    else
    {
      Serial.println("BatteryTask: INA226 calibration write FAILED");
    }

    Serial.println("BatteryTask: started");

    while (true)
    {
      uint16_t rawVoltage = 0;
      uint16_t rawCurrent = 0;

      bool voltageOk = readINA226Reg(REG_BUS_VOLTAGE, rawVoltage);
      bool currentOk = readINA226Reg(REG_CURRENT, rawCurrent);

      if (voltageOk && currentOk)
      {
        int busVoltageMv = static_cast<int>(rawVoltage * BUS_VOLTAGE_LSB_MV);
        int currentMa = static_cast<int>(
            static_cast<int16_t>(rawCurrent) * CURRENT_LSB_MA);

        int percent = voltageToPercent(busVoltageMv);

        // Charging detection: negative current = charging on Tab5
        // (INA226 shunt orientation: positive = discharging)
        bool charging = (currentMa < 0);

        data.update(percent, charging, currentMa, busVoltageMv);
      }
      else
      {
        Serial.println("BatteryTask: INA226 read failed");
      }

      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }

} // namespace

void BatteryTask::start(PipelineContext &ctx, UBaseType_t priority,
                        BaseType_t core)
{
  xTaskCreatePinnedToCore(batteryTask, "Battery", 4096, &ctx, priority,
                          nullptr, core);
}
