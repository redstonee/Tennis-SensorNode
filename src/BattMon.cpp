#include <Arduino.h>
#include "BattMon.h"
#include "config.h"
#include "ulog.h"

namespace BattMon
{
    constexpr auto SAMPLE_RATE = 50;

    constexpr auto BUF_SIZE = 20;         // Size of the ADC buffer
    static uint16_t adcBuf[BUF_SIZE]{0};  // Buffer to store ADC values
    static uint8_t bufIndex = 0;          // Index for the ADC buffer
    static bool bufferFull = false;       // Flag to indicate if the buffer is full
    static HardwareTimer adcTimer(TIM1); // Timer for ADC sampling

    /**
     * @brief Read the ADC value and store it in the buffer
     * 
     */
    inline void readADC()
    {
        adcBuf[bufIndex++] = analogRead(VSENS_PIN); // Read the ADC value and store it in the buffer
        if (bufIndex >= BUF_SIZE)
        {
            bufferFull = true; // Set the buffer full flag
            bufIndex = 0;      // Reset the buffer index
        }
    }

    /**
     * @brief Start the ADC sampling
     * 
     */
    void begin()
    {
        analogReadResolution(ADC_RESOLUTION);
        adcTimer.setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
        adcTimer.attachInterrupt(readADC);
        adcTimer.resume();
    }

    /**
     * @brief Get the battery voltage
     *
     * @return float Battery voltage in volts, or -1 if the data is not ready
     */
    float getVoltage()
    {
        if (!bufferFull)
        {
            return -1;
        }

        uint32_t rawValue = 0;
        // Calculate the average ADC value
        for (auto &v : adcBuf)
        {
            rawValue += v;
        }
        rawValue /= BUF_SIZE;

        auto voltage = (rawValue * 3.3) * SAMPLE_FACTOR / (1 << ADC_RESOLUTION);
        return voltage;
    }

    /**
     * @brief Stop the ADC sampling
     * 
     */
    void stop()
    {
        adcTimer.pause();
    }

} // namespace BattMon
