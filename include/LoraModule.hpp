#pragma once
#include <Arduino.h>
#include <vector>
#include <EEPROM.h>
#include <ulog.h>
#include "config.h"

extern "C"
{
    extern const uint8_t sensorID; // Sensor ID
    extern const uint8_t netID;    // Network ID of the nodes
}

class LoraModule
{
private:
    HardwareSerial &loraSerial;
    int pinM0;
    int pinM1;

    enum class TxMode : uint8_t
    {
        Broadcast = 0,
        P2P,
    };

    static constexpr auto EEPROM_CFG_ADDRESS = 0x00;

    struct __attribute__((packed)) ModuleRegs
    {
        uint8_t addH;  // High byte of module address
        uint8_t addL;  // Low byte of module address
        uint8_t netId; // Network ID
        uint8_t cfg0;
        uint8_t cfg1;
        uint8_t channel; // Channel 850.125MHz + channel * 1MHz
        uint8_t cfg3;
        uint8_t cryptH;
        uint8_t cryptL;
    } moduleRegs;

    static const ModuleRegs defaultRegs;

    inline uint8_t indexOfRegs(uint8_t &reg)
    {
        return reinterpret_cast<uint8_t *>(&reg) - reinterpret_cast<uint8_t *>(&moduleRegs);
    }

    /*
        CFG0[7:5]: baud rate
        000: 1200bps
        001: 2400bps
        010: 4800bps
        011: 9600bps
        100: 19200bps
        101: 38400bps
        110: 57600bps
        111: 115200bps

        CFG0[4:3]: Parity
        00: 8N1
        01: 8O1
        10: 8E1
        11: 8N1

        CFG0[2:0]: Air data rate
        000: 2.4kbps
        001: 2.4kbps
        010: 2.4kbps
        011: 4.8kbps
        100: 9.6kbps
        101: 19.2kbps
        110: 38.4kbps
        111: 62.5kbps
    */

    /*
        CFG3[6]: TX mode
        0: Broadcast mode
        1: P2P mode
    */

    /**
     * @brief Read the registers from the module and store them in the moduleRegs struct
     *
     * @param data Data buffer to read to from the module
     * @param offset Offset to start reading from
     * @param size Length of the data to read
     * @return true if the registers were successfully read
     */
    bool readRegs(uint8_t *data, uint8_t offset = 0, uint8_t size = 1)
    {
        setMode(Mode::Config);
        if (offset + size > sizeof(moduleRegs))
            return false;

        loraSerial.write(0xC1);
        loraSerial.write(offset);
        loraSerial.write(size);

        uint8_t response[size + 3];
        loraSerial.readBytes(response, size + 3);
        if (response[0] != 0xC1)
            return false;
        if (response[1] != offset)
            return false;
        if (response[2] != size)
            return false;

        setMode(Mode::Transmit);

        memcpy(data, response + 3, size);
        return true;
    }

    inline bool readAllRegs()
    {
        return readRegs(reinterpret_cast<uint8_t *>(&moduleRegs), 0, sizeof(moduleRegs));
    }

    /**
     * @brief Switch to config mode and write the registers to the module, then switch back to transmit mode
     *
     * @param data Data to write to the module
     * @param offset Offset to start writing from
     * @param size Length of the data to write
     * @return true if the registers were successfully written
     */
    bool writeRegs(const uint8_t *data, const uint8_t offset = 0, const uint8_t size = 1)
    {
        setMode(Mode::Config);

        if (offset + size > sizeof(moduleRegs))
            return false;

        loraSerial.write(0xC0);
        loraSerial.write(offset);
        loraSerial.write(size);
        loraSerial.write(data, size);

        uint8_t response[3];
        loraSerial.readBytes(response, 3);
        if (response[0] != 0xC0)
            return false;
        if (response[1] != offset)
            return false;
        if (response[2] != size)
            return false;

        setMode(Mode::Transmit);
        return true;
    }

    inline bool writeAllRegs()
    {
        return writeRegs(reinterpret_cast<uint8_t *>(&moduleRegs), 0, sizeof(moduleRegs));
    }

    inline void saveCfg(const ModuleRegs &cfg)
    {
        eeprom_buffer_fill();
        for (auto i = 0; i < sizeof(ModuleRegs); i++)
        {
            eeprom_buffered_write_byte(EEPROM_CFG_ADDRESS + i, reinterpret_cast<uint8_t *>(&moduleRegs)[i]);
        }
        // Mark end of config
        eeprom_buffered_write_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs), 0x11);
        eeprom_buffered_write_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs) + 1, 0x45);
        eeprom_buffered_write_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs) + 2, 0x14);
        eeprom_buffer_flush();
    }

    inline bool loadCfg(ModuleRegs &cfg)
    {
        eeprom_buffer_fill();

        // Check if config is valid
        uint8_t check[] = {
            eeprom_buffered_read_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs)),
            eeprom_buffered_read_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs) + 1),
            eeprom_buffered_read_byte(EEPROM_CFG_ADDRESS + sizeof(ModuleRegs) + 2)};
        if (check[0] != 0x11 || check[1] != 0x45 || check[2] != 0x14)
        {
            return false;
        }

        for (auto i = 0; i < sizeof(cfg); i++)
        {
            reinterpret_cast<uint8_t *>(&cfg)[i] = eeprom_buffered_read_byte(EEPROM_CFG_ADDRESS + i);
        }
        writeAllRegs();
        return true;
    }

public:
    LoraModule(HardwareSerial &serial, int tx, int rx, int m0, int m1) : loraSerial(serial), pinM0(m0), pinM1(m1)
    {
        loraSerial.setTx(tx);
        loraSerial.setRx(rx);
        loraSerial.setTimeout(50);
        loraSerial.begin(9600);
        pinMode(pinM0, OUTPUT);
        pinMode(pinM1, OUTPUT);
    }

    enum class Mode : uint8_t
    {
        Transmit = 0,
        WOR,
        Config,
        Sleep,
    };

    void setMode(Mode mode)
    {
        digitalWrite(pinM0, static_cast<uint8_t>(mode) & 0x01);
        digitalWrite(pinM1, static_cast<uint8_t>(mode) & 0x02);
        delay(30); // Wait for mode switch
    }

    /**
     * @brief Initialize the lora module and load the config from EEPROM
     *
     * @return true if the module was successfully initialized
     */
    inline bool begin()
    {
        if (!readAllRegs())
        {
            ULOG_ERROR("Failed to read lora registers");
            return false;
        }

        if (memcmp(&moduleRegs, &defaultRegs, sizeof(ModuleRegs)))
        {
            ULOG_WARNING("Lora registers are different from saved config, applying the saved config");
            moduleRegs = defaultRegs;
            if (!writeAllRegs())
            {
                ULOG_ERROR("Failed to write lora registers");
                return false;
            }
        }
        return true;
    }

    inline void setAddr(uint16_t addr)
    {
        moduleRegs.addH = (addr >> 8) & 0xFF;
        moduleRegs.addL = addr & 0xFF;
    }

    inline uint16_t getAddr()
    {
        return (moduleRegs.addH << 8) | moduleRegs.addL;
    }

    inline void setNetId(uint8_t netId)
    {
        moduleRegs.netId = netId;
    }

    inline void setTxMode(TxMode txMode)
    {
        moduleRegs.cfg3 &= ~(1 << 6);
        moduleRegs.cfg3 |= static_cast<uint8_t>(txMode) << 6;
    }

    /**
     * @brief Set the channel of the module
     *
     * @param channel Channel number (0-80)
     * @note The channel is calculated as 850.125MHz + channel * 1MHz
     */
    inline void setChannel(uint8_t channel)
    {
        if (channel > 80)
            return;
        moduleRegs.channel = channel;
    }

    /**
     * @brief Commit the changes to the module registers
     *
     * @return true if the changes were successfully committed
     */
    inline bool commit()
    {
        return writeAllRegs();
    }

    /**
     * @brief Send data to a specific node in the network
     *
     * @param addr Address of the node to send data to
     * @param channel Channel of the node to send data to
     * @param data Data to send
     * @param size Size of the data
     * @note The module must be in P2P mode
     */
    inline void sendP2P(const uint16_t addr, const uint8_t channel, const uint8_t *data, size_t size)
    {
        loraSerial.write(addr >> 8);
        loraSerial.write(addr & 0xFF);
        loraSerial.write(moduleRegs.channel);
        loraSerial.write(data, size);
    }

    /**
     * @brief Send data to a specific node in the network
     *
     * @param addr Address of the node to send data to
     * @param channel Channel of the node to send data to
     * @param data Data vector to send
     * @note The module must be in P2P mode
     */
    inline void sendP2P(const uint16_t addr, const uint8_t channel, const std::vector<uint8_t> data)
    {
        sendP2P(addr, channel, data.data(), data.size());
    }

    /**
     * @brief Send a single byte to a specific node in the network
     *
     * @param addr Address of the node to send data to
     * @param channel Channel of the node to send data to
     * @param byte Data byte to send
     * @note The module must be in P2P mode
     */
    inline void sendP2P(const uint16_t addr, const uint8_t channel, const uint8_t byte)
    {
        loraSerial.write(addr >> 8);
        loraSerial.write(addr & 0xFF);
        loraSerial.write(moduleRegs.channel);
        loraSerial.write(byte);
    }

    /**
     * @brief Send data to all nodes in the network
     *
     * @param data Data to send
     * @param size Size of the data
     * @note The module must be in broadcast mode
     */
    inline void sendBroadcast(const uint8_t *data, const size_t size)
    {
        loraSerial.write(data, size);
    }

    /**
     * @brief Send data to all nodes in the network
     *
     * @param data Data vector to send
     * @note The module must be in broadcast mode
     */
    inline void sendBroadcast(const std::vector<uint8_t> data)
    {
        sendBroadcast(data.data(), data.size());
    }

    void receive(char *data, int size) {}
};

const LoraModule::ModuleRegs LoraModule::defaultRegs = {
    .addH = 0x00,
    .addL = LORA_SENSOR_ADDRESS_BASE + sensorID,
    .netId = netID,
    .cfg0 = 0b01100000, // 9600bps, 8N1, 2.4kbps
    .cfg1 = 0b00100000, // 240 bytes, LDO, No RSSI, Max power
    .channel = LORA_DONGLE_ADDRESS,
    .cfg3 = 0b01000000, // No RSSI, P2P mode, No relay, No LBT
    .cryptH = 0x00,
    .cryptL = 0x00,
};