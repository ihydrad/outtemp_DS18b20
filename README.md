# Wireless Temperature Sensor Project

## 📖 Overview

A battery-powered wireless temperature sensor using ATmega328P microcontroller with ultra-low power consumption design. The sensor measures ambient temperature using DS18B20 sensor and transmits data via nRF24L01+ radio module.

## 🔧 Hardware Components

### Main Components
- **MCU**: ATmega328P @ 1MHz (power optimized)
- **Temperature Sensor**: DS18B20 (1-Wire digital sensor)
- **Wireless Module**: nRF24L01+ (2.4GHz transceiver)
- **Power**: Battery powered with voltage monitoring

### Pin Configuration
| Component | Arduino Pin | ATmega Pin | Description |
|-----------|-------------|------------|-------------|
| DS18B20 VCC | D3 | PD3 | Power control for DS18B20 |
| DS18B20 Data | D4 | PD4 | 1-Wire data line |
| nRF24 CE | D9 | PB1 | Chip Enable |
| nRF24 CSN | D10 | PB2 | SPI Chip Select |
| nRF24 MOSI | D11 | PB3 | SPI Master Out |
| nRF24 MISO | D12 | PB4 | SPI Master In |
| nRF24 SCK | D13 | PB5 | SPI Clock |
| Battery ADC | A2 | PC2 | Battery voltage measurement |
| ADC VCC | A1 | PC1 | ADC reference voltage |
| ADC GND | A3 | PC3 | ADC reference ground |
| Debug LED | D7 | PD7 | Status indication |

## ⚡ Power Management Features

### Ultra-Low Power Design
- **Deep Sleep Mode**: ATmega328P in SLEEP_MODE_PWR_DOWN
- **CPU Frequency**: Reduced to 1MHz for power efficiency
- **Watchdog Timer**: 8-second intervals for wake-up timing
- **Peripheral Control**: ADC and unused peripherals disabled during sleep
- **Brown-out Detector**: Disabled during sleep for additional power saving

### Power Consumption Optimization
- All unused pins configured as INPUT_PULLUP during sleep
- DS18B20 powered only during measurements
- nRF24L01+ powered down between transmissions
- Optimized SPI communication for fast data transfer

## 📊 Data Transmission

### Packet Format (5 bytes)
| Byte | Bits 7-4 | Bits 3-0 | Description |
|------|----------|----------|-------------|
| 0 | Reset Source | Transmitter ID | Status and identification |
| 1 | Temperature High Byte | Temperature data (signed) |
| 2 | Temperature Low Byte | °C × 100 format |
| 3 | Battery High Byte | Battery voltage data |
| 4 | Battery Low Byte | Voltage × 100 format |

### Transmission Settings
- **Frequency**: 2.4GHz (Channel 121)
- **Data Rate**: 1Mbps (reliable for battery operation)
- **Power Level**: RF24_PA_HIGH (good range with efficiency)
- **Transmission Interval**: ~1 minute (adjustable)
- **Address**: "1Node" (5-byte address)

## 🔬 Sensor Configuration

### DS18B20 Temperature Sensor
- **Resolution**: 10-bit (0.25°C precision)
- **Conversion Time**: ~200ms
- **Temperature Range**: -55°C to +125°C
- **Power Mode**: Parasitic power with external VCC control
- **Error Handling**: Returns -127.0°C on communication failure

### Battery Monitoring
- **ADC Reference**: Internal 1.1V
- **Voltage Divider**: 2.15:1 ratio
- **Resolution**: ~0.01V precision
- **Range**: 0-3.3V typical battery range

## 💻 Software Architecture

### Code Style Features
- **Modern C++**: Uses constexpr, static_cast, const correctness
- **Descriptive Naming**: Clear function and variable names
- **Comprehensive Documentation**: Doxygen-style comments
- **Error Handling**: Robust sensor communication checks
- **Memory Efficiency**: Optimized data structures and algorithms

### Function Organization
```cpp
// Hardware abstraction
void initializeRadio()
void configureDS18B20()
float readDS18B20Temperature()

// Power management
void enableWatchdog()
void disableWatchdog()
void enterSleepMode()
void wakeupFromSleep()

// Data handling
void prepareDataPacket()
```

### Debug Features
- **Conditional Compilation**: `#ifdef DEBUG` blocks
- **Serial Output**: Detailed sensor readings and transmission status
- **Visual Indication**: LED blink during transmission
- **Reset Source Tracking**: MCU status register monitoring

## 🛠️ Build Configuration

### PlatformIO Settings
```ini
[env:pro16MHzatmega328]
platform = atmelavr
board = pro16MHzatmega328
framework = arduino
board_build.f_cpu = 1000000L  ; 1MHz for power efficiency
lib_deps = 
    nrf24/RF24@^1.3.12
    paulstoffregen/OneWire@^2.3.5
```

### Compilation Flags
- Optimized for size and power consumption
- Debug symbols available in DEBUG mode
- Hardware-specific optimizations enabled

## 🚀 Performance Characteristics

### Battery Life Estimation
- **Sleep Current**: ~1-5µA (deep sleep mode)
- **Active Current**: ~20-30mA (during transmission)
- **Duty Cycle**: <0.1% (active time)
- **Estimated Battery Life**: 6-12 months on 2×AA batteries

### Communication Range
- **Line of Sight**: 50-100m typical
- **Indoor Range**: 10-30m depending on obstacles
- **Frequency**: 2.425GHz (Channel 121)
- **Interference Resistance**: Good with auto-retry disabled

## 📋 Usage Instructions

### Initial Setup
1. Configure hardware connections according to pin mapping
2. Compile and upload firmware using PlatformIO
3. Power on with battery pack
4. Monitor serial output in DEBUG mode for verification

### Operation Modes
- **Normal Mode**: Automatic sensor readings every ~1 minute
- **Debug Mode**: Enable `#define DEBUG` for detailed logging
- **Sleep Mode**: Ultra-low power between transmissions

### Troubleshooting
- **No Transmission**: Check nRF24L01+ connections and power
- **Temperature Errors**: Verify DS18B20 wiring and power control
- **High Power Consumption**: Ensure proper sleep mode configuration

## 📈 Future Enhancements

### Potential Improvements
- **Adaptive Transmission**: Variable intervals based on temperature changes
- **Multi-Sensor Support**: Support for multiple DS18B20 sensors
- **Encryption**: Secure data transmission implementation
- **OTA Updates**: Over-the-air firmware update capability
- **Mesh Networking**: Multi-hop communication for extended range

### Hardware Upgrades
- **External Antenna**: Improved communication range
- **Solar Charging**: Self-sustaining power system
- **Environmental Sensors**: Humidity, pressure, light sensors
- **Enclosure Design**: Weatherproof housing for outdoor use

## 📄 License

This project is open-source and available under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for improvements and bug fixes.

---

**Project Status**: ✅ Stable and Production Ready  
**Last Updated**: October 2025  
**Version**: 2.0.0