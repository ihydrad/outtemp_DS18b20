/**
 * @file main.cpp
 * @brief Wireless Temperature Sensor with nRF24L01+ and DS18B20
 * @author Your Name
 * @date 2025
 * 
 * Battery-powered temperature sensor using:
 * - ATmega328P @ 1MHz for low power consumption
 * - DS18B20 temperature sensor
 * - nRF24L01+ wireless module
 * - Deep sleep mode with WDT wake-up
 */

// ================================================================================================
// INCLUDES
// ================================================================================================
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <RF24.h>
#include <OneWire.h>

// ================================================================================================
// DEBUG CONFIGURATION
// ================================================================================================
//#define DEBUG

#ifdef DEBUG
    #include <printf.h>
#endif

// ================================================================================================
// BIT MANIPULATION MACROS
// ================================================================================================
#define SET_BIT(port, bit)      ((port) |= (1 << (bit)))
#define CLEAR_BIT(port, bit)    ((port) &= ~(1 << (bit)))
#define TOGGLE_BIT(port, bit)   ((port) ^= (1 << (bit)))
#define BIT_IS_CLEAR(port, bit) (((port) & (1 << (bit))) == 0)
#define BIT_IS_SET(port, bit)   (((port) & (1 << (bit))) != 0)

// ================================================================================================
// HARDWARE CONFIGURATION
// ================================================================================================
// Sensor and communication settings
constexpr uint8_t  TRANSMITTER_ID      = 1;
constexpr uint8_t  TRANSMISSION_DELAY  = 30;      // Minutes between transmissions
constexpr uint8_t  PAYLOAD_SIZE         = 5;      // Data packet size in bytes
constexpr uint8_t  RF_CHANNEL          = 121;     // nRF24L01+ channel (0-127)

// Pin definitions
constexpr uint8_t  PIN_DS18B20_VCC     = 3;       // DS18B20 power control
constexpr uint8_t  PIN_DS18B20_DATA    = 4;       // DS18B20 data line
constexpr uint8_t  PIN_ADC_GROUND      = A3;      // ADC reference ground
constexpr uint8_t  PIN_BATTERY_ADC     = A2;      // Battery voltage measurement
constexpr uint8_t  PIN_ADC_VCC         = A1;      // ADC reference voltage
constexpr uint8_t  PIN_LED_DEBUG       = 7;       // Debug LED

// Measurement constants
constexpr float    ADC_VOLTAGE_DIVIDER = 2.15f;   // Voltage divider ratio
constexpr float    DS18B20_ERROR_TEMP  = -127.0f; // Error temperature value

// Power control macros for DS18B20
#define DS18B20_POWER_ON()    SET_BIT(PORTD, PD3)
#define DS18B20_POWER_OFF()   CLEAR_BIT(PORTD, PD3)

// ================================================================================================
// GLOBAL OBJECTS AND VARIABLES
// ================================================================================================
RF24    g_radio(9, 10);                           // nRF24L01+ (CE=9, CSN=10)
OneWire g_ds18b20(PIN_DS18B20_DATA);              // DS18B20 sensor

// Communication data
uint8_t g_data_packet[PAYLOAD_SIZE];              // Data to transmit
uint8_t g_tx_address[] = "1Node";                 // Transmission address

// System state
volatile uint8_t g_watchdog_counter = 7 * TRANSMISSION_DELAY;  // WDT counter
uint8_t          g_mcu_reset_source = 0;          // Last reset cause

// ================================================================================================
// FUNCTION DECLARATIONS
// ================================================================================================
void enableWatchdog(void);
void disableWatchdog(void);
void initializeRadio(void);
void enterSleepMode(void);
void wakeupFromSleep(void);
void prepareDataPacket(void);
void configureDS18B20(void);
float readDS18B20Temperature(void);

// ================================================================================================
// INTERRUPT SERVICE ROUTINES
// ================================================================================================
/**
 * @brief Watchdog Timer interrupt handler
 * Increments counter for transmission timing
 */
ISR(WDT_vect) {
    g_watchdog_counter++;
}

// ================================================================================================
// MAIN SETUP FUNCTION
// ================================================================================================
/**
 * @brief System initialization and configuration
 * Sets up CPU frequency, pins, peripherals and enters sleep mode
 */
void setup() {
    // Store and clear MCU status register
    g_mcu_reset_source = MCUSR;
    MCUSR = 0;
    
    // Set CPU frequency to 1MHz for power saving
    CLKPR = (1 << CLKPCE);                                    // Enable clock prescaler change
    CLKPR = (0 << CLKPCE) | (0 << CLKPS3) | (0 << CLKPS2) |  // Set prescaler to 8
            (1 << CLKPS1) | (1 << CLKPS0);                    // (8MHz / 8 = 1MHz)
    
    // Configure DS18B20 power pin
    SET_BIT(DDRD, PD3);                                       // Set as output
    CLEAR_BIT(PORTD, PD3);                                    // Initially off
    
    // Initialize peripherals
    initializeRadio();
    configureDS18B20();
    
    // Configure ADC
    analogReference(INTERNAL);                                // Use 1.1V internal reference
    ADCSRA = 0;                                              // Disable ADC for power saving
    
    // Disable analog comparator
    SET_BIT(ACSR, ACD);
    
    #ifdef DEBUG
        Serial.begin(9600);
        delay(1000);
        printf_begin();
        g_radio.printDetails();
        Serial.print(F("Last reset source: 0x"));
        Serial.println(g_mcu_reset_source, HEX);
        delay(1000);
    #endif
    
    // Configure sleep mode and enter deep sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    enterSleepMode();
}

// ================================================================================================
// TEMPERATURE SENSOR FUNCTIONS
// ================================================================================================
/**
 * @brief Read temperature from DS18B20 sensor
 * @return Temperature in Celsius, or DS18B20_ERROR_TEMP on error
 */
float readDS18B20Temperature() {
    #ifdef DEBUG
        byte sensor_data[9];
    #else
        byte sensor_data[2];
    #endif
    
    // Power on sensor and wait for stabilization
    DS18B20_POWER_ON();
    delay(5);
    
    // Check if sensor is present
    if (!g_ds18b20.reset()) {
        DS18B20_POWER_OFF();
        return DS18B20_ERROR_TEMP;
    }
    
    // Start temperature conversion
    g_ds18b20.write(0xCC);  // Skip ROM command
    g_ds18b20.write(0x44);  // Start conversion
    
    // Wait for conversion to complete (10-bit resolution: ~200ms)
    delay(200);
    
    // Read conversion result
    if (!g_ds18b20.reset()) {
        DS18B20_POWER_OFF();
        return DS18B20_ERROR_TEMP;
    }
    
    g_ds18b20.write(0xCC);  // Skip ROM command
    g_ds18b20.write(0xBE);  // Read scratchpad
    
    #ifdef DEBUG
        for (uint8_t i = 0; i < 9; i++) {
            sensor_data[i] = g_ds18b20.read();
        }
        Serial.print(F("DS18B20 Config: 0x"));
        Serial.println(sensor_data[4], HEX);
    #else
        sensor_data[0] = g_ds18b20.read();
        sensor_data[1] = g_ds18b20.read();
    #endif
    
    DS18B20_POWER_OFF();
    
    // Process temperature data (optimized calculation for 10-bit resolution)
    int16_t raw_temperature = (sensor_data[1] << 8) | sensor_data[0];
    float temperature = static_cast<float>(raw_temperature) / 16.0f;
    
    return temperature;
}

/**
 * @brief Configure DS18B20 sensor for optimal operation
 * Sets 10-bit resolution for fast conversion and saves to EEPROM
 */
void configureDS18B20() {
    DS18B20_POWER_ON();
    delay(10);  // Wait for power stabilization
    
    if (!g_ds18b20.reset()) {
        DS18B20_POWER_OFF();
        return;
    }
    
    // Configure sensor for 10-bit resolution (fast conversion)
    g_ds18b20.write(0xCC);  // Skip ROM command
    g_ds18b20.write(0x4E);  // Write scratchpad command
    g_ds18b20.write(0x7F);  // TH register (+127°C)
    g_ds18b20.write(0x80);  // TL register (-128°C)
    g_ds18b20.write(0x3F);  // Config: 10-bit resolution (375ms conversion)
    
    delay(10);
    
    // Save configuration to EEPROM
    g_ds18b20.reset();
    g_ds18b20.write(0xCC);  // Skip ROM command
    g_ds18b20.write(0x48);  // Copy scratchpad to EEPROM
    
    delay(15);  // Wait for EEPROM write
    g_ds18b20.reset();
    DS18B20_POWER_OFF();
}

// ================================================================================================
// RADIO COMMUNICATION FUNCTIONS
// ================================================================================================
/**
 * @brief Initialize nRF24L01+ radio module
 * Configures SPI interface and radio parameters for low power operation
 */
void initializeRadio() {
    // Configure SPI pins manually for optimal performance
    SET_BIT(DDRB, PB2);     // MOSI as output
    SET_BIT(DDRB, PB3);     // SCK as output  
    SET_BIT(DDRB, PB5);     // SS as output
    CLEAR_BIT(DDRB, PB4);   // MISO as input
    
    // Initialize SPI with optimized settings
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);  // Fast SPI for efficiency
    
    // Configure radio module
    g_radio.begin();
    g_radio.setChannel(RF_CHANNEL);
    g_radio.setDataRate(RF24_1MBPS);                    // Reliable speed for battery operation
    g_radio.setPALevel(RF24_PA_HIGH);                   // Good range with power efficiency
    g_radio.disableDynamicPayloads();
    g_radio.setPayloadSize(PAYLOAD_SIZE);
    g_radio.setAutoAck(false);                          // Disable auto-acknowledgment
    g_radio.setRetries(0, 0);                          // No retries for power saving
    g_radio.setAddressWidth(5);
    g_radio.openWritingPipe(g_tx_address);
    g_radio.setCRCLength(RF24_CRC_8);
    g_radio.stopListening();
    g_radio.powerDown();                                // Power down immediately
}

// ================================================================================================
// POWER MANAGEMENT FUNCTIONS  
// ================================================================================================
/**
 * @brief Enable Watchdog Timer for 8-second intervals
 * Configures WDT to generate interrupts every 8 seconds for wake-up timing
 */
void enableWatchdog() {
    cli();                                              // Disable interrupts
    wdt_reset();                                        // Reset watchdog
    CLEAR_BIT(MCUSR, WDRF);                            // Clear watchdog reset flag
    SET_BIT(WDTCSR, WDCE);                             // Enable watchdog change
    SET_BIT(WDTCSR, WDE);
    WDTCSR = (1 << WDP3) | (1 << WDP0);                // Set 8-second timeout
    SET_BIT(WDTCSR, WDIE);                             // Enable watchdog interrupt
    sei();                                              // Enable interrupts
    
    #ifdef DEBUG
        Serial.print(F(". "));
        delay(100);
    #endif
}

/**
 * @brief Disable Watchdog Timer
 * Safely disables watchdog timer operation
 */
void disableWatchdog() {
    cli();                                              // Disable interrupts
    wdt_reset();                                        // Reset watchdog
    CLEAR_BIT(MCUSR, WDRF);                            // Clear watchdog reset flag
    SET_BIT(WDTCSR, WDCE);                             // Enable watchdog change
    SET_BIT(WDTCSR, WDE);
    WDTCSR = (1 << WDP3) | (1 << WDP0);                // Set timeout
    WDTCSR = 0;                                         // Disable watchdog
    sei();                                              // Enable interrupts
    
    #ifdef DEBUG
        Serial.print(F("WDT_DIS->"));
        delay(100);
    #endif
}

/**
 * @brief Enter deep sleep mode with maximum power saving
 * Configures all pins for minimum power consumption and enters sleep
 */
void enterSleepMode() {
    g_radio.powerDown();
    
    // Disable ADC for power saving
    ADCSRA = 0;
    
    // Save current pin states for restoration after wake-up
    const uint8_t saved_ddrb  = DDRB;
    const uint8_t saved_ddrd  = DDRD;
    const uint8_t saved_ddrc  = DDRC;
    const uint8_t saved_portb = PORTB;
    const uint8_t saved_portd = PORTD;
    const uint8_t saved_portc = PORTC;
    
    // Set all unused pins as inputs with pull-ups for minimum power consumption
    DDRB = 0x00;    // All pins as inputs
    DDRD = 0x00;
    DDRC = 0x00;
    
    PORTB = 0xFF;   // Enable pull-ups
    PORTD = 0xFF;
    PORTC = 0xFF;
    
    // Keep working pins in correct state
    SET_BIT(DDRD, PD3);                                // DS18B20 power pin as output
    CLEAR_BIT(PORTD, PD3);                             // Keep DS18B20 power off
    
    // Enable sleep and watchdog timer
    sleep_enable();
    enableWatchdog();
    
    // Disable Brown-out Detector during sleep
    MCUCR = (1 << BODS) | (1 << BODSE);
    MCUCR = (1 << BODS);
    sleep_cpu();
    
    // Restore pin states after wake-up
    sleep_disable();
    DDRB  = saved_ddrb;
    DDRD  = saved_ddrd;
    DDRC  = saved_ddrc;
    PORTB = saved_portb;
    PORTD = saved_portd;
    PORTC = saved_portc;
}

/**
 * @brief Wake up from sleep mode and initialize peripherals
 * Restores system state after wake-up from deep sleep
 */
void wakeupFromSleep() {
    disableWatchdog();
    g_radio.powerUp();
    delay(2);  // Wait for nRF24 stabilization
    
    // Configure pins for battery voltage measurement
    pinMode(PIN_BATTERY_ADC, INPUT);
    pinMode(PIN_ADC_VCC, OUTPUT);
    digitalWrite(PIN_ADC_VCC, HIGH);
    pinMode(PIN_ADC_GROUND, OUTPUT);
    digitalWrite(PIN_ADC_GROUND, LOW);
    
    // Enable ADC with power-efficient prescaler
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Prescaler 128
}

// ================================================================================================
// DATA PREPARATION AND TRANSMISSION
// ================================================================================================
/**
 * @brief Prepare data packet for transmission
 * Reads sensors and formats data into transmission packet
 */
void prepareDataPacket() {
    g_data_packet[0] = TRANSMITTER_ID;
    
    // Include reset source information if available
    if (g_mcu_reset_source) {
        g_data_packet[0] |= (g_mcu_reset_source << 4);
        g_mcu_reset_source = 0;
    }
    
    // Read temperature sensor
    float temperature = readDS18B20Temperature();
    int16_t temp_encoded = static_cast<int16_t>(temperature * 100.0f);
    
    // Stabilize before ADC reading
    delay(2);
    uint16_t adc_raw = analogRead(PIN_BATTERY_ADC);
    int16_t battery_voltage = static_cast<int16_t>(adc_raw / ADC_VOLTAGE_DIVIDER);
    
    // Pack data into transmission format
    g_data_packet[1] = static_cast<uint8_t>(temp_encoded >> 8);
    g_data_packet[2] = static_cast<uint8_t>(temp_encoded & 0xFF);
    g_data_packet[3] = static_cast<uint8_t>(battery_voltage >> 8);
    g_data_packet[4] = static_cast<uint8_t>(battery_voltage & 0xFF);

    #ifdef DEBUG
        Serial.print(F("STATE: 0x"));
        Serial.println(g_data_packet[0], HEX);
        Serial.print(F("BAT ADC: "));
        Serial.print(adc_raw);
        Serial.print(F(", V: "));
        Serial.println(battery_voltage / 100.0f, 2);
        Serial.print(F("Temp: "));
        Serial.println(temperature, 2);
        Serial.print(F("Data: "));
        for (uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
            Serial.print(F("0x"));
            if (g_data_packet[i] < 0x10) Serial.print(F("0"));
            Serial.print(g_data_packet[i], HEX);
            if (i < PAYLOAD_SIZE - 1) Serial.print(F(" "));
        }
        Serial.println();
    #endif
}

// ================================================================================================
// MAIN PROGRAM LOOP
// ================================================================================================
/**
 * @brief Main program loop
 * Checks transmission timing and handles sensor reading/data transmission cycles
 */
void loop() {
    // Check if it's time to transmit (approximately every minute)
    if (g_watchdog_counter >= (7 * TRANSMISSION_DELAY)) {
        g_watchdog_counter = 0;
        
        // Wake up peripherals and prepare for transmission
        wakeupFromSleep();
        
        // Indicate activity with debug LED
        SET_BIT(DDRD, PD7);                            // LED as output
        SET_BIT(PORTD, PD7);                           // Turn on LED
        
        // Prepare and transmit data
        prepareDataPacket();
        bool transmission_result = g_radio.write(&g_data_packet, PAYLOAD_SIZE);
        
        #ifdef DEBUG
            Serial.print(F("TX result: "));
            Serial.println(transmission_result ? F("OK") : F("FAIL"));
            delay(100);
        #endif
        
        // Turn off debug LED
        CLEAR_BIT(PORTD, PD7);                         // Turn off LED
        CLEAR_BIT(DDRD, PD7);                          // LED as input
    }
    
    // Return to deep sleep mode
    enterSleepMode();
}
