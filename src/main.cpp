#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <RF24.h>
#include <OneWire.h>

//#define DEBUG

#ifdef DEBUG
  #include <printf.h>
#endif

#define SBR(port, bit)        port |= (1<<bit)
#define CBR(port, bit)        port &= (~(1<<bit))
#define INV(port, bit)        port ^= (1<<bit)
#define SBRC(port, bit)      ((port & (1<<bit)) == 0)
#define SBRS(port, bit)      ((port & (1<<bit)) != 0)

#define TR_NUM               1
#define TR_DELMIN            30
#define DS_VCC               3
#define DS_DTA               4
#define ADCGND               A3
#define ADCBATPIN            A2
#define ADCVCC               A1
#define LED_DEBUG            7
#define DIV_ADC              2.15f  // Использование float константы
#define PayloadSize          5
#define Channel              121
// Более быстрые прямые манипуляции с портами вместо digitalWrite
#define DS_POW_EN            PORTD |= (1<<PD3)
#define DS_POW_DIS           PORTD &= ~(1<<PD3) 


RF24            radio(9, 10); //4 for CE and 15 for CSN
OneWire         ds(DS_DTA);

uint8_t data[PayloadSize],
        tx_adrr[]="1Node",
        wdt_cnt = 7*TR_DELMIN,
        mcur;

void enWDT(void);
void disWDT();
void initNRF(void);
void sleep(void);
void wakeup(void);
void prepData(void);
void conf_ds18b20(void);

ISR(WDT_vect){
  wdt_cnt++;
}

void setup() {  
  mcur = MCUSR;
  MCUSR = 0;
  // Установка частоты CPU на 1МГц для экономии энергии
  CLKPR = (1<<CLKPCE);
  CLKPR = (0<<CLKPCE)|(0<<CLKPS3)|(0<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0);  
  
  // Настройка пинов для DS18B20
  DDRD |= (1<<PD3);  // DS_VCC как выход
  PORTD &= ~(1<<PD3); // Изначально выключен
  
  initNRF();
  conf_ds18b20(); // Включаем конфигурацию DS18B20
  
  // Настройка ADC
  analogReference(INTERNAL);
  ADCSRA = 0; // Отключаем ADC для экономии
  
  // Отключаем аналоговый компаратор
  ACSR = (1<<ACD);  
  
  #ifdef DEBUG
    Serial.begin(9600);
    delay(1000);
    printf_begin();
    radio.printDetails();
    Serial.print("Last reset source: ");    
    Serial.println(mcur, HEX);
    delay(1000);
  #endif  
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep();
}

float read_ds18b20(){
  #ifdef DEBUG
    byte data[9];
  #else
    byte data[2];
  #endif
  
  DS_POW_EN; 
  delay(5); // Небольшая пауза для стабилизации питания
  
  if(!ds.reset()){ // Проверяем наличие датчика
    DS_POW_DIS;
    return -127.0f;
  }
  
  // Запуск преобразования
  ds.write(0xCC); // Skip ROM
  ds.write(0x44); // Start conversion
  
  // Ожидание завершения (для 10-bit достаточно 200мс)
  delay(200);
  
  // Чтение результата
  if(!ds.reset()){
    DS_POW_DIS;
    return -127.0f;
  }
  
  ds.write(0xCC); // Skip ROM
  ds.write(0xBE); // Read scratchpad
  
  #ifdef DEBUG
    for(uint8_t i = 0; i < 9; i++)
      data[i] = ds.read();
    Serial.print("CFG DS = 0x");
    Serial.println(data[4], HEX);
  #else
    data[0] = ds.read();
    data[1] = ds.read();
  #endif
  
  DS_POW_DIS;
  
  // Обработка температуры (оптимизированный расчет)
  int16_t raw = (data[1] << 8) | data[0];
  
  // Для 10-bit разрешения используем деление на 16
  float temp = (float)raw / 16.0f;
  
  return temp;
}

void conf_ds18b20(){
  DS_POW_EN;
  delay(10); // Ждем стабилизации питания
  
  if(!ds.reset()){
    DS_POW_DIS;
    return;
  }
  
  // Конфигурируем датчик на 10-bit разрешение для быстрого чтения
  ds.write(0xCC);  // Skip ROM
  ds.write(0x4E);  // Write Scratchpad
  ds.write(0x7F);  // TH register (+127°C)
  ds.write(0x80);  // TL register (-128°C) 
  ds.write(0x3F);  // Config register: 10-bit resolution (375ms conversion)
  
  delay(10);
  
  // Сохраняем конфигурацию в EEPROM
  ds.reset();
  ds.write(0xCC);  // Skip ROM
  ds.write(0x48);  // Copy scratchpad to EEPROM
  
  delay(15); // Ждем записи в EEPROM
  ds.reset();
  DS_POW_DIS;
}

void initNRF(){
  // Настройка SPI пинов как выходы
  DDRB |= (1<<PB2) | (1<<PB3) | (1<<PB5); // MOSI, SCK, SS
  DDRB &= ~(1<<PB4); // MISO как вход
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); // Ускоряем SPI
  
  radio.begin();
  radio.setChannel(125);
  radio.setDataRate(RF24_1MBPS); // Более надежная скорость для батарейного питания
  radio.setPALevel(RF24_PA_HIGH); // Достаточная мощность, экономия энергии
  radio.disableDynamicPayloads();
  radio.setPayloadSize(PayloadSize);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setAddressWidth(5);
  radio.openWritingPipe(tx_adrr);
  radio.setCRCLength(RF24_CRC_8);
  radio.stopListening();
  radio.powerDown(); // Сразу выключаем для экономии
}

void enWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR |= bit (WDIE);
  sei();
  #ifdef DEBUG
    Serial.print(". ");
    delay(1000);
  #endif
}

void disWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR  = 0;
  sei();
  #ifdef DEBUG
    Serial.print("WDT_DIS->");
    delay(1000);
  #endif
}

void sleep(){
  radio.powerDown();
  
  // Отключаем ADC для экономии энергии
  ADCSRA = 0;
  
  // Настраиваем все пины на выход и низкий уровень для минимального потребления
  // Сохраняем состояние важных пинов
  uint8_t oldDDRB = DDRB;
  uint8_t oldDDRD = DDRD; 
  uint8_t oldDDRC = DDRC;
  uint8_t oldPORTB = PORTB;
  uint8_t oldPORTD = PORTD;
  uint8_t oldPORTC = PORTC;
  
  // Устанавливаем все неиспользуемые пины в состояние INPUT_PULLUP для экономии
  DDRB = 0x00;  // Все как входы
  DDRD = 0x00;
  DDRC = 0x00;
  
  PORTB = 0xFF; // Подтяжки включены
  PORTD = 0xFF;
  PORTC = 0xFF;
  
  // Но исключаем рабочие пины
  DDRD |= (1<<PD3);  // DS_VCC остается выходом
  PORTD &= ~(1<<PD3); // И в низком состоянии
  
  // Отключаем Brown-out detector в программном режиме
  sleep_enable();
  enWDT();
  
  MCUCR = (1<<BODS) | (1<<BODSE);
  MCUCR = (1<<BODS);
  sleep_cpu();
  
  // Восстанавливаем состояние после пробуждения
  sleep_disable();
  
  DDRB = oldDDRB;
  DDRD = oldDDRD;
  DDRC = oldDDRC;
  PORTB = oldPORTB;
  PORTD = oldPORTD;
  PORTC = oldPORTC;
}

void wakeup(){
  disWDT();
  radio.powerUp();
  delay(2); // Небольшая пауза для стабилизации nRF24
  
  // Настройка пинов для измерения батареи
  pinMode(ADCBATPIN, INPUT);
  pinMode(ADCVCC, OUTPUT);
  digitalWrite(ADCVCC, HIGH);
  pinMode(ADCGND, OUTPUT);
  digitalWrite(ADCGND, LOW);
  
  // Включаем ADC с предделителем для экономии энергии
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Предделитель 128
}

void prepData(){
  data[0] = TR_NUM;
  if(mcur) {
    data[0] |= (mcur << 4);
    mcur = 0;   
  }
  
  float temperature = read_ds18b20();
  int16_t temp = (int16_t)(temperature * 100.0f);
  
  // Стабилизация перед измерением ADC
  delay(2);
  uint16_t adc_raw = analogRead(ADCBATPIN);
  int16_t bat = (int16_t)(adc_raw / DIV_ADC);
  
  data[1] = (uint8_t)(temp >> 8);
  data[2] = (uint8_t)(temp & 0xFF);  
  data[3] = (uint8_t)(bat >> 8);  
  data[4] = (uint8_t)(bat & 0xFF);

  #ifdef DEBUG    
    Serial.print("STATE: 0x");
    Serial.println(data[0], HEX);
    Serial.print("BAT ADC: ");
    Serial.print(adc_raw);
    Serial.print(", V: ");
    Serial.println(bat / 100.0f, 2);
    Serial.print("Temp: ");
    Serial.println(temperature, 2);  
    Serial.print("Data: ");
    for(uint8_t i = 0; i < PayloadSize; i++){
      Serial.print("0x");
      if(data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if(i < PayloadSize-1) Serial.print(" ");
    }
    Serial.println();    
  #endif 
}

void loop() {
  if(wdt_cnt >= (7 * TR_DELMIN)){    // 7.4*8sec ≈ 1min            
    wdt_cnt = 0;    
    wakeup();    
    
    // Быстрое включение/выключение LED для индикации
    DDRD |= (1<<PD7);   // LED_DEBUG как выход
    PORTD |= (1<<PD7);  // Включаем LED
    
    prepData();
    
    // Попытка отправки с простой проверкой
    bool result = radio.write(&data, PayloadSize);
    
    #ifdef DEBUG
      Serial.print("TX result: ");
      Serial.println(result ? "OK" : "FAIL");
      delay(100);
    #endif
    
    PORTD &= ~(1<<PD7); // Выключаем LED
    DDRD &= ~(1<<PD7);  // LED_DEBUG как вход
  }                                             
  sleep();                                                        
}
