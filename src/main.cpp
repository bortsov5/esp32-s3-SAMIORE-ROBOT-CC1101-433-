#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <EEPROM.h>
#include <RCSwitch.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 8
#define OLED_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RCSwitch mySwitch = RCSwitch();

const int arraySize = 128;
int rssiValues[arraySize]; // массив для хранения значений RSSI
int Sindex = 0;            // текущий индекс для заполнения массива
int printIndex = 0;
bool serialOutput = false;
String inputString = "";  // строка для приема команд
bool stringComplete = false;

#define CCBUFFERSIZE 64
#define RECORDINGBUFFERSIZE 4096 // Buffer for recording the frames
#define EPROMSIZE 512            // Size of EEPROM in your Arduino chip. For ESP32 it is Flash simulated only 512 bytes, ESP8266 is 4096
#define BUF_LENGTH 128           // Buffer for the incoming command.

#define ARRAY_SIZE 3210
#define SHIFT_AMOUNT 107
int8_t rssiHistorty[ARRAY_SIZE];
const int rssi_threshold = -75;

// defining PINs set for ESP32 module
// Example for XIAO ESP32 C3
byte sck = 14;  // GPIO 8
byte miso = 12; // GPIO 4
byte mosi = 13; // GPIO 10
byte ss = 15;   // GPIO 20
int gdo0 = 2;   // GPIO 21
int gdo2 = 3;   // GPIO 7

#define RX_PIN gdo2 // Pino de recepção
#define TX_PIN gdo0 // Pino de transmissão

// 300-348 MHz,
// 387-464 MHz and
// 779-928 MHz

float settingf1 = 300.0f;
float st = 0.01f;
float currentMhz = 300.0f;
int dopMhz = 64;
float settingf2 = 301.28f;
float freq = 300.0f;
int rssi = -100;
int menu_mode = 1;
int sel_mode = 1; // 1 - left, 2 - step, 3 - right
int8_t event = 0;
int8_t FreezeMode = 0;
int setVizFilt = 90; // срез сигнала

#define BUTTON_PINUP 4    // вверх
#define BUTTON_PINDOWN 5  // вниз
#define BUTTON_PINLEFT 6  // вправо
#define BUTTON_PINRIGHT 7 // влево

const unsigned char analize[] PROGMEM = {
    0b00011000,
    0b00111100,
    0b01100110,
    0b11000011,
    0b11000011,
    0b11111111,
    0b11000011,
    0b11000011};

const unsigned char cherep[] PROGMEM = {
    0b01111100,
    0b10010010,
    0b11101110,
    0b11101110,
    0b01111100,
    0b00000000,
    0b01111100,
    0b00000000};

const unsigned char cherepOn[] PROGMEM = {
    0b10000011,
    0b01101101,
    0b00010001,
    0b00010001,
    0b10000011,
    0b11111111,
    0b10000011,
    0b11111111};

const unsigned char selLeft[] PROGMEM = {
    0b00010000,
    0b00110000,
    0b01110000,
    0b11111111,
    0b01110000,
    0b00110000,
    0b00010000,
    0b00000000};

const unsigned char selRight[] PROGMEM = {
    0b00001000, // Пиксель 3 (зеркально относительно исходного)
    0b00001100, // Пиксели 3 и 2
    0b00001110, // Пиксели 3, 2, 1
    0b11111111, // Основание (оставляем таким же)
    0b00001110,
    0b00001100,
    0b00001000,
    0b00000000};

const unsigned char sel2[] PROGMEM = {
    0b00011000,
    0b00111100,
    0b11111111};

const unsigned char menu[] PROGMEM = {
    0b11111111,
    0b00000000,
    0b11111111,
    0b00000000,
    0b11111111};

const unsigned char S[] PROGMEM = {
    0b01111111,
    0b10000000,
    0b01111110,
    0b00000001,
    0b11111110};

// Initialize CC1101 board with default settings, you may change your preferences here
static void cc1101initialize(void)
{
  // initializing library with custom pins selected
  ELECHOUSE_cc1101.setSpiPin(sck, miso, mosi, ss);
  ELECHOUSE_cc1101.setGDO(gdo0, gdo2);

  // Main part to tune CC1101 with proper frequency, modulation and encoding
  ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setGDO0(gdo0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
  ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode. value 0 is for RAW recording/replaying
  ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
  ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
  ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
  ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
  ELECHOUSE_cc1101.setDRate(9.6);         // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
  ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. Default is 211,145 (Syncword high, Syncword low)
  ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
  ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
  ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
  ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
  ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
  ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
  ELECHOUSE_cc1101.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
  ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
  ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
  ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
  ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
}

// Execute a complete CC1101 command.

int getMaxIndex()
{
  int maxVal = rssiValues[0];
  int maxIndex = 0;
  for (int i = 1; i < arraySize; i++)
  {
    if (rssiValues[i] < maxVal)
    {
      maxVal = rssiValues[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}

void btn()
{

  event = 0;
  if (digitalRead(BUTTON_PINUP) == LOW)
  {
    event = 1; // Representa U8X8_MSG_GPIO_MENU_NEXT
  }
  else if (digitalRead(BUTTON_PINDOWN) == LOW)
  {
    event = 2; // Representa U8X8_MSG_GPIO_MENU_PREV
  }
  else if (digitalRead(BUTTON_PINLEFT) == LOW)
  {
    event = 3; // Representa U8X8_MSG_GPIO_MENU_SELECT
  }
  else if (digitalRead(BUTTON_PINRIGHT) == LOW)
  {
    event = 4; // Representa a função de voltar para o menu principal
  }

  if (event > 0)
  {
    // Serial.println(event);
    if (event == 4)
    { // право
      sel_mode = sel_mode + 1;
    }

    if (event == 3)
    { // лево
      sel_mode = sel_mode - 1;
    }

    if (sel_mode == 1)
    {
      if (event == 2)
      {
        settingf1 = settingf1 + st * 128;
        // 300-348 MHz, 387-464MHz and 779-928 MHz
        if ((settingf1 >= 348.0f) && (settingf1 < 378.0f))
        {
          settingf1 = 387.0f;
        }
        if ((settingf1 >= 464.0f) && (settingf1 < 779.0f))
        {
          settingf1 = 779.0f;
        }
        if ((settingf1 > 928.0f - st * 128))
        {
          settingf1 = 928.0f - st * 128;
        }
      }

      if (event == 1)
      {
        settingf1 = settingf1 - st * 128;

        if ((settingf1 < 387.0f) && (settingf1 > 348.0f))
        {
          settingf1 = 348.0f - st * 128;
        }

        if ((settingf1 < 779.0f) && (settingf1 > 464.0f))
        {
          settingf1 = 464.0f - st * 128;
        }

        if (settingf1 < 300.0f)
        {
          settingf1 = 300.0f;
        }
      }
    }

    if (sel_mode == 2)
    {

      if (event == 2)
      {
        st = st + 0.1f;
      }

      if (event == 1)
      {
        st = st - 0.1f;
      }

      if (st < 0.1f)
      {
        st = 0.1f;
      }
    }

    if (sel_mode == 3)
    {
      if (event == 2)
      {
        dopMhz = dopMhz - 1;
      }

      if (event == 1)
      {
        dopMhz = dopMhz + 1;
      }

      if (dopMhz < 0)
      {
        dopMhz = 0;
      }

      if (dopMhz > 128)
      {
        dopMhz = 128;
      }
    }

    if (sel_mode == 4)
    {
      if (event == 2)
      {
        setVizFilt = setVizFilt - 1;
      }

      if (event == 1)
      {
        setVizFilt = setVizFilt + 1;
      }
    }

    if (sel_mode == 5)
    {

      if (event == 2)
      {
        FreezeMode = FreezeMode - 1;
      }

      if (event == 1)
      {
        FreezeMode = FreezeMode + 1;
      }

      if (FreezeMode < 0)
      {
        FreezeMode = 0;
        menu_mode = 1;
      }

      if (FreezeMode == 0)
      {
        menu_mode = 1;
      }

      if (FreezeMode == 1)
      {
        menu_mode = 2;
      }

      if ((FreezeMode == 2))
      {
        menu_mode = 3;
      }

      if (FreezeMode > 2)
      {
        FreezeMode = 3;
        menu_mode = 3;
      }
    }

    if (sel_mode <= 0)
    {
      sel_mode = 1;
    }

    if (sel_mode > 5)
    {
      sel_mode = 5;
    }

    currentMhz = settingf1 + dopMhz * st;

    delay(300);
  }
}

void DrawHistory()
{
  for (int y = 0; y < 30; y++)
  {
    for (int x = 0; x < 107; x++)
    {
      if (rssiHistorty[ARRAY_SIZE - y * 107 + x] > 0)
      {
        display.drawPixel(10 + x, 10 + y, SSD1306_WHITE);
      }
    }
  }
}

void Analyser()
{

  while (true)
  {
    btn();
    if ((menu_mode == 2))
    {
      break;
    }

    display.clearDisplay();
    display.setRotation(0);

    if (sel_mode == 1)
    {
      display.drawBitmap(0, 45, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 2)
    {
      display.drawBitmap(35, 10, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 3)
    {
      display.drawBitmap(80, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 4)
    {
      display.drawBitmap(110, 50, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 5)
    {
      display.drawBitmap(12, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    if (FreezeMode == 0)
    {
      display.drawBitmap(12, 1, cherep, 8, 8, SSD1306_WHITE);
    }
    if (FreezeMode == 1)
    {
      display.drawBitmap(12, 1, cherepOn, 8, 8, SSD1306_WHITE);
    }
    if (FreezeMode == 2)
    {
      display.drawBitmap(12, 1, analize, 8, 8, SSD1306_WHITE);
    }

    display.setCursor(0, 20);
    display.printf("Analyzing...");

    display.setCursor(75, 1);
    display.print(currentMhz);

    int rssi;
    uint32_t detectedFrequency = 0;
    int detectedRssi = -100;

    ELECHOUSE_cc1101.setMHZ(currentMhz);
    ELECHOUSE_cc1101.SetRx();
    delayMicroseconds(3500);
    rssi = ELECHOUSE_cc1101.getRssi();

    if (rssi >= rssi_threshold && rssi > detectedRssi)
    {
      detectedRssi = rssi;
      detectedFrequency = currentMhz;
    }

    if (detectedFrequency != 0)
    {
      display.setCursor(0, 20);
      display.printf("Signal detected:");
      display.setCursor(0, 30);
      display.printf("Frequency:%.2fMHz", (float)detectedFrequency / 1000000.0);
      display.setCursor(0, 40);
      display.printf("RSSI:%ddBm", detectedRssi);
      detectedFrequency = 0;
    }

    delay(600);

    display.display();
  }

  ELECHOUSE_cc1101.SetRx();
  mySwitch.disableTransmit();
  delay(100);
  mySwitch.enableReceive(RX_PIN);
}

void dead()
{

  ELECHOUSE_cc1101.setMHZ(currentMhz);

  while (true)
  {
    btn();
    if ((menu_mode == 1) || (menu_mode == 3))
    {
      break;
    }

    display.clearDisplay();
    display.setRotation(0);

    display.setCursor(75, 1);
    display.print(currentMhz);

    display.setCursor(0, 20);
    display.print("Sending Random:");

    if (sel_mode == 1)
    {
      display.drawBitmap(0, 45, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 2)
    {
      display.drawBitmap(35, 10, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 3)
    {
      display.drawBitmap(80, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 4)
    {
      display.drawBitmap(110, 50, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 5)
    {
      display.drawBitmap(12, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    if (FreezeMode == 0)
    {
      display.drawBitmap(12, 1, cherep, 8, 8, SSD1306_WHITE);
    }
    if (FreezeMode == 1)
    {
      display.drawBitmap(12, 1, cherepOn, 8, 8, SSD1306_WHITE);
    }
    if (FreezeMode == 2)
    {
      display.drawBitmap(12, 1, analize, 8, 8, SSD1306_WHITE);
    }

    unsigned long randomValue = 100000000 + random(900000000);
    int randomBitLength = 28;
    int randomProtocol = 0 + random(12);

    mySwitch.disableReceive();
    delay(100);
    mySwitch.enableTransmit(TX_PIN);
    ELECHOUSE_cc1101.SetTx();

    mySwitch.setProtocol(randomProtocol);
    mySwitch.send(randomValue, randomBitLength);

    delay(100);

    display.setCursor(0, 30);
    display.print(randomValue);
    display.setCursor(0, 40);
    display.print(randomProtocol);

    display.display();

    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(100);
    mySwitch.enableReceive(RX_PIN);
  }
}

void processSerialCommand(String command)
{
  command.toLowerCase();
  
  if (command == "serial on") {
    serialOutput = true;
    Serial.println("OK: Serial output enabled");
  }
  else if (command == "serial off") {
    serialOutput = false;
    Serial.println("OK: Serial output disabled");
  }
  else if (command == "serial status") {
    Serial.print("Serial output: ");
    Serial.println(serialOutput ? "ENABLED" : "DISABLED");
  }
  else if (command == "help") {
    Serial.println("Available commands:");
    Serial.println("serial on - Enable serial output");
    Serial.println("serial off - Disable serial output");
    Serial.println("serial status - Check serial output status");
    Serial.println("scan - Force start scanning");
    Serial.println("stop - Stop current operation");
  }
  else if (command == "scan") {
    menu_mode = 1;
    Serial.println("OK: Starting scan");
  }
  else if (command == "stop") {
    menu_mode = 2;
    Serial.println("OK: Stopping current operation");
  }
  else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
  }
}

void scan_com()
{
 ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setRxBW(58);
  ELECHOUSE_cc1101.SetRx();
  freq = settingf1;
  delay(200);

  while (true)
  {

// Проверяем команды из Serial
    if (stringComplete) {
      processSerialCommand(inputString);
      inputString = "";
      stringComplete = false;
    }

    btn();
    if (menu_mode == 2)
    {
      break;
    }

    settingf2 = settingf1 + 128 * st;

    // Сканируем все частоты и получаем RSSI
    for (int i = 0; i < 128; i++)
    {
      freq = settingf1 + i * st;
      ELECHOUSE_cc1101.setMHZ(freq);
      rssi = ELECHOUSE_cc1101.getRssi();
      rssiValues[i] = rssi;
    }

    // Передаем данные через Serial в формате: частота,RSSI
    Serial.print("SCAN:");
    for (int i = 0; i < 128; i++)
    {
      float currentFreq = settingf1 + i * st;
      Serial.print(currentFreq, 2);
      Serial.print(",");
      Serial.print(rssiValues[i]);
      if (i < 127) Serial.print(";");
    }
    Serial.println(); // Завершаем строку

    // Минимальная задержка для стабильности
    delay(10);
  }
}

void scan()
{
  // Serial.print(F("\r\nScanning frequency range from : "));
  //  Serial.print(settingf1);
  //  Serial.print(F(" MHz to "));
  //  Serial.print(settingf2);
  //  Serial.print(F(" MHz, press any key for stop or wait...\r\n"));
  // initialize parameters for scanning
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setRxBW(58);
  ELECHOUSE_cc1101.SetRx();
  // Do scanning until some key pressed
  freq = settingf1; // start frequency for scanning
  delay(200);

  while (true)
  {

// Проверяем команды из Serial
    if (stringComplete) {
      processSerialCommand(inputString);
      inputString = "";
      stringComplete = false;
    }

    btn();

    if (menu_mode == 2)
    {
      break;
    }

    settingf2 = settingf1 + 128 * st;

    for (int i = 0; i < 128; i++)
    {
      freq = settingf1 + i * st;
      ELECHOUSE_cc1101.setMHZ(freq);
      rssi = ELECHOUSE_cc1101.getRssi();
      rssiValues[i] = rssi;
    }

    // Сдвиг данных на 107 позиций вперед
    memmove(rssiHistorty + SHIFT_AMOUNT, rssiHistorty, (ARRAY_SIZE - SHIFT_AMOUNT) * sizeof(int8_t));
    // Обнуляем начальные 107 элементов, так как они теперь содержат старые данные
    memset(rssiHistorty, 0, SHIFT_AMOUNT * sizeof(int8_t));

    for (int i = 10; i < 118; i++)
    {
      rssiHistorty[i - 10] = map(rssiValues[i], -95, -1 * setVizFilt, 0, 1);
    }

    display.clearDisplay();
    DrawHistory();
    // float maxIndex = getMaxIndex();
    // if (maxIndex > 0)
    // {
    //   printIndex = maxIndex;
    // }

    display.setRotation(3);
    display.setCursor(25, 0);
    display.print(settingf1);
    // display.setCursor(30, 10);
    // display.print(" MHz");
    display.setCursor(25, 120);
    display.print(settingf2);
    display.setRotation(0);

    display.setCursor(30, 1);
    display.print(st);

    display.setCursor(75, 1);
    display.print(currentMhz);

    if (sel_mode == 1)
    {
      display.drawBitmap(0, 45, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 2)
    {
      display.drawBitmap(35, 10, sel2, 8, 3, SSD1306_WHITE);
    }
    if (sel_mode == 3)
    {
      display.drawBitmap(80, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 4)
    {
      display.drawBitmap(110, 50, sel2, 8, 3, SSD1306_WHITE);
    }

    if (sel_mode == 5)
    {
      display.drawBitmap(12, 10, sel2, 8, 3, SSD1306_WHITE);
    }

    for (int i = 0; i < arraySize; i++)
    {
      int bar_height = map(rssiValues[i], -95, -50, 0, 65);
      display.drawLine(i, SCREEN_HEIGHT - 1, i, SCREEN_HEIGHT - bar_height, SSD1306_WHITE);
    }

    display.drawLine(110, 45 - 95 + setVizFilt, 118, 45 - 95 + setVizFilt, SSD1306_WHITE);

    display.drawLine(dopMhz, 10, dopMhz, 10 - 10, SSD1306_WHITE);

    if (FreezeMode == 0)
    {
      display.drawBitmap(12, 1, cherep, 8, 8, SSD1306_WHITE);
    }
    else
    {
      display.drawBitmap(12, 1, cherepOn, 8, 8, SSD1306_WHITE);
    }

    display.display();

  }; // End of While
}

void setup()
{

  // initialize USB Serial Port CDC
  Serial.begin(115200);
  while (!Serial) {
   ; // wait for serial port to connect
  }

// Добавляем обработчик прерываний для приема данных
  Serial.onReceive([]() {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == '\n') {
        stringComplete = true;
      } else {
        inputString += inChar;
      }
    }
  });

  pinMode(BUTTON_PINUP, INPUT_PULLUP);
  pinMode(BUTTON_PINDOWN, INPUT_PULLUP);
  pinMode(BUTTON_PINLEFT, INPUT_PULLUP);
  pinMode(BUTTON_PINRIGHT, INPUT_PULLUP);

  Serial.println(F("CC1101 terminal tool connected, use 'help' for list of commands...\n\r"));
  Serial.println(F("(C) Adam Loboda 2023\n\r  "));

  Serial.println(); // print CRLF

  // Init EEPROM - for ESP32 based boards only
  EEPROM.begin(EPROMSIZE);

  // Инициализация дисплея SSD1306
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 не подключен!");
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 0);
  display.println("sdr v0.0.1");

  // display.drawBitmap(20, 20, sel, 8, 8, SSD1306_WHITE); //стрелка
  //  display.drawBitmap(40, 20, sel2, 8, 3, SSD1306_WHITE);

  display.display();

  // initialize CC1101 module with preffered parameters
  cc1101initialize();

  if (ELECHOUSE_cc1101.getCC1101())
  { // Check the CC1101 Spi connection.
    Serial.println(F("cc1101 initialized. Connection OK\n\r"));
  }
  else
  {
    Serial.println(F("cc1101 connection error! check the wiring.\n\r"));
  };

  currentMhz = settingf1 + dopMhz * st;
  mySwitch.enableReceive(RX_PIN);
  mySwitch.enableTransmit(TX_PIN);
}



void loop()
{

// Обработка команд из Serial
  if (stringComplete) {
    inputString.trim();
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  if (menu_mode == 1)
  {
    if (serialOutput)
     {
      scan_com();
     } else 
     {
       scan();
     }
  }

  if (menu_mode == 2)
  {
    dead();
  }

  if (menu_mode == 3)
  {
    Analyser();
  }

} // end of main LOOP

/*

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 8
#define OLED_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
   Serial.begin(115200);

  // Инициализация дисплея SSD1306
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 не подключен!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SSD1306 OK!");
  display.display();
  delay(2000);

   // Инициализация CC1101
  ELECHOUSE_cc1101.Init(F_433);  // set frequency - F_433, F_868, F_965 MHz
  ELECHOUSE_cc1101.SetReceive();

}
byte buffer[61] = {0};
void loop() {
  if (ELECHOUSE_cc1101.CheckReceiveFlag())
  {
    int len = ELECHOUSE_cc1101.ReceiveData(buffer);
    buffer[len] = '\0';
    Serial.println((char *) buffer);
    ELECHOUSE_cc1101.SetReceive();
  }


}

*/