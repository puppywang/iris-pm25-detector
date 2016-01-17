/*********************************************************************
  Editor: WangYi
  E-Mail: puppywang@gmail.com
  Date:   2015-12-28
  Version: V1.0
  Description
   PM2.5 sensor code for Iris

 ----------------------------------------------------------------------------- Nicola Coppola
 * Pin layout should be as follows:
 * Signal     Pin              Pin               Pin
 *            Arduino Uno      Arduino Mega      MFRC522 board
 * ------------------------------------------------------------
 * Reset      9                5                 RST
 * SPI SS     10               53                SDA
 * SPI MOSI   11               51                MOSI
 * SPI MISO   12               50                MISO
 * SPI SCK    13               52                SCK
- *
 *********************************************************************/

#include <U8glib.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>

#define joystickPin 0
#define dhtPin 2 //DHT PIN2
#define RES 6    //LED reset pin PIN6
#define DC 7     //LED DC pin PIN3
#define sensorSetPin 4 // PM2.5 Sensor RX PIN4

#define RedLedPin 9
#define GreenLedPin 10
//#define BlueLedPin  3

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21
#define dhtPin 2
#define dhtType DHT11
#define dhtCount 6

#define SS_PIN 5
#define RST_PIN 3

unsigned long _lastreadtime;
boolean firstreading;
uint8_t data[6];
boolean joystickChange;
uint8_t draw_state = 0;
boolean show_color = true;

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
SoftwareSerial sensorSerial(8, 14); // RX, TX
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance.

/*********************************************************************
  DHT Initialization
*********************************************************************/
void dhtBegin(void) {
  // set up the pins!
  pinMode(dhtPin, INPUT);
  digitalWrite(dhtPin, HIGH);
  _lastreadtime = 0;
  firstreading = true;
}

/*********************************************************************
  LED Screen Initialization
*********************************************************************/
void ledBegin(void) {
  pinMode(DC, OUTPUT);
  digitalWrite(DC, LOW);            //DC=0
  pinMode(RES, OUTPUT);
  digitalWrite(RES, HIGH);   delay(100);
  digitalWrite(RES, LOW);    delay(100);
  digitalWrite(RES, HIGH);   delay(100);
}

/**
* Helper to print MFRC522 module info
*/
void ShowReaderVersion() {
  // Get the MFRC522 firmware version
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.print(F("Firmware Version: 0x"));
  Serial.print(v, HEX);
  if (v == 0x88)
    Serial.print(F(" = (clone)"));
  else if (v == 0x90)
    Serial.print(F(" = v0.0"));
  else if (v == 0x91)
    Serial.print(F(" = v1.0"));
  else if (v == 0x92)
    Serial.print(F(" = v2.0"));
  else
    Serial.print(F(" = (unknown)"));
  Serial.println();
  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF))
    Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
}

void rfidBegin(void) {
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522 card
  
  Serial.println(F("*****************************"));
  Serial.println(F("MFRC522 Digital self test"));
  Serial.println(F("*****************************"));
  ShowReaderVersion();  // Show version of PCD - MFRC522 Card Reader
  Serial.println(F("Performing test..."));
  bool result = mfrc522.PCD_PerformSelfTest();
  Serial.println(F("-----------------------------"));
  Serial.print(F("Result: "));
  if (result)
    Serial.println(F("OK"));
  else
    Serial.println(F("DEFECT or UNKNOWN"));
  Serial.println();

  mfrc522.PCD_Init(); // Init MFRC522 card
  Serial.println("Scan PICC to see UID and type...");
}

void pm25SensorBegin(void) {
  // Setup PM2.5 Sensor
  sensorSerial.begin(9600);

  // SET PIN = 1, WORKING
  pinMode(sensorSetPin, OUTPUT);
  digitalWrite(sensorSetPin, HIGH);
}

/*********************************************************************
  private
  DHT read date
*********************************************************************/
boolean dhtRead(void) {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;

  digitalWrite(dhtPin, HIGH);

  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // ie there was a rollover
    _lastreadtime = 0;
  }
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
    return true; // return last correct measurement
  }
  firstreading = false;
  /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
  _lastreadtime = millis();

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // now pull it low for ~20 milliseconds
  pinMode(dhtPin, OUTPUT);
  digitalWrite(dhtPin, LOW);
  delay(20);
  cli();
  digitalWrite(dhtPin, HIGH);
  delayMicroseconds(40);
  pinMode(dhtPin, INPUT);

  // read in timings
  for ( i = 0; i < MAXTIMINGS; i++) {
    counter = 0;
    while (digitalRead(dhtPin) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(dhtPin);
    if (counter == 255) break;
    // ignore first 3 transitions
    if ((i >= 4) && (i % 2 == 0)) {
      // shove each bit into the storage bytes
      data[j / 8] <<= 1;
      if (counter > dhtCount)
        data[j / 8] |= 1;
      j++;
    }
  }
  sei();
  // check we read 40 bits and that the checksum matches
  if ((j >= 40) &&
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }
  return false;
}

/*********************************************************************
  Return the value of Temperature
  boolean S : Scale.  True == Farenheit; False == Celcius
*********************************************************************/
float readTemperature() {
  float f;

  if (dhtRead()) {
    switch (dhtType) {
      case DHT11:
        f = data[2];

        return f;
      case DHT22:
      case DHT21:
        f = data[2] & 0x7F;
        f *= 0xff;
        f += data[3];
        f /= 10;
        if (data[2] & 0x80) {
          f *= -1;
        }

        return f;
    }
  }
  Serial.print("Read fail");
  return NAN;
}

/*********************************************************************
  Return the value of Humidity
  value: 0-100%
*********************************************************************/
float readHumidity(void) {
  float f;
  if (dhtRead()) {
    switch (dhtType) {
      case DHT11:
        f = data[0];
        return f;
      case DHT22:
      case DHT21:
        f = data[0];
        f *= 0xff;
        f += data[1];
        f /= 10;
        return f;
    }
  }
  Serial.print("Read fail");
  return NAN;
}

/*********************************************************************
  Return the state of Joystick
  value 1: right
      2: up
      3: left
      4: down
      5: push
*********************************************************************/
int readJoystick(void) {
  int key_value = 0;
  int stickRaw = analogRead(joystickPin);
  static int keyValueLast = 0;
  if (stickRaw < 70)
    key_value = 1; //right
  else if (70 <= stickRaw && stickRaw < 235)
    key_value = 5; //push
  else if (235 <= stickRaw && stickRaw < 420)
    key_value = 2; //up
  else if (420 <= stickRaw && stickRaw < 620)
    key_value = 3; //left
  else if (620 <= stickRaw && stickRaw < 900)
    key_value = 4; //dowm
  else
    key_value = 0;
  if (keyValueLast != key_value) {
    joystickChange = true;
    keyValueLast = key_value;
  }
  return key_value;
}

void u8g_prepare(void) {
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
}

int pm25_value = 0;
int pm25_cf_value = 0;
int pm1_value = 0;
int pm1_cf_value = 0;
int pm10_value = 0;
int pm10_cf_value = 0;
int count_03_um = 0;
int count_05_um = 0;
int count_10_um = 0;
int count_25_um = 0;
int count_50_um = 0;
int count_100_um = 0;


void u8g_drawxyz(int value) {
  // 鍙橀噺xyz琚媶鍒嗗苟鍒嗕负3涓�煎垎鍒樉绀�
  int v = value / 10000;
  value -= v * 10000;
  int w = value / 1000;
  value -= w * 1000;
  int x = value / 100;
  value -= x * 100; 
  int y = value / 10;
  value -= y * 10;
  int z = value;

  char s_v[2] = " ";
  char s_w[2] = " ";
  char s_x[2] = " ";
  char s_y[2] = " ";
  char s_z[2] = " ";

  s_v[0] = v + 48;
  s_w[0] = w + 48;
  s_x[0] = x + 48;
  s_y[0] = y + 48;
  s_z[0] = z + 48;

  u8g.setScale2x2();//大字体
  boolean first_digit = true;
  if (v > 0) {
    u8g.drawStr(10, 12, s_v);
    first_digit = false;
  }
  if (!first_digit || w > 0) {
    u8g.drawStr(20, 12, s_w);
    first_digit = false;
  }
  if (!first_digit || x > 0) {
    u8g.drawStr(30, 12, s_x);
    first_digit = false;
  }
  if (!first_digit || y > 0) {
    u8g.drawStr(40, 12, s_y);
    first_digit = false;
  }
  if (!first_digit || z > 0) {
    u8g.drawStr(50, 12, s_z);
    first_digit = false;
  }
  u8g.undoScale();
}

//读取PMS1003的数据。并根据通信协议转化成有效的值。
void processSensorData()
{
  uint8_t one_readout = 0;
  uint8_t i = 0;
  uint8_t readout_bit[32] = {0};
  int checksum = 0;
  
  while (sensorSerial.available() > 0)
  {
/*
    Serial.println("Begin Read PM2.5 Sensor...");
*/
    // Base on the protocol of Plantower PMS1003
    one_readout = sensorSerial.read();
    if (one_readout == 0x42) // head1 ok
    {
      readout_bit[0] = one_readout;
      one_readout = sensorSerial.read();
      if (one_readout == 0x4d) // head2 ok
      {
        readout_bit[1] = one_readout;
        checksum = 0x42 + 0x4d;
        for (int i = 2; i < 30; i++) //data recv and crc calc
        {
          readout_bit[i] = sensorSerial.read();
          checksum += readout_bit[i];
        }
        readout_bit[30] = sensorSerial.read();
        readout_bit[31] = sensorSerial.read();
/*
        Serial.println();
        Serial.print(checksum);
        Serial.print("  ");
        Serial.println(readout_bit[30] * 0x100 + readout_bit[31]);
*/
        if (checksum == readout_bit[30] * 0x100 + readout_bit[31]) //crc ok
        {
/*
          Serial.println("Done PM2.5 Readout~");
          Serial.flush();
*/
          pm1_value = readout_bit[10] * 256 + readout_bit[11];
          pm25_value = readout_bit[12] * 256 + readout_bit[13];
          pm10_value = readout_bit[14] * 256 + readout_bit[15];
          pm1_cf_value = readout_bit[4] * 256 + readout_bit[5];
          pm25_cf_value = readout_bit[6] * 256 + readout_bit[7];
          pm10_cf_value = readout_bit[8] * 256 + readout_bit[9];
          count_03_um = readout_bit[16] * 256 + readout_bit[17];
          count_05_um = readout_bit[18] * 256 + readout_bit[19];
          count_10_um = readout_bit[20] * 256 + readout_bit[21];
          count_25_um = readout_bit[22] * 0xff + readout_bit[23];
          count_50_um = readout_bit[24] * 256 + readout_bit[25];
          count_100_um = readout_bit[26] * 256 + readout_bit[27];
          return;
        }
      }
    }
  }
}

void hexDump(byte* buf, int len) {
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10) {
      Serial.print(0);
    }
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.print(F("(Cnt = "));
  Serial.print(len);
  Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  
  ledBegin();
  dhtBegin();
  rfidBegin();
  pm25SensorBegin();
  u8g.setColorIndex(1);             //displayMode : u8g_MODE_BW
/*
  Serial.print("TxModeReg:");
  Serial.println(mfrc522.PCD_ReadRegister(MFRC522::TxModeReg));
  Serial.print("RxModeReg:");
  Serial.println(mfrc522.PCD_ReadRegister(MFRC522::RxModeReg));
*/
  Serial.print("PCD_GetAntennaGain:");
  Serial.println(mfrc522.PCD_GetAntennaGain());
  mfrc522.PCD_SetAntennaGain(112);
  Serial.print("New PCD_GetAntennaGain:");
  Serial.println(mfrc522.PCD_GetAntennaGain());

  // RGBLedBegin
  analogWrite(RedLedPin, 255);
  analogWrite(GreenLedPin, 255);
//  analogWrite(BlueLedPin, 255);
}

void loop() {
  u8g.firstPage();
  processSensorData();
  do {
    draw();
  }
  while (u8g.nextPage());
/*
  byte bufferATQA[2] = {0};
  byte bufferATQASize = sizeof(bufferATQA);
  byte result = mfrc522.PICC_RequestA(bufferATQA, &bufferATQASize);
  Serial.print("PICC_RequestA:");
  Serial.println(result);
  hexDump(bufferATQA, bufferATQASize);
  if (result == MFRC522::STATUS_OK) {
    byte bufferSelectAll[10] = {0};
    byte bufferSelectAllSize = sizeof(bufferSelectAll);
    byte selectCmd[] = { 0x93, 0x20 };
    result = mfrc522.PCD_TransceiveData(selectCmd, sizeof(selectCmd), bufferSelectAll, &bufferSelectAllSize);
    if (bufferSelectAllSize == 5 && (bufferSelectAll[0] ^ bufferSelectAll[1] ^ bufferSelectAll[2] ^ bufferSelectAll[3] == bufferSelectAll[4])) {
      Serial.print("PICC_Select:");
      Serial.println(result);
      hexDump(bufferSelectAll, bufferSelectAllSize);
      if (result == MFRC522::STATUS_OK) {
        byte bufferSelectCard[10] = {0};
        byte bufferSelectCardSize = sizeof(bufferSelectCard);
        byte selectCardCmd[9] = { 0x93, 0x70 };
        memcpy(selectCardCmd + 2, bufferSelectAll, 5);
        mfrc522.PCD_CalculateCRC(selectCardCmd, 7, selectCardCmd + 7);
        Serial.println("");
        result = mfrc522.PCD_TransceiveData(selectCardCmd, sizeof(selectCardCmd), bufferSelectCard, &bufferSelectCardSize);
        Serial.print("PICC_SelectCard:");
        Serial.println(result);
        hexDump(bufferSelectCard, bufferSelectCardSize);
        if (result == MFRC522::STATUS_OK) {
          mfrc522.PICC_HaltA();
        }
      }
    }
  }
*/
  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      // Dump debug info about the card. PICC_HaltA() is automatically called.
      mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
    }
  }

  // rebuild the picture after some delay
  delay(150);
  int joy_stick = readJoystick();
  // Next state
  if (joy_stick == 1 || joy_stick == 4) {
    draw_state++;
  } else if (joy_stick == 2 || joy_stick == 3) {
    draw_state--;
    if (draw_state < 0) {
      draw_state += 14;
    }
  } else if (joy_stick == 5) {
    show_color = !show_color;
  }
  draw_state = draw_state % 14;
  if (show_color) {
    if (pm25_value < 80) {
      analogWrite(RedLedPin, 255);
      analogWrite(GreenLedPin, 250);
//      analogWrite(BlueLedPin, 255);
    } else if (pm25_value < 160) {
      analogWrite(RedLedPin, 250);
      analogWrite(GreenLedPin, 250);
//      analogWrite(BlueLedPin, 255);
    } else {
      analogWrite(RedLedPin, 250);
      analogWrite(GreenLedPin, 255);
//      analogWrite(BlueLedPin, 255);
    }
  } else {
    analogWrite(RedLedPin, 255);
    analogWrite(GreenLedPin, 255);
//    analogWrite(BlueLedPin, 255);
  }
}

void draw (void) {
  u8g_prepare();
  switch (draw_state) {
    case 0: {
        u8g.drawStr(5, 5, "pm2.5=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm25_value);
        break;
      }
    case 1: {
        u8g.drawStr(5, 5, "pm1=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm1_value);
        break;
      }
    case 2: {
        u8g.drawStr(5, 5, "pm10=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm10_value);
        break;
      }
    case 3: {
        u8g.drawStr(5, 5, "pm2.5_cf=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm25_cf_value);
        break;
      }
    case 4: {
        u8g.drawStr(5, 5, "pm1_cf=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm1_cf_value);
        break;
      }
    case 5: {
        u8g.drawStr(5, 5, "pm10_cf=");
        u8g.drawStr(90, 55, "ug/m3");
        u8g_drawxyz(pm10_cf_value);
        break;
      }
    case 6: {
        u8g.drawStr(5, 5, ">0.3um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_03_um);
        break;
      }
    case 7: {
        u8g.drawStr(5, 5, ">0.5um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_05_um);
        break;
      }
    case 8: {
        u8g.drawStr(5, 5, ">1.0um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_10_um);
        break;
      }
    case 9: {
        u8g.drawStr(5, 5, ">2.5um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_25_um);
        break;
      }
    case 10: {
        u8g.drawStr(5, 5, ">5.0um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_50_um);
        break;
      }
    case 11: {
        u8g.drawStr(5, 5, ">10.0um");
        u8g.drawStr(60, 55, "count/100ml");
        u8g_drawxyz(count_100_um);
        break;
      }
    case 12: {
        u8g.drawStr(5, 5, "Temp=");
        u8g.drawStr(120, 55, "C");
        u8g_drawxyz(readTemperature());
        break;
      }
    case 13: {
        u8g.drawStr(5, 5, "Humi=");
        u8g.drawStr(120, 55, "%");
        u8g_drawxyz(readHumidity());
        break;
      }
  }
}


