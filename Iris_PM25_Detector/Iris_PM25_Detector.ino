/*********************************************************************
  Editor: WangYi
  E-Mail: puppywang@gmail.com
  Date:   2015-12-28
  Version: V1.0
  Description
   PM2.5 sensor code for Iris

*********************************************************************/

#include "U8glib.h"

#define joystickPin 0
#define dhtPin 2 //DHT PIN2
#define RES 6    //LED reset pin PIN6
#define DC 7     //LED DC pin PIN3
#define PM25SetPin 4 // PM2.5 Sensor RX PIN4

#define RedLedPin 9
#define GreenLedPin 10
#define BlueLedPin  3

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21
#define dhtPin 2
#define dhtType DHT11
#define dhtCount 6

unsigned long _lastreadtime;
boolean firstreading;
uint8_t data[6];
boolean joystickChange;
uint8_t draw_state = 0;
boolean show_color = true;

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

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
  int x = value / 100; //变量xyz被拆分并分为3个值分别显示
  int y = value % 100;
  int z = y % 10;
  y = y / 10;
  char s_x[2] = " ";
  char s_y[2] = " ";
  char s_z[2] = " ";
  s_x[0] = x + 48;
  s_y[0] = y + 48;
  s_z[0] = z + 48;

  u8g.setScale2x2();//大字体
  if (x > 0) { //消除第一位为0时的显示问题
    u8g.drawStr(20, 12, s_x);
    u8g.drawStr(30, 12, s_y);
    u8g.drawStr(40, 12, s_z);
  }
  if (x == 0) {
    if (y == 0) {
      u8g.drawStr(40, 12, s_z);
    }
    u8g.drawStr(30, 12, s_y);
    u8g.drawStr(40, 12, s_z);
  }
  u8g.undoScale();
}

//读取PMS1003的数据。并根据通信协议转化成有效的值。
void ProcessSerialData()
{
  uint8_t one_readout = 0;
  uint8_t i = 0;
  uint8_t readout_bit[32] = {0};
  int checksum = 0;
  while (Serial.available() > 0)
  {
    Serial.println("Begin Read PM2.5 Sensor...");
    // Base on the protocol of Plantower PMS1003
    one_readout = Serial.read();
    delay(2); // wait until packet is received
    if (one_readout == 0x42) // head1 ok
    {
      readout_bit[0] =  one_readout;
      one_readout = Serial.read();
      if (one_readout == 0x4d) // head2 ok
      {
        readout_bit[1] =  one_readout;
        checksum = 0x42 + 0x4d;
        for (int i = 2; i < 30; i++) //data recv and crc calc
        {
          readout_bit[i] = Serial.read();
          delay(2);
          checksum += readout_bit[i];
        }
        readout_bit[30] = Serial.read();
        delay(1);
        readout_bit[31] = Serial.read();
        Serial.println();
        Serial.print(checksum);
        Serial.print("  ");
        Serial.println(readout_bit[30] * 0xff + readout_bit[31]);
        if (true /*checksum == readout_bit[30] * 0xff + readout_bit[31]*/) //crc ok
        {
          Serial.println("Done PM2.5 Readout~");
          Serial.flush();
          pm1_value = readout_bit[10] * 0xff + readout_bit[11];
          pm25_value = readout_bit[12] * 0xff + readout_bit[13];
          pm10_value = readout_bit[14] * 0xff + readout_bit[15];
          pm1_cf_value = readout_bit[4] * 0xff + readout_bit[5];
          pm25_cf_value = readout_bit[6] * 0xff + readout_bit[7];
          pm10_cf_value = readout_bit[8] * 0xff + readout_bit[9];
          count_03_um = readout_bit[16] * 0xff + readout_bit[17];
          count_05_um = readout_bit[18] * 0xff + readout_bit[19];
          count_10_um = readout_bit[20] * 0xff + readout_bit[21];
          count_25_um = readout_bit[22] * 0xff + readout_bit[23];
          count_50_um = readout_bit[24] * 0xff + readout_bit[25];
          count_100_um = readout_bit[26] * 0xff + readout_bit[27];
          return;
        }
      }
    }
  }
}

void setup() {
  ledBegin();
  u8g.setColorIndex(1);             //displayMode : u8g_MODE_BW
  // Setup PM2.5 Sensor
  Serial.begin(9600);
  // SET PIN = 1, WORKING
  pinMode(PM25SetPin, OUTPUT);
  digitalWrite(PM25SetPin, HIGH);

  analogWrite(RedLedPin, 255);
  analogWrite(GreenLedPin, 255);
  analogWrite(BlueLedPin, 255);
}

void loop() {
  u8g.firstPage();
  ProcessSerialData();
  do {
    draw();
  }
  while (u8g.nextPage());

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
      analogWrite(BlueLedPin, 255);
    } else if (pm25_value < 160) {
      analogWrite(RedLedPin, 255);
      analogWrite(GreenLedPin, 250);
      analogWrite(BlueLedPin, 250);
    } else {
      analogWrite(RedLedPin, 250);
      analogWrite(GreenLedPin, 255);
      analogWrite(BlueLedPin, 255);
    }
  } else {
    analogWrite(RedLedPin, 255);
    analogWrite(GreenLedPin, 255);
    analogWrite(BlueLedPin, 255);
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

