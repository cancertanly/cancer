/*
  Энергомонитор для Arduino Nano (ATmega328P)
  - PZEM-004T v1.0 через SoftwareSerial (RX=2, TX=3)
  - TFT ST7735S 128x160 (CS=10, DC=8, RST=9)
  - DS1307 RTC по I2C
  - MicroSD (CS=4) через SdFat (легче SD.h)
  - Меню на 4 кнопках (A0..A3, INPUT_PULLUP)

  Важное по RAM:
  - Без String и динамической памяти
  - Текстовые константы в PROGMEM/F()
  - Небольшие фиксированные буферы
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <avr/pgmspace.h>

// -------------------- Пины --------------------
static const uint8_t PIN_PZEM_RX = 2;
static const uint8_t PIN_PZEM_TX = 3;

static const uint8_t PIN_TFT_CS  = 10;
static const uint8_t PIN_TFT_DC  = 8;
static const uint8_t PIN_TFT_RST = 9;

static const uint8_t PIN_SD_CS   = 4;

static const uint8_t PIN_BTN_UP   = A0;
static const uint8_t PIN_BTN_DOWN = A1;
static const uint8_t PIN_BTN_SEL  = A2;
static const uint8_t PIN_BTN_BACK = A3;

// -------------------- Тайминги --------------------
static const uint16_t PERIOD_PZEM_MS   = 1100;
static const uint16_t PERIOD_TFT_MS    = 350;
static const uint16_t PERIOD_LOG_MS    = 10000;
static const uint16_t PERIOD_CLOCK_MS  = 500;
static const uint16_t PERIOD_SD_RETRY  = 3000;
static const uint16_t BTN_DEBOUNCE_MS  = 40;

// -------------------- Цвета RGB565 --------------------
static const uint16_t C_BLACK  = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_CYAN   = 0x07FF;
static const uint16_t C_RED    = 0xF800;

SoftwareSerial pzem(PIN_PZEM_RX, PIN_PZEM_TX);
SdFat sd;
FsFile logFile;

struct Measurements {
  uint16_t voltage_x10;  // 2301 => 230.1V
  uint16_t current_x100; // 123 => 1.23A
  uint16_t power;        // W
  uint32_t energy;       // Wh
  bool valid;
} m;

struct DateTimeS {
  uint8_t sec, min, hour, day, mon;
  uint16_t year;
} now;

enum MenuState : uint8_t {
  MENU_NONE = 0,
  MENU_ROOT,
  MENU_SET_HOUR,
  MENU_SET_MIN,
  MENU_SET_DAY,
  MENU_SET_MON,
  MENU_SET_YEAR,
  MENU_RESET_ENERGY
};

MenuState menuState = MENU_NONE;

// -------------------- Мини-шрифт 5x7 --------------------
// Символы 32..95 (нужно для цифр/латиницы/знаков)
static const uint8_t font5x7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
  0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
  0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x7F,0x20,0x18,0x20,0x7F,
  0x63,0x14,0x08,0x14,0x63, 0x03,0x04,0x78,0x04,0x03, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x80,0x80,0x80,0x80,0x80
};

// -------------------- Низкоуровневый TFT --------------------
inline void tftSelect(bool en){ digitalWrite(PIN_TFT_CS, en ? LOW : HIGH); }
inline void tftDC(bool data){ digitalWrite(PIN_TFT_DC, data ? HIGH : LOW); }

void tftWriteCmd(uint8_t c){
  tftSelect(true); tftDC(false); SPI.transfer(c); tftSelect(false);
}
void tftWriteData(uint8_t d){
  tftSelect(true); tftDC(true); SPI.transfer(d); tftSelect(false);
}

void tftSetAddr(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1){
  tftWriteCmd(0x2A); tftWriteData(0); tftWriteData(x0); tftWriteData(0); tftWriteData(x1);
  tftWriteCmd(0x2B); tftWriteData(0); tftWriteData(y0); tftWriteData(0); tftWriteData(y1);
  tftWriteCmd(0x2C);
}

void tftFillRect(uint8_t x,uint8_t y,uint8_t w,uint8_t h,uint16_t color){
  tftSetAddr(x,y,x+w-1,y+h-1);
  tftSelect(true); tftDC(true);
  uint8_t hi = color >> 8, lo = color & 0xFF;
  uint16_t n = (uint16_t)w * h;
  while(n--){ SPI.transfer(hi); SPI.transfer(lo); }
  tftSelect(false);
}

void tftClear(uint16_t color){ tftFillRect(0,0,128,160,color); }

void tftDrawChar(uint8_t x,uint8_t y,char c,uint16_t fg,uint16_t bg,uint8_t scale){
  if(c < 32 || c > 95) c = '?';
  uint16_t idx = (uint16_t)(c - 32) * 5;
  for(uint8_t col=0; col<5; col++){
    uint8_t bits = pgm_read_byte(&font5x7[idx + col]);
    for(uint8_t row=0; row<8; row++){
      uint16_t color = (bits & 0x01) ? fg : bg;
      if(scale == 1) tftFillRect(x + col, y + row, 1, 1, color);
      else tftFillRect(x + col*scale, y + row*scale, scale, scale, color);
      bits >>= 1;
    }
  }
  if(scale == 1) tftFillRect(x+5,y,1,8,bg);
  else tftFillRect(x+5*scale,y,scale,8*scale,bg);
}

void tftPrint(uint8_t x,uint8_t y,const char* s,uint16_t fg,uint16_t bg,uint8_t scale){
  while(*s){
    tftDrawChar(x,y,*s++,fg,bg,scale);
    x += 6*scale;
    if(x > (128 - 6*scale)) break;
  }
}

void tftInit(){
  pinMode(PIN_TFT_CS, OUTPUT); pinMode(PIN_TFT_DC, OUTPUT); pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);

  digitalWrite(PIN_TFT_RST, LOW); delay(5);
  digitalWrite(PIN_TFT_RST, HIGH); delay(120);

  tftWriteCmd(0x11); delay(120);
  tftWriteCmd(0x3A); tftWriteData(0x05); // RGB565
  tftWriteCmd(0x36); tftWriteData(0xC8); // ориентация
  tftWriteCmd(0x29);
  tftClear(C_BLACK);
}

// -------------------- RTC DS1307 --------------------
uint8_t bcd2bin(uint8_t b){ return (b >> 4) * 10 + (b & 0x0F); }
uint8_t bin2bcd(uint8_t b){ return ((b / 10) << 4) | (b % 10); }

bool rtcRead(DateTimeS &dt){
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0);
  if(Wire.endTransmission() != 0) return false;
  if(Wire.requestFrom(0x68, 7) != 7) return false;

  dt.sec  = bcd2bin(Wire.read() & 0x7F);
  dt.min  = bcd2bin(Wire.read());
  dt.hour = bcd2bin(Wire.read() & 0x3F);
  Wire.read(); // день недели
  dt.day  = bcd2bin(Wire.read());
  dt.mon  = bcd2bin(Wire.read());
  dt.year = 2000 + bcd2bin(Wire.read());
  return true;
}

bool rtcWrite(const DateTimeS &dt){
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0);
  Wire.write(bin2bcd(dt.sec));
  Wire.write(bin2bcd(dt.min));
  Wire.write(bin2bcd(dt.hour));
  Wire.write((uint8_t)1);
  Wire.write(bin2bcd(dt.day));
  Wire.write(bin2bcd(dt.mon));
  Wire.write(bin2bcd((uint8_t)(dt.year - 2000)));
  return Wire.endTransmission() == 0;
}

// -------------------- PZEM v1.0 --------------------
// Мини-реализация запроса: команды B0..B3, CRC = сумма байтов
uint8_t pzemCrc(const uint8_t* b, uint8_t len){
  uint16_t s = 0;
  for(uint8_t i=0;i<len;i++) s += b[i];
  return (uint8_t)(s & 0xFF);
}

bool pzemQuery(uint8_t cmd, uint32_t &value){
  uint8_t req[7] = {cmd, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x00}; // addr 192.168.1.1
  req[6] = pzemCrc(req, 6);

  while(pzem.available()) pzem.read();
  pzem.write(req, 7);

  uint8_t resp[7];
  uint8_t i = 0;
  uint32_t t0 = millis();
  while(i < 7 && (millis() - t0) < 120){
    if(pzem.available()) resp[i++] = pzem.read();
  }
  if(i != 7) return false;
  if(resp[6] != pzemCrc(resp, 6)) return false;

  value = ((uint32_t)resp[1] << 16) | ((uint16_t)resp[2] << 8) | resp[3];
  return true;
}

void pzemPollNonBlocking(){
  static uint8_t stage = 0;
  uint32_t raw;
  bool ok = false;

  switch(stage){
    case 0: ok = pzemQuery(0xB0, raw); if(ok) m.voltage_x10 = raw; break;
    case 1: ok = pzemQuery(0xB1, raw); if(ok) m.current_x100 = raw; break;
    case 2: ok = pzemQuery(0xB2, raw); if(ok) m.power = raw; break;
    case 3: ok = pzemQuery(0xB3, raw); if(ok){ m.energy = raw; m.valid = true; } break;
  }

  stage = (stage + 1) & 0x03;
}

void pzemResetEnergy(){
  uint8_t req[7] = {0x42, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x00};
  req[6] = pzemCrc(req, 6);
  pzem.write(req, 7);
}

// -------------------- SD логгер --------------------
bool sdReady = false;
uint32_t lastSdTry = 0;

void buildCsvName(const DateTimeS& dt, char* out){
  // DDMMYY.CSV
  out[0] = '0' + dt.day / 10;
  out[1] = '0' + dt.day % 10;
  out[2] = '0' + dt.mon / 10;
  out[3] = '0' + dt.mon % 10;
  uint8_t y = dt.year % 100;
  out[4] = '0' + y / 10;
  out[5] = '0' + y % 10;
  out[6] = '.';
  out[7] = 'C'; out[8] = 'S'; out[9] = 'V';
  out[10] = '\0';
}

void sdTryInit(){
  if(sdReady) return;
  if(millis() - lastSdTry < PERIOD_SD_RETRY) return;
  lastSdTry = millis();
  sdReady = sd.begin(PIN_SD_CS, SD_SCK_MHZ(8));
}

bool sdWriteLogLine(){
  if(!sdReady || !m.valid) return false;
  char fn[11];
  buildCsvName(now, fn);

  if(!logFile.open(fn, O_RDWR | O_CREAT | O_AT_END)){
    sdReady = false;
    return false;
  }

  if(logFile.size() == 0){
    logFile.println(F("Date;Time;V;A;W;Wh"));
  }

  char line[48];
  uint8_t y = now.year % 100;
  // DD.MM.YY;HH:MM:SS;230.1;1.23;123;456
  snprintf(line, sizeof(line),
           "%02u.%02u.%02u;%02u:%02u:%02u;%u.%u;%u.%02u;%u;%lu",
           now.day, now.mon, y, now.hour, now.min, now.sec,
           m.voltage_x10 / 10, m.voltage_x10 % 10,
           m.current_x100 / 100, m.current_x100 % 100,
           m.power, m.energy);

  bool ok = logFile.println(line);
  logFile.close();
  if(!ok) sdReady = false;
  return ok;
}

// -------------------- Кнопки/меню --------------------
enum Btn : uint8_t { BTN_NONE=0, BTN_UP, BTN_DOWN, BTN_SEL, BTN_BACK };

Btn readButtons(){
  static uint32_t tDeb = 0;
  static Btn last = BTN_NONE;
  Btn cur = BTN_NONE;
  if(!digitalRead(PIN_BTN_UP)) cur = BTN_UP;
  else if(!digitalRead(PIN_BTN_DOWN)) cur = BTN_DOWN;
  else if(!digitalRead(PIN_BTN_SEL)) cur = BTN_SEL;
  else if(!digitalRead(PIN_BTN_BACK)) cur = BTN_BACK;

  if(cur != last){ tDeb = millis(); last = cur; return BTN_NONE; }
  if(cur != BTN_NONE && (millis() - tDeb) > BTN_DEBOUNCE_MS) return cur;
  return BTN_NONE;
}

void clampDateTime(DateTimeS& dt){
  if(dt.hour > 23) dt.hour = 0;
  if(dt.min > 59) dt.min = 0;
  if(dt.day < 1) dt.day = 1; if(dt.day > 31) dt.day = 31;
  if(dt.mon < 1) dt.mon = 1; if(dt.mon > 12) dt.mon = 12;
  if(dt.year < 2024) dt.year = 2024; if(dt.year > 2099) dt.year = 2099;
}

void menuHandle(){
  Btn b = readButtons();
  if(b == BTN_NONE) return;

  if(menuState == MENU_NONE){
    if(b == BTN_SEL) menuState = MENU_ROOT;
    return;
  }

  if(menuState == MENU_ROOT){
    if(b == BTN_UP) menuState = MENU_SET_HOUR;
    else if(b == BTN_DOWN) menuState = MENU_RESET_ENERGY;
    else if(b == BTN_BACK) menuState = MENU_NONE;
    else if(b == BTN_SEL) menuState = MENU_SET_HOUR;
    return;
  }

  if(menuState == MENU_RESET_ENERGY){
    if(b == BTN_SEL){ pzemResetEnergy(); m.energy = 0; }
    if(b == BTN_BACK) menuState = MENU_ROOT;
    return;
  }

  // Редактирование времени
  DateTimeS t = now;
  if(menuState == MENU_SET_HOUR){
    if(b == BTN_UP) t.hour = (t.hour + 1) % 24;
    else if(b == BTN_DOWN) t.hour = (t.hour + 23) % 24;
    else if(b == BTN_SEL) menuState = MENU_SET_MIN;
    else if(b == BTN_BACK) menuState = MENU_ROOT;
  } else if(menuState == MENU_SET_MIN){
    if(b == BTN_UP) t.min = (t.min + 1) % 60;
    else if(b == BTN_DOWN) t.min = (t.min + 59) % 60;
    else if(b == BTN_SEL) menuState = MENU_SET_DAY;
    else if(b == BTN_BACK) menuState = MENU_SET_HOUR;
  } else if(menuState == MENU_SET_DAY){
    if(b == BTN_UP && t.day < 31) t.day++;
    else if(b == BTN_DOWN && t.day > 1) t.day--;
    else if(b == BTN_SEL) menuState = MENU_SET_MON;
    else if(b == BTN_BACK) menuState = MENU_SET_MIN;
  } else if(menuState == MENU_SET_MON){
    if(b == BTN_UP && t.mon < 12) t.mon++;
    else if(b == BTN_DOWN && t.mon > 1) t.mon--;
    else if(b == BTN_SEL) menuState = MENU_SET_YEAR;
    else if(b == BTN_BACK) menuState = MENU_SET_DAY;
  } else if(menuState == MENU_SET_YEAR){
    if(b == BTN_UP && t.year < 2099) t.year++;
    else if(b == BTN_DOWN && t.year > 2024) t.year--;
    else if(b == BTN_SEL){ clampDateTime(t); rtcWrite(t); menuState = MENU_ROOT; }
    else if(b == BTN_BACK) menuState = MENU_SET_MON;
  }

  clampDateTime(t);
  now = t;
}

// -------------------- Рендер --------------------
void fmtVoltage(char* out){
  // "230.1V"
  out[0] = '0' + (m.voltage_x10 / 1000) % 10;
  out[1] = '0' + (m.voltage_x10 / 100) % 10;
  out[2] = '0' + (m.voltage_x10 / 10) % 10;
  out[3] = '.';
  out[4] = '0' + (m.voltage_x10 % 10);
  out[5] = 'V';
  out[6] = 0;
}

void drawMainScreen(){
  char buf[20];
  tftClear(C_BLACK);

  // Дата/время
  snprintf(buf, sizeof(buf), "%02u.%02u.%02u %02u:%02u:%02u",
           now.day, now.mon, (uint8_t)(now.year%100), now.hour, now.min, now.sec);
  tftPrint(2, 2, buf, C_CYAN, C_BLACK, 1);

  // Напряжение крупно в центре
  char vbuf[8];
  fmtVoltage(vbuf);
  tftPrint(14, 52, vbuf, C_YELLOW, C_BLACK, 3);

  // Низ: ток/мощность/энергия/SD
  snprintf(buf, sizeof(buf), "I:%u.%02uA", m.current_x100/100, m.current_x100%100);
  tftPrint(2, 120, buf, C_WHITE, C_BLACK, 1);
  snprintf(buf, sizeof(buf), "P:%uW", m.power);
  tftPrint(2, 132, buf, C_WHITE, C_BLACK, 1);
  snprintf(buf, sizeof(buf), "E:%luWh", m.energy);
  tftPrint(2, 144, buf, C_GREEN, C_BLACK, 1);
  tftPrint(84, 144, sdReady ? "SD:OK" : "SD:ERR", sdReady ? C_GREEN : C_RED, C_BLACK, 1);
}

void drawMenuOverlay(){
  tftFillRect(0, 96, 128, 22, 0x2104);
  if(menuState == MENU_ROOT) tftPrint(2, 100, "UP:TIME DN:RESET", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_SET_HOUR) tftPrint(2, 100, "SET HOUR", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_SET_MIN) tftPrint(2, 100, "SET MIN", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_SET_DAY) tftPrint(2, 100, "SET DAY", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_SET_MON) tftPrint(2, 100, "SET MON", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_SET_YEAR) tftPrint(2, 100, "SET YEAR SEL=SAVE", C_WHITE, 0x2104, 1);
  else if(menuState == MENU_RESET_ENERGY) tftPrint(2, 100, "SEL:RESET ENERGY", C_WHITE, 0x2104, 1);
}

// -------------------- Setup/Loop --------------------
void setup(){
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_SEL, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);

  SPI.begin();
  Wire.begin();
  pzem.begin(9600);

  tftInit();
  rtcRead(now);
  sdTryInit();

  m.valid = false;
  m.voltage_x10 = 0;
  m.current_x100 = 0;
  m.power = 0;
  m.energy = 0;
}

void loop(){
  static uint32_t tPzem=0, tTft=0, tLog=0, tRtc=0;
  uint32_t ms = millis();

  if(ms - tRtc >= PERIOD_CLOCK_MS){
    tRtc = ms;
    rtcRead(now);
  }

  if(ms - tPzem >= PERIOD_PZEM_MS){
    tPzem = ms;
    pzemPollNonBlocking();
  }

  if(!sdReady) sdTryInit();

  if(ms - tLog >= PERIOD_LOG_MS){
    tLog = ms;
    sdWriteLogLine();
  }

  menuHandle();

  if(ms - tTft >= PERIOD_TFT_MS){
    tTft = ms;
    drawMainScreen();
    if(menuState != MENU_NONE) drawMenuOverlay();
  }
}
