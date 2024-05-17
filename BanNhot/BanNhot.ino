#include "lib/CRC16.h"
#include <Adafruit_NeoPixel.h>

//-----------SENSOR------------//
#define numSen 4
#define SENSOR0 12
#define SENSOR1 13
#define SENSOR2 14
#define SENSOR3 21
int SENSOR[numSen] = {SENSOR0, SENSOR1, SENSOR2, SENSOR3};

//-----------LED------------//
#define numLED 3
#define LED1 40 // LED1 --> PIXEL1
#define LED2 41 // LED2 --> PIXEL2
#define LED0 42 // LED0 --> PIXEL3
int LED[numLED] = {LED0, LED1, LED2};

//-----------LED, BUZZER ON BOARD------------//
#define BUZZER 1
#define LED_BLINK 2

bool led_status = 0;
bool buz_status = 0;

//-----------Trạng thái cảm biến------------//
bool sensor_state[numSen] = {0, 0, 0, 0};
bool lastSensorState[numSen] = {0, 0, 0, 0};
bool temp_state[numSen] = {0, 0, 0, 0};
bool sensor_change[numSen] = {0, 0, 0, 0};

uint32_t lastDebounceTime[numSen] = {0, 0, 0, 0};

uint16_t count = 0;
uint16_t count1 = 0;

uint32_t ui32_time_blink = 0;
uint32_t ui32_timecho_guitrangthai;

uint32_t ui32_currentTime = 0;
uint32_t ui32_timecho_guilenh;
uint32_t ui32_timedoccambien = 0;
uint32_t ui32_timechar = 0;
uint32_t timeout_buz = 0;

uint16_t ui16_order_command;
uint16_t ui16_last_order_command = 0xFFFF;
uint16_t ui16_self_order_command = 0x0001;

uint8_t function_code = 0x07;

uint8_t ui8_phanhoi_saiCRC;
uint8_t ui8_phanhoi_trunglenh;

//-----------Serial------------//
uint8_t ui8_Byteindex = 0;
uint8_t ui8_data_send[15];
uint8_t ui8_data_rec[100];
uint8_t ui8_status = 0x00;
uint8_t ui8_function = 0x00;
uint8_t ui8_slaveID = 0x04;

bool ui2_BTcomplete = false;
bool ui2_BTreceive = false;
bool ui2_start = false;

uint8_t ui8_phanhoi_saiFunction;

//-----------Trạng thái thùng nhớt------------//
#define NO_STATE 255 // Không có trạng thái
enum TrangThaiThungNhot
{
  CAN = 0,      // Trạng thái bình chứa nhớt cạn
  DAY,          // Trạng thái bình chứa nhớt đầy
  BINHTHUONG,   // Trạng thái bình thường (trong thùng có nhớt)
  DANGHOATDONG, // Trạng thái thùng đang hoạt động
};

enum TrangThaiVoiBanNhot
{
  NGUNGBAN = 0, // Trạng thái vòi ngừng bán
  DANGBAN,      // Trạng thái vòi đang bán
};

uint8_t ui8_state_thung[3] = {BINHTHUONG, BINHTHUONG, BINHTHUONG}; // Mảng lưu trạng thái thùng

//-----------Trạng thái thùng nhớt và vòi bán nhớt nhận từ PC------------//
uint8_t ui8_state_thung_nhantuPC[3] = {NO_STATE, NO_STATE, NO_STATE};
// uint8_t ui8_state_thung_change[3] = {0, 0, 0};
/// Trạng thái 3 thùng chứa nhớt nhận từ PC
///  0 -> Thùng thải | 1 -> Thùng 1 | 2 -> Thùng 2

uint8_t ui8_state_voi_nhantuPC[3] = {NO_STATE, NO_STATE, NO_STATE};
// uint8_t ui8_state_voi_change[2] = {0, 0};
/// Trạng thái 2 vòi bán nhớt nhận từ PC
///  0 -> Vòi 1 | 1 -> Vòi 2

//-----------Khai báo cho LED PIXEl------------//
#define NUMPIXELS 160 // Số bóng LED trên một Strip
#define BRIGHTNESS 50 // Độ sáng (từ 0 đến 255)

Adafruit_NeoPixel strip[3] = {
    Adafruit_NeoPixel(NUMPIXELS, LED0, NEO_RBG + NEO_KHZ800),
    Adafruit_NeoPixel(NUMPIXELS, LED1, NEO_RBG + NEO_KHZ800),
    Adafruit_NeoPixel(NUMPIXELS, LED2, NEO_RBG + NEO_KHZ800)};

uint8_t ui8_state_LEDPIXEL[3] = {NO_STATE, NO_STATE, NO_STATE}; // Mảng lưu trạng thái LEDPIXEL

// Thời gian nháy hiệu ứng khi đang bơm
uint32_t ui32_time_hieuung[3] = {0, 0, 0};
uint32_t timedelay_hieuung = 1000; // Nháy sau 1 s

bool ui2_state_hieuung[3] = {0, 0, 0}; // Mảng lưu trạng thái nháy

void setup()
{
  Serial.begin(115200);

  strip[0].begin();
  strip[1].begin();
  strip[2].begin();
  strip[0].setBrightness(BRIGHTNESS);
  strip[1].setBrightness(BRIGHTNESS);
  strip[2].setBrightness(BRIGHTNESS);

  pinMode(SENSOR0, INPUT_PULLUP);
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);
  pinMode(SENSOR3, INPUT_PULLUP);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED_BLINK, OUTPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  ui32_currentTime = millis();
}
/**********************Đọc trang thái cảm biến************************/

void readSensorDebounce(int pin_cambien, int index, int delay)
{
  bool reading = !digitalRead(pin_cambien);

  if (reading != lastSensorState[index])
  {
    lastDebounceTime[index] = millis();
    lastSensorState[index] = reading;
  }

  if ((millis() - lastDebounceTime[index]) > delay)
  {
    sensor_state[index] = reading;
  }
  if (sensor_state[index] != temp_state[index])
  {
    temp_state[index] = sensor_state[index];
    sensor_change[index] = 1;
  }
}

/**********************Đọc cảm biến************************/
void readSensor()
{
  for (int i = 0; i < numSen; i++)
  {
    readSensorDebounce(SENSOR[i], i, 10);
  }
}

/**********************Gửi trạng thái cảm biến************************/
void sendState(uint8_t sensor, uint8_t state)
{

  uint8_t sizeofdata = 10;
  uint8_t *data_send = (uint8_t *)malloc(sizeofdata * sizeof(uint8_t));

  *(data_send) = ui8_slaveID;
  *(data_send + 1) = 0x08;
  *(data_send + 2) = ui16_self_order_command >> 8;
  *(data_send + 3) = ui16_self_order_command & (0x00FF);
  *(data_send + 4) = 0x02;

  for (int i = 0; i < numSen; i++)
  {
    if (sensor == SENSOR[i])
      *(data_send + 5) = i + 1;
  }

  if (state == 0)
    *(data_send + 6) = 0x00;
  else if (state == 1)
    *(data_send + 6) = 0xff;

  uint8_t bytecount = 7;

  uint16_t ui16_crc_pro = calcCRC16(bytecount, data_send);
  *(data_send + bytecount) = ui16_crc_pro >> 8;
  *(data_send + bytecount + 1) = ui16_crc_pro & 0x00FF;

  bytecount += 2;

  for (int i = 0; i < bytecount; i++)
  {
    Serial.write(*(data_send + i));
  }

  if (ui16_self_order_command < 0xFFFF)
    ++ui16_self_order_command;
  else
    ui16_self_order_command = 0x0001;

  free(data_send);
  ui32_timecho_guilenh = millis();
}
/**********************Gửi phản hồi lên PC**************************/
void sendRespond(uint8_t function_code, uint8_t status)
{

  uint8_t sizeofdata = 8;
  uint8_t *data_send = (uint8_t *)malloc(sizeofdata * sizeof(uint8_t));
  *(data_send) = ui8_slaveID;
  *(data_send + 1) = function_code;
  *(data_send + 2) = ui16_self_order_command >> 8;
  *(data_send + 3) = ui16_self_order_command & (0x00FF);
  *(data_send + 4) = 0x02;
  *(data_send + 5) = status;
  uint16_t ui16_crc_pro = calcCRC16(6, data_send);
  *(data_send + 6) = ui16_crc_pro >> 8;
  *(data_send + 7) = ui16_crc_pro & 0x00FF;

  for (int i = 0; i < 8; i++)
  {
    Serial.write(*(data_send + i));
  }
  if (ui16_self_order_command < 0xFFFF)
    ++ui16_self_order_command;
  else
    ui16_self_order_command = 0x0001;
  free(data_send);
  ui32_timecho_guilenh = millis();
}

/**********************Xử lý đọc cảm biến************************/
void processSensor()
{
  for (int i = 0; i < numSen; i++)
  {
    if (sensor_change[i] == 1)
    {
      sendState(SENSOR[i], sensor_state[i]);
      sensor_change[i] = 0;
    }
  }
}
/**********************Đọc tín hiệu Serial************************/
void readSerial()
{
  if (Serial.available())
  {
    ui32_timechar = millis();
    while (Serial.available())
    {
      unsigned char inChar = (unsigned char)Serial.read();
      if (!ui2_start)
      {
        ui2_BTreceive = true;
        ui2_start = true;
        ui8_Byteindex = 0;
      }
      if (ui2_start && (millis() - ui32_timechar) < 10)
      {
        ui8_data_rec[ui8_Byteindex] = inChar;
        ui8_Byteindex++;
        ui32_timechar = millis();
      }
    }
  }
  if (ui2_BTreceive && ((millis() - ui32_timechar) > 50))
  {
    ui2_BTreceive = false;
    ui2_BTcomplete = true;
  }
  if (ui2_BTcomplete == true)
  {
    if (ui8_Byteindex != 0 && ui8_data_rec[0] != ui8_slaveID)
    {
      ui8_status = 0x02;
      ui8_data_rec[0] = 0x02;
      ui8_Byteindex = 0;
      ui2_BTcomplete = false;
      ui2_start = false;
    }
    else if (ui8_Byteindex != 0 && ui8_data_rec[0] == ui8_slaveID)
    {

      processSerial();
      ui8_Byteindex = 0;
      ui2_BTcomplete = false;
      ui2_start = false;
    }
  }
}
/**********************Xử lý tín hiệu Serial************************/
void processSerial()
{
  uint8_t uc8_data[ui8_Byteindex - 2], uc8_checkcrc[2];
  uint8_t uc8_crchigh, uc8_crclow;
  uint16_t ui16_crc = 0;
  uint16_t ui16_du_lieu_data_nhan;

  for (int i = 0; i < ui8_Byteindex - 2; i++)
  {
    uc8_data[i] = ui8_data_rec[i];
  }

  uc8_checkcrc[0] = ui8_data_rec[ui8_Byteindex - 2];
  uc8_checkcrc[1] = ui8_data_rec[ui8_Byteindex - 1];

  ui16_crc = calcCRC16(ui8_Byteindex - 2, uc8_data);
  uc8_crclow = ui16_crc >> 8;
  uc8_crchigh = ui16_crc & 0x00FF;

  if (uc8_crchigh != uc8_checkcrc[1] || uc8_crclow != uc8_checkcrc[0])
  {
    ui8_function = 0x00;
    ui8_status = 0x01;
    ui8_phanhoi_saiCRC = 1;
    return;
  }
  ui16_crc = 0;
  ui16_order_command = uc8_data[2];
  ui16_order_command = (ui16_order_command << 8) | uc8_data[3];
  ui16_du_lieu_data_nhan = uc8_data[5];
  ui16_du_lieu_data_nhan = (ui16_du_lieu_data_nhan << 8);
  ui16_du_lieu_data_nhan = ui16_du_lieu_data_nhan | uc8_data[6];

  if (ui16_order_command == ui16_last_order_command)
  {
    ui8_phanhoi_trunglenh = 1;
    return;
  }

  switch (uc8_data[1])
  {

  /*****************KHỞI ĐỘNG KẾT THÚC QUÁ TRÌNH*********************/
  case 0x04:
    ui16_last_order_command = ui16_order_command;
    if (ui16_du_lieu_data_nhan == 0x0000)
    {
      // START_USER();
      // ui8_kethuc=0;
      // ui8_hacuadi = 0;
      // ui8_chophephacua = 0;
      // ui8_status = 0x00;
      // ui8_chaymotorbangtai = 1;
      // ui8_chaymotornghienchai = 1;
      // ui8_phanhoi_dungFunction = 1;
      // ui32_timeoutnhandien = millis() + 30000;
    }
    // END USER
    else if (ui16_du_lieu_data_nhan == 0x00FF)
    {
      // STOP_USER();
      // // ui8_hetchoigame = 1;
      // ui8_kethuc=1;
      // ui8_status = 0x00;
      // ui8_phanhoi_dungFunction = 1;
      // // ui8_chaymotorbangtai = 0;
      // // ui8_tatdongcobangtai = 1;
      // // int timechay = 20000;
      // // ui32_timeout_chaybangtai = millis() + timechay;
      // ui8_tatbangtaingay = 1;
      // ui32_timeout_ketthuc = millis() + 1000;
    }
    else
    {
      ui8_phanhoi_saiFunction = 1;
    }
    break;

  /*****************NHẬN VỊ TRÍ VÒI BÁN NHỚT******************/
  case 0X07:

    ui16_last_order_command = ui16_order_command;

    if (uc8_data[5] == 0x01) // Trạng thái vòi nhớt 1
    {
      if (uc8_data[6] == 0xff)
        ui8_state_voi_nhantuPC[1] = DANGBAN; // Bắt đầu bán
      else if (uc8_data[6] == 0x00)
        ui8_state_voi_nhantuPC[1] = NGUNGBAN; // Ngừng bán
    }
    else if (uc8_data[5] == 0x02) // Trạng thái vòi nhớt 2
    {
      if (uc8_data[6] == 0xff)
        ui8_state_voi_nhantuPC[2] = DANGBAN; // Bắt đầu bán
      else if (uc8_data[6] == 0x00)
        ui8_state_voi_nhantuPC[2] = NGUNGBAN; // Ngừng bán
    }
    break;

  /*****************NHẬN TRẠNG THÁI THÙNG PHI******************/
  case 0X11:
    ui16_last_order_command = ui16_order_command;
    if (uc8_data[5] == 0x01) // Trạng thái thùng phi 1
    {
      if (uc8_data[6] == 0xff)
        ui8_state_thung_nhantuPC[1] = DAY; // Đầy
      else if (uc8_data[6] == 0x00)
        ui8_state_thung_nhantuPC[1] = CAN; // Cạn
    }
    else if (uc8_data[5] == 0x02) // Trạng thái thùng phi 2
    {
      if (uc8_data[6] == 0xff)
        ui8_state_thung_nhantuPC[2] = DAY; // Đầy
      else if (uc8_data[6] == 0x00)
        ui8_state_thung_nhantuPC[2] = CAN; // Cạn
    }

    else if (uc8_data[5] == 0x03) // Trạng thái thung phi thải
    {
      if (uc8_data[6] == 0xff)
        ui8_state_thung_nhantuPC[0] = DAY; // Đầy
      else if (uc8_data[6] == 0x00)
        ui8_state_thung_nhantuPC[0] = CAN; // Cạn
    }

    break;

  /*****************RESET******************/
  case 0x10:
    ui16_last_order_command = ui16_order_command;
    RESET();
    break;

  default:
    break;
  }
  processState();
}

/*****************Xử lý nhận trạng thái******************/
void processState()
{
  for (int i = 0; i < 3; i++)
  {
    if (ui8_state_thung_nhantuPC[i] == DAY || ui8_state_thung_nhantuPC[i] == CAN)
    {
      ui8_state_thung[i] = ui8_state_thung_nhantuPC[i];
      ui8_state_thung_nhantuPC[i] = NO_STATE;
    }

    if (ui8_state_voi_nhantuPC[i] == DANGBAN)
    {
      ui8_state_thung[i] = DANGHOATDONG;
      ui8_state_voi_nhantuPC[i] = NO_STATE;
    }

    if (ui8_state_voi_nhantuPC[i] == NGUNGBAN)
    {
      if (ui8_state_thung[i] == DANGHOATDONG)
      {
        ui8_state_thung[i] = BINHTHUONG;
        ui8_state_voi_nhantuPC[i] = NO_STATE;
      }
      else
        ui8_state_voi_nhantuPC[i] = NO_STATE;
    }
  }
}

/**********************Set trạng thái đèn************************/
// void setLED( bool state[numLED]){
//   for(int i=0; i< numLED; i++){
//     digitalWrite(LED[i], state[i]);
//   }
// }

/**********************Vòng for set màu cho cả Strip************************/
void setOneColorAllStrip(uint8_t n, uint8_t r, uint8_t g, uint8_t b)
{
  for (int j = 0; j < NUMPIXELS; j++)
  {
    strip[n].setPixelColor(j, r, g, b);
  }
}

/**********************Set LED Pixel************************/
void setLEDPixel()
{

  for (int i = 0; i < numLED; i++)
  {

    if (ui8_state_LEDPIXEL[i] != ui8_state_thung[i])
    { // Nếu trạng thái thay đổi mới set lại màu
      ui8_state_LEDPIXEL[i] = ui8_state_thung[i];

      if (ui8_state_LEDPIXEL[i] == BINHTHUONG)
      { // Nếu vòi dang bơm thì xanh dương
        setOneColorAllStrip(i, 0, 0, 255);
      }
      else if (ui8_state_LEDPIXEL[i] == DANGHOATDONG)
      {
        setOneColorAllStrip(i, 0, 255, 0); // Nếu vòi dang bơm thì xanh lá
      }
      else if (ui8_state_LEDPIXEL[i] == DAY)
      {
        if (i == 0)
        {
          setOneColorAllStrip(i, 255, 0, 0); // Nếu là thùng thải đầy thì báo đỏ
        }
        else
          setOneColorAllStrip(i, 255, 255, 255); // Nếu là thùng nhớt bán thì báo trắng
      }
      else if (ui8_state_LEDPIXEL[i] == CAN)
      {
        if (i == 0)
        {
          setOneColorAllStrip(i, 255, 255, 255); // Nếu là thùng thải cạn thì báo trắng
        }
        else
          setOneColorAllStrip(i, 255, 0, 0); // Nếu là thùng nhớt bán cạn thì báo đỏ
      }
    }
    else if (ui8_state_LEDPIXEL[i] == DANGHOATDONG)
    {
      // Hiệu ứng LED khi vòi đang bơm (nháy xanh lá)
      if (millis() > ui32_time_hieuung[i])
      {
        ui32_time_hieuung[i] = millis() + timedelay_hieuung;
        ui2_state_hieuung[i] = !ui2_state_hieuung[i];

        if (ui2_state_hieuung[i] == 0)
          setOneColorAllStrip(i, 0, 255, 0);
        else if (ui2_state_hieuung[i] == 1)
          setOneColorAllStrip(i, 0, 0, 0);
      }
    }
    strip[i].show();
  }
}
/**********************RESET trạng thải thùng************************/
void RESET()
{
  for (int i = 0; i < 3; ++i)
  {
    ui8_state_thung_nhantuPC[i] = NO_STATE;
    ui8_state_voi_nhantuPC[i] = NO_STATE;
    ui8_state_LEDPIXEL[i] = NO_STATE;
    ui8_state_thung[i] = BINHTHUONG;
  }
}

void testLED()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    strip[0].setPixelColor(i, 255, 255, 255, BRIGHTNESS);
    strip[1].setPixelColor(i, 255, 255, 255, BRIGHTNESS);
    strip[2].setPixelColor(i, 255, 255, 255, BRIGHTNESS);
  }
  strip[0].show();
  strip[1].show();
  strip[2].show();
}

/**********************Đèn onboard báo hoạt động (blink ms)**************************/
void denbao(int time)
{
  if (millis() - ui32_time_blink > time)
  {
    ui32_time_blink = millis();
    led_status = !led_status;
    digitalWrite(LED_BLINK, led_status);
  }
}

/**********************Set thời gian còi onboard hoạt động (ms)**************************/
void batcoibao(int time)
{
  digitalWrite(BUZZER, HIGH);
  timeout_buz = millis() + time;
  buz_status = 1;
}
/**********************Tắt còi nếu thời gian vượt qua thời gian đã Set**************************/
void tatcoibao()
{
  if (buz_status == 1 && millis() >= timeout_buz)
  {
    digitalWrite(BUZZER, LOW);
    buz_status = 0;
  }
}

/**********************Loop************************/
int flag = 0;
void loop()
{

  if (flag == 0)
  {

    // Bật còi chỉ một lần
    //  flag = 1;
    //  batcoibao(1000);
  }

  /**********************Main Funtion**************************/
  readSensor();
  processSensor();

  readSerial();

  setLEDPixel();

  /**********************Đèn báo hoạt động**************************/
  denbao(1000);
  tatcoibao();
}
