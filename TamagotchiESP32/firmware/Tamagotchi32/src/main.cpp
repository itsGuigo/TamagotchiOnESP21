
 #include <U8g2lib.h>
 #include <Wire.h>
 
 #include "tamalib.h"
 #include "hw.h"
 #include "bitmaps.h"
 #if defined(ENABLE_AUTO_SAVE_STATUS) || defined(ENABLE_LOAD_STATE_FROM_EEPROM)
 #include "savestate.h"
 #endif
 
 #if defined(ESP32)
 #include "Tone32.h"
 #endif
 
 /***** Set display orientation, U8G2_MIRROR_VERTICAL is not supported *****/
 #define U8G2_LAYOUT_NORMAL
 // #define U8G2_LAYOUT_ROTATE_180
 // #define U8G2_LAYOUT_MIRROR
 /**************************************************************************/
 
 #ifdef U8G2_LAYOUT_NORMAL
 U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R0);
 #endif
 
 #ifdef U8G2_LAYOUT_ROTATE_180
 U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R2);
 #endif
 
 #ifdef U8G2_LAYOUT_MIRROR
 U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_MIRROR);
 #endif
 
 #if defined(ESP32)
   #define PIN_BTN_L 18
   #define PIN_BTN_M 19
   #define PIN_BTN_R 23
   #define PIN_BUZZER 15
   #define BUZZER_CHANNEL 0
   #define TONE_CHANNEL 15
 
 #elif defined(ESP8266)
   #define PIN_BTN_L 12
   #define PIN_BTN_M 13
   #define PIN_BTN_R 15
   #define PIN_BUZZER 2
 
 #else
   #define PIN_BTN_L 2
   #define PIN_BTN_M 3
   #define PIN_BTN_R 4
   #define PIN_BUZZER 9
 #endif
 
 #if defined(ESP32)
 void esp32_noTone(uint8_t pin, uint8_t channel)
 {
   ledcDetachPin(pin);
   ledcWrite(channel, 0);
 }
 
 void esp32_tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel)
 {
   if (!ledcRead(channel))
   {
     ledcAttachPin(pin, channel);
   }
 
   ledcWriteTone(channel, frequency);
 }
 #endif
 
 void displayTama();
 
 /**** TamaLib Specific Variables ****/
 static uint16_t current_freq = 0;
 static bool_t matrix_buffer[LCD_HEIGHT][LCD_WIDTH / 8] = {{0}};
 // static byte runOnceBool = 0;
 static bool_t icon_buffer[ICON_NUM] = {0};
 static cpu_state_t cpuState;
 static unsigned long lastSaveTimestamp = 0;
 static long last_interaction = 0;
 /************************************/
 
 static void hal_halt(void)
 {
   // Serial.println("Halt!");
 }
 
 static void hal_log(log_level_t level, char *buff, ...)
 {
   Serial.println(buff);
 }
 
 static timestamp_t custom_log_timestamp(void)
 {
   return millis() * (1000 / SPEED_DIVIDER);
 }
 
 static void hal_sleep_until(timestamp_t ts)
 {
   int32_t remaining = (int32_t)(ts - custom_log_timestamp());
   if (remaining > 0)
   {
 #ifdef ENABLE_DEEPSLEEP
     esp_deep_sleep(remaining);
 #endif
   }
 }
 
 static void hal_update_screen(void)
 {
   displayTama();
 }
 
 static void hal_set_lcd_matrix(u8_t x, u8_t y, bool_t val)
 {
   uint8_t mask;
   if (val)
   {
     mask = 0b10000000 >> (x % 8);
     matrix_buffer[y][x / 8] = matrix_buffer[y][x / 8] | mask;
   }
   else
   {
     mask = 0b01111111;
     for (byte i = 0; i < (x % 8); i++)
     {
       mask = (mask >> 1) | 0b10000000;
     }
     matrix_buffer[y][x / 8] = matrix_buffer[y][x / 8] & mask;
   }
 }
 
 static void hal_set_lcd_icon(u8_t icon, bool_t val)
 {
   icon_buffer[icon] = val;
 }
 
 static void hal_set_frequency(u32_t freq)
 {
   current_freq = freq;
 }
 
 static void hal_play_frequency(bool_t en)
 {
 #ifdef ENABLE_TAMA_SOUND
   if (en)
   {
 
 #if defined(ESP32)
     esp32_tone(PIN_BUZZER, current_freq, 500, BUZZER_CHANNEL);
 #else
     tone(PIN_BUZZER, current_freq);
 #endif
   }
   else
   {
 #if defined(ESP32)
     esp32_noTone(PIN_BUZZER, BUZZER_CHANNEL);
 #else
     noTone(PIN_BUZZER);
 #ifdef ENABLE_TAMA_SOUND_ACTIVE_LOW
     digitalWrite(PIN_BUZZER, HIGH);
 #endif
 #endif
   }
 #endif
 }
 
 static bool_t button4state = 0;
 
 static int hal_handler(void)
 {
 #ifdef ENABLE_SERIAL_DEBUG_INPUT
   if (Serial.available() > 0)
   {
     int incomingByte = Serial.read();
     Serial.println(incomingByte, DEC);
     if (incomingByte == 49)
     {
       hw_set_button(BTN_LEFT, BTN_STATE_PRESSED);
     }
     else if (incomingByte == 50)
     {
       hw_set_button(BTN_LEFT, BTN_STATE_RELEASED);
     }
     else if (incomingByte == 51)
     {
       hw_set_button(BTN_MIDDLE, BTN_STATE_PRESSED);
     }
     else if (incomingByte == 52)
     {
       hw_set_button(BTN_MIDDLE, BTN_STATE_RELEASED);
     }
     else if (incomingByte == 53)
     {
       hw_set_button(BTN_RIGHT, BTN_STATE_PRESSED);
     }
     else if (incomingByte == 54)
     {
       hw_set_button(BTN_RIGHT, BTN_STATE_RELEASED);
     }
   }
 #endif
 
   if (digitalRead(PIN_BTN_L) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     hw_set_button(BTN_LEFT, BTN_STATE_PRESSED);
   }
   else
   {
     hw_set_button(BTN_LEFT, BTN_STATE_RELEASED);
   }
 
   if (digitalRead(PIN_BTN_M) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     hw_set_button(BTN_MIDDLE, BTN_STATE_PRESSED);
   }
   else
   {
     hw_set_button(BTN_MIDDLE, BTN_STATE_RELEASED);
   }
 
   if (digitalRead(PIN_BTN_R) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     hw_set_button(BTN_RIGHT, BTN_STATE_PRESSED);
   }
   else
   {
     hw_set_button(BTN_RIGHT, BTN_STATE_RELEASED);
   }
   return 0;
 }
 
 static hal_t hal = {
     .halt = &hal_halt,
     .log = &hal_log,
     .sleep_until = &hal_sleep_until,
     .get_timestamp = &custom_log_timestamp,
     .update_screen = &hal_update_screen,
     .set_lcd_matrix = &hal_set_lcd_matrix,
     .set_lcd_icon = &hal_set_lcd_icon,
     .set_frequency = &hal_set_frequency,
     .play_frequency = &hal_play_frequency,
     .handler = &hal_handler,
 };
 
 void drawTriangle(uint8_t x, uint8_t y)
 {
   // display.drawLine(x,y,x+6,y);
   display.drawLine(x + 1, y + 1, x + 5, y + 1);
   display.drawLine(x + 2, y + 2, x + 4, y + 2);
   display.drawLine(x + 3, y + 3, x + 3, y + 3);
 }
 
 void drawTamaRow(uint8_t tamaLCD_y, uint8_t ActualLCD_y, uint8_t thick)
 {
   uint8_t i;
   for (i = 0; i < LCD_WIDTH; i++)
   {
     uint8_t mask = 0b10000000;
     mask = mask >> (i % 8);
     if ((matrix_buffer[tamaLCD_y][i / 8] & mask) != 0)
     {
       display.drawBox(i + i + i + 16, ActualLCD_y, 2, thick);
     }
   }
 }
 
 void drawTamaSelection(uint8_t y)
 {
   uint8_t i;
   for (i = 0; i < 7; i++)
   {
     if (icon_buffer[i])
       drawTriangle(i * 16 + 5, y);
     display.drawXBMP(i * 16 + 4, y + 6, 16, 9, bitmaps + i * 18);
   }
   if (icon_buffer[7])
   {
     drawTriangle(7 * 16 + 5, y);
     display.drawXBMP(7 * 16 + 4, y + 6, 16, 9, bitmaps + 7 * 18);
   }
 }
 
 void displayTama()
 {
   uint8_t j;
   display.firstPage();
 #ifdef U8G2_LAYOUT_ROTATE_180
   drawTamaSelection(49);
   display.nextPage();
 
   for (j = 11; j < LCD_HEIGHT; j++)
   {
     drawTamaRow(j, j + j + j, 2);
   }
   display.nextPage();
 
   for (j = 5; j <= 10; j++)
   {
     if (j == 5)
     {
       drawTamaRow(j, j + j + j + 1, 1);
     }
     else
     {
       drawTamaRow(j, j + j + j, 2);
     }
   }
   display.nextPage();
 
   for (j = 0; j <= 5; j++)
   {
     if (j == 5)
     {
       drawTamaRow(j, j + j + j, 1);
     }
     else
     {
       drawTamaRow(j, j + j + j, 2);
     }
   }
   display.nextPage();
 #else
   for (j = 0; j < LCD_HEIGHT; j++)
   {
     if (j != 5)
       drawTamaRow(j, j + j + j, 2);
     if (j == 5)
     {
       drawTamaRow(j, j + j + j, 1);
       display.nextPage();
       drawTamaRow(j, j + j + j + 1, 1);
     }
     if (j == 10)
       display.nextPage();
   }
   display.nextPage();
   drawTamaSelection(49);
   display.nextPage();
 #endif
 }
 
 #ifdef ENABLE_DUMP_STATE_TO_SERIAL_WHEN_START
 void dumpStateToSerial()
 {
   uint16_t i, count = 0;
   char tmp[10];
   cpu_get_state(&cpuState);
   u4_t *memTemp = cpuState.memory;
   uint8_t *cpuS = (uint8_t *)&cpuState;
 
   Serial.println("");
   Serial.println("static const uint8_t hardcodedState[] PROGMEM = {");
   for (i = 0; i < sizeof(cpu_state_t); i++, count++)
   {
     sprintf(tmp, "0x%02X,", cpuS[i]);
     Serial.print(tmp);
     if ((count % 16) == 15)
       Serial.println("");
   }
   for (i = 0; i < MEMORY_SIZE; i++, count++)
   {
     sprintf(tmp, "0x%02X,", memTemp[i]);
     Serial.print(tmp);
     if ((count % 16) == 15)
       Serial.println("");
   }
   Serial.println("};");
   /*
     Serial.println("");
     Serial.println("static const uint8_t bitmaps[] PROGMEM = {");
     for(i=0;i<144;i++) {
       sprintf(tmp, "0x%02X,", bitmaps_raw[i]);
       Serial.print(tmp);
       if ((i % 18)==17) Serial.println("");
     }
     Serial.println("};");  */
 }
 #endif
 
 uint8_t reverseBits(uint8_t num)
 {
   uint8_t reverse_num = 0;
   uint8_t i;
   for (i = 0; i < 8; i++)
   {
     if ((num & (1 << i)))
       reverse_num |= 1 << ((8 - 1) - i);
   }
   return reverse_num;
 }
 
  void setup()
  {
    Serial.begin(9600);

    pinMode(PIN_BTN_L, INPUT);
    pinMode(PIN_BTN_M, INPUT);
    pinMode(PIN_BTN_R, INPUT);

  #if defined(ESP32)
    ledcSetup(BUZZER_CHANNEL, NOTE_C4, 8);
  #endif
    Wire.begin();          // Inicia o barramento I2C
    Wire.setClock(100000); // Frequência padrão segura para muitos displays
    delay(500);            // Aguarda estabilização do display
    display.begin();

    tamalib_register_hal(&hal);
    tamalib_set_framerate(TAMA_DISPLAY_FRAMERATE);
    tamalib_init(1000000);

  #if defined(ENABLE_AUTO_SAVE_STATUS) || defined(ENABLE_LOAD_STATE_FROM_EEPROM)
    initEEPROM();
  #endif

  #ifdef ENABLE_LOAD_STATE_FROM_EEPROM
    if (validEEPROM())
    {
      loadStateFromEEPROM(&cpuState);
    }
    else
    {
      Serial.println(F("No magic number in state, skipping state restore"));
    }
  #elif ENABLE_LOAD_HARCODED_STATE_WHEN_START
    loadHardcodedState();
  #endif

  #ifdef ENABLE_DUMP_STATE_TO_SERIAL_WHEN_START
    dumpStateToSerial();
  #endif
  }
 
 uint32_t right_long_press_started = 0;
 
 void upload_state()
 {
 }
 
 void esp_deep_sleep(int _ms)
 {
 #ifndef MY_MACRO
   return;
 #endif
   // save CURRENT STATE
   saveStateToEEPROM(&cpuState);
 
 
   //DISABLE DISPLAY
   display.clear();
 
   // ENTER DEEPSLEEP
 #if defined(ESP32)
   esp_sleep_enable_timer_wakeup(_ms * 1000);
   esp_deep_sleep_start();
 #elif defined(ESP8266)
   ESP.deepSleep(_ms * 1000);
   yield();
 #endif
 }
 #define DEEPSLEEP_INTERVAL 60000000ULL  //60 segundos (60 milhões de microssegundos)
 void loop()
 {
   tamalib_mainloop_step_by_step();
 #ifdef ENABLE_AUTO_SAVE_STATUS
   if ((millis() - lastSaveTimestamp) > (AUTO_SAVE_MINUTES * 60 * 1000))
   {
     lastSaveTimestamp = millis();
     saveStateToEEPROM(&cpuState);
   }
 
   if (digitalRead(PIN_BTN_M) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     if (millis() - right_long_press_started > AUTO_SAVE_MINUTES * 1000)
     {
       eraseStateFromEEPROM();
 #if defined(ESP8266) || defined(ESP32)
       ESP.restart();
 #endif
     }
   }
   else if (digitalRead(PIN_BTN_R) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     if (millis() - right_long_press_started > AUTO_SAVE_MINUTES * 1000)
     {
 #if defined(ESP8266) || defined(ESP32)
       upload_state();
 #endif
     }
   }
   else if (digitalRead(PIN_BTN_L) == BUTTON_VOLTAGE_LEVEL_PRESSED)
   {
     if (millis() - right_long_press_started > AUTO_SAVE_MINUTES * 1000)
     {
       esp_deep_sleep(DEEPSLEEP_INTERVAL * 1000);
     }
     else
     {
       right_long_press_started = millis();
     }
   }
 #endif // Fechar o #ifdef ENABLE_AUTO_SAVE_STATUS
 }