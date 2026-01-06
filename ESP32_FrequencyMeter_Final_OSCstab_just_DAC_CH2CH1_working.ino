// ESP32 Frequency Meter
// ESP32 DevKit 38 pins + I2C PCF8574 LCD
// Arduino IDE 2.3.2   / ESP32 Arduino V 3.02
// Gustavo Murta e Rui Viana august/2020 - updated 2024/07/19
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// https://www.esp32.com/viewtopic.php?f=19&t=17018
// LCD I2C SDA - GPIO_21
// LCD I2C SCL - GPIO_22
// https://pastebin.com/GgrZ0gpD the PID controll or https://ryand.io/AutoPID/ 

#include "stdio.h"            // Library STDIO
#include "driver/ledc.h"      // Library ESP32 LEDC
#include "driver/pcnt.h"      // Library ESP32 PCNT
#include "soc/pcnt_struct.h"  // Library ESP32 PCNT
#include "esp32/rom/gpio.h"   // Library ESP32 GPIO

#include <LCD-I2C.h>     // Library LCD with PCF8574
#include <Wire.h>      // Library LCD with PCF8574

//#include <Arduino.h>
//#include <OneButton.h>


//#include <DallasTemperature.h>
//#include <OneWire.h>

 

LCD_I2C lcd(0x27, 16, 4);  // LCD_I2C lcd(0x27, 16, 2); Instance LCD I2C - address x3F

#define PCNT_COUNT_UNIT PCNT_UNIT_0        // Set Pulse Counter Unit - 0
#define PCNT_COUNT_CHANNEL PCNT_CHANNEL_0  // Set Pulse Counter channel - 0

#define PCNT_INPUT_SIG_IO GPIO_NUM_34    // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO GPIO_NUM_33     // LEDC output - pulse generator - GPIO_33
#define PCNT_INPUT_CTRL_IO GPIO_NUM_35   // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down
#define OUTPUT_CONTROL_GPIO GPIO_NUM_32  // Timer output control port - GPIO_32
#define PCNT_H_LIM_VAL overflow          // Overflow of Pulse Counter

#define IN_BOARD_LED GPIO_NUM_2  // ESP32 native LED - GPIO 2

#define DAC_CH1 25
#define DAC_CH2 26
//PID Definitions
#define OUTPUT_MIN 0 //autoPID 
#define OUTPUT_MAX 255 //autoPID 

#define FREQ_READ_DELAY 800 //can only read digital temp sensor every ~750ms

const int buttonPin = 17;
int buttonState = 0; 
bool flag = true;                // Flag to enable print frequency reading
uint32_t overflow = 20000;       // Max Pulse Counter value 20000
int16_t pulses = 0;              // Pulse Counter value
uint32_t multPulses = 0;         // Number of PCNT counter overflows
uint32_t sample_time = 1000000;  // Sample time of 1 second to count pulses (change the value to calibrate frequency meter)
uint32_t osc_freq = 16000;       // Oscillator frequency - initial 16000 Hz (may be 1 Hz to 40 MHz)
uint32_t mDuty = 0;              // Duty value
uint32_t resolution = 0;         // Resolution value of Oscillator
float frequency = 0;             // frequency value was originaly  float but changet to double because of autoPID requare doubleh
float frequency_1 = 0;             // new update frequency value was originaly  float but changet to double because of autoPID requare doubleh
float frequencyOld = 1; 
float frequencyStation = 15; 
char buf[32];                    // Buffer

//int button;
int oldButton = 0;
const double frequencyCorrection = 0.999965101217967;
int state = 0;
volatile uint8_t dac = 128;
volatile float_t dac_d = 128;
volatile uint8_t daCh2 = 128;
volatile float_t da_Ch2 = 128;
//double increment = 0.0001;



unsigned long lastFrequencyUpdate; //tracks clock time of last temp update double

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateFrequency() {
  if ((millis() - lastFrequencyUpdate) > FREQ_READ_DELAY) {
    frequency_1 = frequency; //get frequency reading
    lastFrequencyUpdate = millis();
    //frequencySensors.requestFrequency(); //request reading for next time
    return true;
  }
  return false;
}//void updateTemperature

esp_timer_create_args_t create_args;  // Create an esp_timer instance
esp_timer_handle_t timer_handle;      // Create an single timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // portMUX_TYPE to do synchronism

//----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                  // Serial Console Arduino 115200 Bps
  Serial.println(" Input the Frequency - 1 to 40 MHz");  // Console print
  pinMode(buttonPin, INPUT_PULLUP);
  Wire.begin();
  lcd.begin(&Wire);
  lcd.display();
  lcd.backlight();
  //lcd.print("  Frequency:");  // LCD print
  init_frequencyMeter();      // Initialize Frequency Meter
}

//----------------------------------------------------------------------------
void init_osc_freq()  // Initialize Oscillator to test Freq Meter
{
  resolution = (log(80000000 / osc_freq) / log(2)) / 2;  // Calc of resolution of Oscillator
  if (resolution < 1) resolution = 1;                    // set min resolution
  Serial.println(resolution);                            // Print
  mDuty = (pow(2, resolution)) / 2;                      // Calc of Duty Cycle 50% of the pulse
  Serial.println(mDuty);                                 // Print

  ledc_timer_config_t ledc_timer = {};  // LEDC timer config instance

  ledc_timer.duty_resolution = ledc_timer_bit_t(resolution);  // Set resolution
  ledc_timer.freq_hz = osc_freq;                              // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;               // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_0;                        // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                             // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};  // LEDC Channel config instance

  ledc_channel.channel = LEDC_CHANNEL_0;           // Set HS Channel - 0
  ledc_channel.duty = mDuty;                       // Set Duty Cycle 50%
  ledc_channel.gpio_num = LEDC_HS_CH0_GPIO;        // LEDC Oscillator output GPIO 33
  ledc_channel.intr_type = LEDC_INTR_DISABLE;      // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;  // Set LEDC high speed mode
  ledc_channel.timer_sel = LEDC_TIMER_0;           // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);              // Config LEDC channel
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)  // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timerMux);        // disabling the interrupts
  multPulses++;                             // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);  // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timerMux);         // enabling the interrupts
}

//----------------------------------------------------------------------------------
void init_PCNT(void)  // Initialize and run PCNT unit
{
  pcnt_config_t pcnt_config = {};  // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;  // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;  // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;              // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;        // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;      // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;           // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;           // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;      // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;         // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                  // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT);  // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT);  // Clear PCNT unit

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);   // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);  // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                    // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);  // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
void read_PCNT(void *p)  // Read Pulse Counter
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);            // Stop counter - output control LOW
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);  // Read Pulse Counter value
  flag = true;                                       // Change flag to enable print
}

//---------------------------------------------------------------------------------
void init_frequencyMeter() {
  init_osc_freq();  // Initialize Oscillator
  init_PCNT();      // Initialize and run PCNT unit

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                  // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);  // Set GPIO 32 as output

  create_args.callback = read_PCNT;               // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);  // Create esp-timer instance

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);  // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);     // Set GPIO matrin IN - Freq Meter input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);  // Set GPIO matrix OUT - to inboard LED
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos)  // Format an unsigned long (32 bits) into a string
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = ',';  // decimal separator
  return s;
}
//----------------------------------------------------------------------------------------
char *ltos(long val, char *s, int radix)  // Format an long (32 bits) into a string
{
  if (radix < 2 || radix > 36) {
    s[0] = 0;
  } else {
    char *p = s;
    if (radix == 10 && val < 0) {
      val = -val;
      *p++ = '-';
    }
    p = ultos_recursive(val, p, radix, 0) - 1;
    *p = 0;
  }
  return s;
}

//---------------------------------------------------------------------------------
void loop() {
buttonState = digitalRead(buttonPin);
//Serial.println(buttonState);
  if (buttonState == LOW &&  oldButton == 0 ) {
    printf("we have a new button press \n"); //we have a new button press
     
      if(state == 0) // if the state is off, turn it on
         {
            printf("Locking freq  \n");  
            //osc_freq = frequency;
            frequencyOld = frequency;
            state =  1;
            dac_d = 128;
            dac = dac_d;
            da_Ch2 = 128;
            daCh2 = da_Ch2;
            dacWrite(DAC_CH1, dac); // middle voltage on pin 25
            dacWrite(DAC_CH2, daCh2); // middle voltage on pin 26
            // oldButton = 1;   
           // init_osc_freq();  // reconfigure ledc function - oscillator
            osc_freq =  frequencyOld;
           //init_osc_freq(); 

                    
             
         } 
         else  // if the state is off, turn it on
         {
            printf(" Unlock freq  \n");  
            //osc_freq = 10;
            frequencyOld = 1;
            state =  0;
           //  oldButton = 1;   
            //init_osc_freq();  // reconfigure ledc function - oscillator
            //DAC_CH1= 128;
             osc_freq = 1;
             dac_d = 128;
            dac = dac_d;
            da_Ch2 = 128;
            daCh2 = da_Ch2;
            dacWrite(DAC_CH1, dac); // middle voltage on pin 25
            dacWrite(DAC_CH2, daCh2); // middle voltage on pin 26
            //init_osc_freq();
            
         }   
         oldButton = 1;   
   } else if (buttonState == HIGH &&  oldButton == 1 ){
           oldButton = 0;
           printf("button relissed   \n");  
           //osc_freq =  frequencyOld;
           //init_osc_freq();  // reconfigure ledc function - oscillator        
            
           osc_freq =  frequencyOld;
          //init_osc_freq();
          } 

       //   osc_freq =  frequencyOld;
        //   init_osc_freq();
           
  //  if (buttonState == HIGH && oldButton == 1) {
 
  // //  printf("LOCKED up \n");  

  // // frequencyOld = 10;
  // // osc_freq =  frequencyOld;
  // // state = 0;
  // // init_osc_freq();  // reconfigure ledc function - oscillator
  // oldButton == 0;
  // }
    //frequencyStation
 //          increment = (frequency - osc_freq)*0.0001;
    if (osc_freq == 1){
      dac = 128;
      dacWrite(DAC_CH1, dac); // middle voltage on pin 25
      daCh2 = 128;
      dacWrite(DAC_CH2, daCh2); // middle voltage on pin 26
       } else  {
        if (frequency > osc_freq  && dac_d > 1 && da_Ch2 > 1 ) {
          dac_d = dac_d - 0.000001;
          dac = dac_d;
          //dacWrite(DAC_CH1, dac); // decrease voltage on pin 25
          da_Ch2 = da_Ch2 - 0.00001;
          daCh2 = da_Ch2;
         // dacWrite(DAC_CH2, daCh2); // decrease voltage on pin 26
         } 
          if (frequency > osc_freq  && da_Ch2 > 1) {
          da_Ch2 = da_Ch2 - 0.001;
          daCh2 = da_Ch2;
          //dacWrite(DAC_CH2, daCh2); // decrease voltage on pin 26
          } 
        // if (frequency > osc_freq + 100 && dac_d > 1 && da_Ch2 > 1) {
        //   //dac = 0;
        //   dac_d = dac_d - 0.0001;
        //   dac = dac_d;
        //  // dacWrite(DAC_CH1, dac); // decrease voltage on pin 25
        //    da_Ch2 = da_Ch2 - 0.001;
        //   daCh2 = da_Ch2;
        //  // dacWrite(DAC_CH2, daCh2); // decrease voltage on pin 26
        // } 
        // if (frequency > osc_freq + 200 && da_Ch2 > 1) {
        //   da_Ch2 = da_Ch2 - 0.001;
        //   daCh2 = da_Ch2;
        //   //dacWrite(DAC_CH2, daCh2); // decrease  voltage on pin 26
        //   } 
        if (frequency < osc_freq  && dac_d <= 254 && da_Ch2 <= 254)   {
          dac_d = dac_d + 0.00001;
          dac = dac_d;
         // dacWrite(DAC_CH1, dac); // increase voltage on pin 25
           da_Ch2 = da_Ch2 + 0.0001;
          daCh2 = da_Ch2;
         // dacWrite(DAC_CH2, daCh2); // decrease voltage on pin 26
        } 
        if (frequency > osc_freq  && da_Ch2 <= 254) {
          da_Ch2 = da_Ch2 + 0.0001;
          daCh2 = da_Ch2;
          //dacWrite(DAC_CH2, daCh2); // increase voltage on pin 26
          } 
        // if (frequency < osc_freq - 100 && dac_d <= 254 && da_Ch2 <= 254)   {
        //   dac_d = dac_d + 0.0001;
        //   dac = dac_d;
        //   //dacWrite(DAC_CH1, dac); // increase voltage on pin 25
        //   da_Ch2 = da_Ch2 + 0.0001;
        //   daCh2 = da_Ch2;
        //   //dacWrite(DAC_CH2, daCh2); // decrease voltage on pin 26
        // } 
        // if (frequency > osc_freq - 200 && da_Ch2 <= 254) {
        //   da_Ch2 = da_Ch2 + 0.0001;
        //   daCh2 = da_Ch2;
        //   //dacWrite(DAC_CH2, daCh2); // increase voltage on pin 26
        //   } 
          if (frequency > osc_freq && dac_d > 1 ) {
          dac_d = dac_d - 0.00001;
          dac = dac_d;
          //dacWrite(DAC_CH1, dac); // decrease voltage on pin 25
         } 
           
        if (frequency < osc_freq  && dac_d <= 254)   {
          dac_d = dac_d + 0.00001;
          dac = dac_d;
         // dacWrite(DAC_CH1, dac); // increase voltage on pin 25
          
        } 
       
    } 
      dacWrite(DAC_CH1, dac);
      dacWrite(DAC_CH2, daCh2);
      
      
    // if (frequencyOld == 1) {
    //     dac = 128;
    //   dacWrite(DAC_CH1, dac); // set in the middle the voltage on pin 25 
      
    //   } 
 if((frequency * frequencyCorrection) >= 18000000) {
  frequencyStation = frequency * frequencyCorrection  -4032000;
   // reconfigure ledc function - oscillator
  }else {
   frequencyStation = frequency * frequencyCorrection -455000;
  
  } 
   
// osc_freq =  frequencyOld;
  if (flag == true)  // If count has ended
  {
    flag = false;                                          // Change flag to disable print
    frequency = (pulses + (multPulses * overflow)) / 2;    // Calculation of frequency
    // printf("         Frequency : %s", (ltos(frequency, buf, 10)));  // Print frequency with commas
    // printf(" Hz \n");                                          // Print unity Hz
    // printf("osc_freq  : %s", (ltos(osc_freq , buf, 10)));  // Print oscilator frequency with commas
    // printf(" Hz \n");                              // Print unity Hz
   
    lcd.setCursor(0, 0); 
     lcd.print("DAC1 ");                     // Set cursor position - column and row
    lcd.print((ltos(dac, buf, 10)));  // LCD print DAC_ch1
          // LCD print unity 
    
    lcd.setCursor(11, 0);                    // Set cursor posit
    lcd.print("DAC2 ");  
    lcd.print((ltos(daCh2, buf, 10)));  // LCD print
    lcd.setCursor(0, 2);                    // Set cursor position - column and row
    lcd.print((ltos(frequency, buf, 10)));  // LCD print frequency
    lcd.print(" Hz OSC");         // LCD print unity Hz 

    lcd.setCursor(0, 1);                   // Set cursor position - column and row
    lcd.print((ltos( frequencyStation , buf, 10)));  // LCD print frequency
    lcd.print(" Hz St");         // LCD print unity Hz 
    lcd.setCursor(0, 3);                   // Set cursor position - column and row
    lcd.print((ltos(osc_freq, buf, 10)));  // LCD print frequency
    lcd.print(" Hz set");         // LCD print unity Hz 

    multPulses = 0;  // Clear overflow counter
    // Put your function here, if you want
      //dacWrite(DAC_CH1, 128);
    //Serial.println("DAC Value 128");
    // lcd.setCursor(2, 10);
    //lcd.print("DAC 128 "); 
    //delay(100);  // Delay 100 ms
    // Put your function here, if you want

    pcnt_counter_clear(PCNT_COUNT_UNIT);              // Clear Pulse Counter
    esp_timer_start_once(timer_handle, sample_time);  // Initialize High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);           // Set enable PCNT count
  }

  String inputString = "";  // clear temporary string
  //osc_freq = 0;             // Clear oscillator frequency
  while (Serial.available()) {
    char inChar = (char)Serial.read();  // Reads a byte on the console
    inputString += inChar;              // Add char to string
    if (inChar == '\n')                 // If new line (enter)
    {
      osc_freq = inputString.toInt();  // Converts String into integer value
      inputString = "";                // Clear string
    }
  }
  // if (osc_freq != 0)  // If some value inputted to oscillator frequency
  // {
   // init_osc_freq();  // reconfigure ledc function - oscillator
  // }
}
