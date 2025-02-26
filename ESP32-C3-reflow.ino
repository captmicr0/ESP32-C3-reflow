#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <MAX31855.h>
#include <PID_v1_bc.h>
#include <Preferences.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define SCK_PIN 4
#define MISO_PIN 5
#define CS_MAX1_PIN 7
#define CS_MAX2_PIN 10

#define SSR1_PIN 3

#define BTN1_PIN 2
#define BTN2_PIN 1
#define BTN3_PIN 0

#define WIDTH 128
#define HEIGHT 64

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 9, /* data=*/ 8, /* reset=*/ U8X8_PIN_NONE); // pure SW emulated I2C

double pid_setpoint, pid_input, pid_output;
double Kp = 25, Ki = 0.3, Kd = 0.05;
double Kpr = 50, Kir = 0.1, Kdr = 0;
PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
PID pid_reflow(&pid_input, &pid_output, &pid_setpoint, Kpr, Kir, Kdr, DIRECT);

int dutyCycle = 100;
const int PWMFreq = 20000; //50hz
const int PWMChannel = 0;
const int PWMResolution = 10; //12 bits 0-4095

MAX31855 thermocouple1(SCK_PIN, CS_MAX2_PIN, MISO_PIN);
MAX31855 thermocouple2(SCK_PIN, CS_MAX1_PIN, MISO_PIN);

float current_temperature1 = 0.0;
float current_temperature2 = 0.0;
bool temp1_error = true;
bool temp2_error = true;

bool running_reflow = false;
bool init_reflow = false;
bool running_const = false;
bool init_const = false;
bool rampup_reflow = true;
bool rampup_const = true;
bool rampup_const_close = true;
bool preheat_done = false;
bool reflow_done = false;
bool buzzer_state = true;

// Soldering profile
float temperature_setpoint_reflow = 0.0;
float temperature_setpoint_const = 100.0;
float temperature_off = 30.0;
float temperature_soak1 = 150.0;
float temperature_soak2 = 160.0;
float temperature_reflow = 220.0;
float temperature_cooling = 210.0;
int profile_counter = 0;

Preferences preferences;

#define N_PROFILES 10.0
float selected_profile = 0.0;
float profile_temp[10][10]; // initial, soak1, soak2, reflow, cooling
float profile_time[] = {0.0, 120.0, 200.0, 260.0, 300.0};
int temp_reflow_individual[300];

void store_profile(){
  String key = "profile" + String(int(selected_profile)) + "_";
  preferences.putFloat((key + "soak1").c_str(), temperature_soak1);
  preferences.putFloat((key + "soak2").c_str(), temperature_soak2);
  preferences.putFloat((key + "reflow").c_str(), temperature_reflow);
  preferences.putFloat((key + "cooling").c_str(), temperature_cooling);
}

void read_all_profiles(){
  // preferences.putFloat((name + "_0").c_str(), acs_offset_zero);
  for(int i=0; i<int(N_PROFILES); i++){
    String key = "profile" + String(i) + "_";
    profile_temp[i][1] = preferences.getFloat((key + "soak1").c_str());
    profile_temp[i][2] = preferences.getFloat((key + "soak2").c_str());
    profile_temp[i][3] = preferences.getFloat((key + "reflow").c_str());
    profile_temp[i][4] = preferences.getFloat((key + "cooling").c_str());
  }
}

void calculate_profile_individual(){
  profile_temp[int(selected_profile)][0] = temperature_off;
  profile_temp[int(selected_profile)][1] = temperature_soak1;
  profile_temp[int(selected_profile)][2] = temperature_soak2;
  profile_temp[int(selected_profile)][3] = temperature_reflow;
  profile_temp[int(selected_profile)][4] = temperature_cooling;

  store_profile();

  for(int i=0; i<300; i++){
    if(i >= profile_time[3]) temp_reflow_individual[i] = (profile_temp[int(selected_profile)][4]-profile_temp[int(selected_profile)][3])/(profile_time[4]-profile_time[3]) * (i-profile_time[3]) + profile_temp[int(selected_profile)][3];
    else if(i >= profile_time[2]) temp_reflow_individual[i] = (profile_temp[int(selected_profile)][3]-profile_temp[int(selected_profile)][2])/(profile_time[3]-profile_time[2]) * (i-profile_time[2]) + profile_temp[int(selected_profile)][2];
    else if(i >= profile_time[1]) temp_reflow_individual[i] = (profile_temp[int(selected_profile)][2]-profile_temp[int(selected_profile)][1])/(profile_time[2]-profile_time[1]) * (i-profile_time[1]) + profile_temp[int(selected_profile)][1];
    else if(i >= profile_time[0]) temp_reflow_individual[i] = (profile_temp[int(selected_profile)][1]-profile_temp[int(selected_profile)][0])/(profile_time[1]-profile_time[0]) * (i-profile_time[0]) + profile_temp[int(selected_profile)][0];
  }
}


// Set used fonts with width and height of each character
struct fonts {
  const uint8_t* font;
  int width;
  int height;
};

fonts font_xl = {u8g2_font_profont29_mf, 16, 19};
fonts font_m = {u8g2_font_profont17_mf, 9, 11};
fonts font_s = {u8g2_font_profont12_mf, 6, 8};
fonts font_xs = {u8g2_font_profont10_mf, 5, 6};

// Custom type to set up a page on the OLED display. Baisc structure is a page of three fields.
// A top one with the power, a left one with the voltage and a right one with the current.
struct display_page {
  String title;
  float *value;
  String msg;
  bool is_value;
};

struct display_page_int {
  String title;
  int *value;
  String msg;
  bool is_value;
};

float val1 = 0.0;
float placeholder = 0.0;
String msg2display_top = "";
String msg2display_bot = "";

display_page page_start_reflow = {"Start Reflow", &placeholder, "Start Reflow", false};
display_page page_start_const = {"Start Const", &placeholder, "Start Const", false};
display_page page_select_profile = {"Set Profile", &selected_profile, "", true};
display_page page_const_temp = {"Const Temp", &temperature_setpoint_const, "", true};
display_page page_reflow_temp = {"Reflow Temp", &temperature_reflow, "", true};
display_page page_soak1_temp = {"Soak1 Temp", &temperature_soak1, "", true};
display_page page_soak2_temp = {"Soak2 Temp", &temperature_soak2, "", true};
display_page page_cooling_temp = {"Cooling Temp", &temperature_cooling, "", true};

// Menu and Pages
#define N_PAGES 8
int current_page = 0;
bool new_page = true;
unsigned long t_new_page = 0;

struct page {
  int p;
  display_page &page;
};

page pages[N_PAGES] = {
  {0, page_start_reflow},
  {1, page_select_profile},
  {2, page_start_const},
  {3, page_soak1_temp},
  {4, page_soak2_temp},
  {5, page_reflow_temp},
  {6, page_cooling_temp},
  {7, page_const_temp},
};

// Alignment for the OLED display
enum box_alignment {
  Center,
  Left,
  Right
};

// Custom type for setting up a box which can be drawn to the oled display. It is defined by
// the top left corner and the width and height of the box.
struct box {
  int32_t left, top, width, height;
  int32_t radius;
  bool has_frame;

  void draw(bool fill) {
    int frame = has_frame ? 4 : 0;
    if (fill) u8g2.drawRBox(left + frame, top + frame, width - (2*frame), height - (2*frame), radius);
    else u8g2.drawRFrame(left, top, width, height, radius);
  }

  void print_text(String str, fonts f, box_alignment align, float pos) {
    u8g2.setFont(f.font);
    int x = 0;
    if(align == Center) x = left + 0.5 * (width - (f.width * str.length() - 1));
    else if(align == Left) x = left + 5;
    else if(align == Right) x = left + width - f.width * str.length() - 5;
    u8g2.setCursor(x, top + pos * height + f.height / 2);
    u8g2.print(str);
  }
};

// Initialize the boxes for the OLED
box top_left = {0, 0, 36, 10, 0, false};
box top = {top_left.width, 0, 56, 10, 0, false};
box top_right = {top.left + top.width, 0, top_left.width, 10, 0, false};
box mid_main = {0, top.height, 128, 34, 0, false};
box bot = {0, mid_main.top + mid_main.height, 128, 10, 0, false};

// Update the OLED display with the three boxes according to the given pages
void update_display(){
  u8g2.clearBuffer();
  mid_main.draw(false);

  if(new_page){
    if(millis() <= t_new_page + 750) mid_main.print_text(pages[current_page].page.title, font_m, Center, 0.5);
    else new_page = false;
  }

  if(!new_page){
    if(pages[current_page].page.is_value){
      if(current_page == 1){ //set profile page
        mid_main.print_text(String(*pages[current_page].page.value, 0), font_m, Center, 0.5);
        msg2display_bot = String(profile_temp[int(selected_profile)][1], 0) + ", " + String(profile_temp[int(selected_profile)][2], 0) + ", " +
              String(profile_temp[int(selected_profile)][3], 0) + ", " + String(profile_temp[int(selected_profile)][4], 0);
      }
      else mid_main.print_text(String(*pages[current_page].page.value, 1) + "C", font_s, Center, 0.5);
    }
    else{
      if (temp1_error) {
        mid_main.print_text(("Cur Temp: ERROR C"), font_s, Left, 0.21);
      } else {
        mid_main.print_text(("Cur Temp: " + String(current_temperature1) + " C"), font_s, Left, 0.21);
      }
      mid_main.print_text(("Set Temp: " + String(pid_setpoint) + " C"), font_s, Left, 0.5);
      mid_main.print_text(("Progress: " + String(float(profile_counter)/3.0, 0) + " %"), font_s, Left, 0.8);
    }

    if (temp1_error) {
      top_left.print_text("ERROR C", font_xs, Center, 0.5);
    } else {
      top_left.print_text(String(current_temperature1, 0) + "C", font_xs, Center, 0.5);
    } 
    if (temp2_error) {
      top_right.print_text("ERROR C", font_xs, Center, 0.5);
    } else {
      top_right.print_text(String(current_temperature2, 0) + "C", font_xs, Center, 0.5);
    }
    bot.print_text(msg2display_bot, font_xs, Center, 0.5);
    top.print_text(msg2display_top, font_xs, Center, 0.5);
  }
  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  Serial.println("start");
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(CS_MAX1_PIN, OUTPUT);
  pinMode(CS_MAX2_PIN, OUTPUT);
  digitalWrite(CS_MAX1_PIN, LOW);
  digitalWrite(CS_MAX2_PIN, LOW);
  pinMode(SSR1_PIN, OUTPUT);
  digitalWrite(SSR1_PIN, LOW);
  delay(100);

  preferences.begin("reflowprofiles", false);
  delay(100);
  read_all_profiles();

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  u8g2.begin();
  delay(100);
  t_new_page = millis();
  update_display();
}

unsigned long t_thermo = millis();
unsigned long t_display = millis() + 100;
unsigned long t_pid_on = millis() + 400;
unsigned long t_start_reflow = millis();
unsigned long t_reflow_pid = millis() + 300;
unsigned long t_reflow_control = millis() + 500;

unsigned long t_const_pid = millis() + 300;
unsigned long t_const_control = millis() + 500;
unsigned long t_rampup = millis() + 600;
unsigned long t_profile_counter = millis() + 800;
unsigned long t_reflow_finish = millis() + 900;

void loop() {
  if(digitalRead(BTN1_PIN) == LOW){
    delay(50);

    if(pages[current_page].page.is_value){
      if(current_page == 1) {
        if(selected_profile > 0)
          *pages[current_page].page.value = *pages[current_page].page.value - 1.0;
      }
      else *pages[current_page].page.value = *pages[current_page].page.value - 5.0;
    }
    update_display();

    unsigned long t_btn_pressed = millis();
    int wait_time_increase = 300;
    bool set_process = false;

    while (digitalRead(BTN1_PIN) == LOW){
      if(millis() >= t_btn_pressed + wait_time_increase && !set_process){
        wait_time_increase = 80;
        t_btn_pressed = millis();

        if(pages[current_page].page.is_value){
          if(current_page == 1) {
            if(selected_profile > 0)
              *pages[current_page].page.value = *pages[current_page].page.value - 1.0;
          }
          else *pages[current_page].page.value = *pages[current_page].page.value - 5.0;
        }
        else{
          if(current_page == 0) {
            running_reflow = !running_reflow;
            init_reflow = true;
            msg2display_top = running_reflow ? "Reflow" : "";
          }
          else if(current_page == 1){
           running_const = !running_const;
           init_const = true;
           msg2display_top = running_const ? "Const" : "";
          }
          set_process = true;
        }
        update_display();
      }
      delay(20);
    }
  }
  else if(digitalRead(BTN2_PIN) == LOW){
    delay(50);

    if(pages[current_page].page.is_value){
      if(current_page == 1) {
        if(selected_profile < N_PROFILES-1)
          *pages[current_page].page.value = *pages[current_page].page.value + 1.0;
      }
      else *pages[current_page].page.value = *pages[current_page].page.value + 5.0;
    }

    update_display();
    unsigned long t_btn_pressed = millis();
    int wait_time_increase = 300;
    bool set_process = false;
    
    while (digitalRead(BTN2_PIN) == LOW){
      if(millis() >= t_btn_pressed + wait_time_increase && !set_process){

        wait_time_increase = 80;
        t_btn_pressed = millis();

        if(pages[current_page].page.is_value){
          if(current_page == 1) {
            if(selected_profile < N_PROFILES-1)
              *pages[current_page].page.value = *pages[current_page].page.value + 1.0;
          }
          else *pages[current_page].page.value = *pages[current_page].page.value + 5.0;
        }
        else{
          if(current_page == 0) {
            running_reflow = !running_reflow;
            init_reflow = true;
            msg2display_top = running_reflow ? "Reflow" : "";
          }
          else if(current_page == 1){
            running_const = !running_const;
            init_const = true;
            msg2display_top = running_const ? "Const" : "";
          }
          set_process = true;
        }
        update_display();
      }
      delay(20);
    }
  }
  else if(digitalRead(BTN3_PIN) == LOW){
    delay(50);
    t_new_page = millis();
    new_page = true;
    current_page = (++current_page) % N_PAGES;
    msg2display_bot = "";
    update_display();

    unsigned long t_btn_pressed = millis();
    while (digitalRead(BTN3_PIN) == LOW){
      if(millis() >= t_btn_pressed + 800){
      }
      delay(20);
    }
  }



  if(init_reflow){
    init_reflow = false;

    if(running_reflow){
      t_start_reflow = millis();
      profile_counter = 0;
      preheat_done = false;
      reflow_done = false;
      calculate_profile_individual();
      pid_setpoint = temp_reflow_individual[0];

      pid_reflow.SetOutputLimits(0, 500);
      pid_reflow.SetMode(AUTOMATIC);
    }
    else{
      msg2display_bot = "";
      digitalWrite(SSR1_PIN, LOW);
    }
  }


  if(running_reflow && !preheat_done){
    if(current_temperature1 < 30){
      if(millis() >= t_reflow_control + 500){
        t_reflow_control = millis();
        digitalWrite(SSR1_PIN, !digitalRead(SSR1_PIN));
        t_pid_on = millis();
      }
    }
    else{
      preheat_done = true;
      t_profile_counter = millis();
    }
  }


  while(reflow_done){
    if(millis() > t_reflow_finish + 1000){
      t_reflow_finish = millis();
      buzzer_state = !buzzer_state;
      ledcWrite(PWMChannel, buzzer_state ? 500 : 0);
      msg2display_bot = "Press any button!";
      update_display();
    }

    if(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW){
      reflow_done = false;
      ledcWrite(PWMChannel, 0);

      profile_counter = 0;
      pid_setpoint = temp_reflow_individual[0];
      
      while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW){ 
        delay(100); 
      }
    }
  }

  

  if(running_reflow && preheat_done){
    int pid_error = int(pid_setpoint) - int(current_temperature1);

    if(millis() >= t_reflow_pid + 200){
      t_const_pid = millis();
      pid_setpoint = temp_reflow_individual[profile_counter];
      pid_input = current_temperature1;
      pid_reflow.Compute();
      msg2display_bot = String(pid_output, 0) + ", " + String(pid_setpoint, 0) + ", " + String(pid_error) + ", " + String(profile_counter) + "/300";// + ", " + String(window_start_time);
    }

    if(millis() >= t_reflow_control + 500){
      t_reflow_control = millis();
      
      if(pid_output > 50){
        digitalWrite(SSR1_PIN, HIGH);
        t_pid_on = millis();
      }
    }

    if(millis() > t_pid_on + pid_output && digitalRead(SSR1_PIN)){
      digitalWrite(SSR1_PIN, LOW);
    }

    if(millis() >= t_profile_counter + 1000){
      t_profile_counter += 1000;
      profile_counter++;
      if(profile_counter > 300){
        ledcWrite(PWMChannel, 500);
        msg2display_top = "";
        buzzer_state = true;
        running_reflow = false;
        t_reflow_finish = millis();
        reflow_done = true;
        digitalWrite(SSR1_PIN, LOW);
      }
    }
  }


  if(init_const){
    init_const = false;

    if(running_const){
      pid_setpoint = temperature_setpoint_const;
      rampup_const = true;
      pid.SetOutputLimits(0, 1000);
      pid.SetMode(AUTOMATIC);
    }
    else{
      msg2display_bot = "";
      digitalWrite(SSR1_PIN, LOW);
    }
  }


  if(running_const){
    int pid_error = int(pid_setpoint) - int(current_temperature1);

    if(pid_error < 5 && pid_error > -5 && rampup_const_close){
      rampup_const_close = false;
      t_rampup = millis();
    }

    if(millis() >= t_const_pid + 200){
      t_const_pid = millis();
      pid_input = current_temperature1;
      pid.Compute();
      msg2display_bot = String(pid_output, 0) + ", " + String(pid_error, 0) + ", " + String(rampup_const, 0);// + ", " + String(window_start_time);
    }

    if(millis() >= t_const_control + 2000){
      t_const_control = millis();
      
      if(rampup_const){
        if(pid_error < 0){
          rampup_const = false;
          pid.SetOutputLimits(0, 300);
          ledcWrite(PWMChannel, 500);
          delay(200);
          ledcWrite(PWMChannel, 0); 
        }
        else if(pid_error < 10){
          pid.SetOutputLimits(0, 150);
        }
        else if(pid_error < 20){
          pid.SetOutputLimits(0, 300);
        }

        if(millis() > t_rampup + 10000 && !rampup_const_close){
          rampup_const = false;
          pid.SetOutputLimits(0, 300);

          ledcWrite(PWMChannel, 500);
          delay(200);
          ledcWrite(PWMChannel, 0); 
        }
      }
      
      if(pid_output > 50){
        digitalWrite(SSR1_PIN, HIGH);
        t_pid_on = millis();
      }
    }

    if(millis() > t_pid_on + pid_output && digitalRead(SSR1_PIN)){
      digitalWrite(SSR1_PIN, LOW);
    }
  }



  if(millis() >= t_display + 500){
    t_display = millis();
    update_display();
  }

  
  if(millis() >= t_thermo + 200){
    t_thermo = millis();
    float temp1, temp2;

    temp1 = thermocouple1.getTemperature();
    temp2 = thermocouple2.getTemperature();

    temp1_error = isnanf(temp1);
    temp2_error = isnanf(temp2);
    
    if (!temp1_error) {
      current_temperature1 = thermocouple1.getTemperature();//random(50, 300);//
    }
    if (!temp2_error) {
      current_temperature2 = thermocouple2.getTemperature();//random(50, 300);//thermocouple2.readCelsius();
    }

    val1 = float(random(50,300));
  }
}
