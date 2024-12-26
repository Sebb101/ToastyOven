#include <Arduino.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>
#include "Adafruit_MAX31855.h"
#include <SimpleRotary.h>
#include <PID_v1_bc.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

// Buzzer Pins
#define buzzerPin 6

// Quadrature Encoder Pins
#define A_CLK   15 //A0
#define B_DT   14 //A1
#define encoder_switch 8 //D8

// Pin A, Pin B, Button Pin
SimpleRotary rotary(A_CLK,B_DT,encoder_switch);

// TC Amplifier Pins
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

float tempC = 0;
float tempF = 0;
uint8_t tcErrorMsg;
uint8_t is_tc_running = 1;// defines the current state of the stop watch: running or not running


// TC IC Constructor
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// PID & Reflow parameters //

double preheatTemp = 180, soakTemp = 150, reflowTemp = 230, cooldownTemp = 25;
float preheatTime = 120000, soakTime = 60000, reflowTime = 120000, cooldownTime = 120000, totalTime = preheatTime + soakTime + reflowTime + cooldownTime;

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// LCD Constructor
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* CS=*/ 10, /* reset=*/ 8);

// U8g2 User Interface
MUIU8G2 mui;

uint8_t is_redraw = 1;
uint8_t rotary_event = 0; // 0 = not turning, 1 = CW, 2 = CCW
uint8_t push_event = 0; // 0 = not pushed, 1 = pushed

// Profile Selection List
uint8_t profileNUM = 0; // 0 = profile 1, 1 = profile 2 and 2 = profile 3

// Detect Encoder rotation or button press
void detect_events(void) {
  uint8_t tmp;
  
  // 0 = not pushed, 1 = pushed  
  tmp = rotary.push();
  if ( tmp != 0 )         // only assign the push event, never clear the event here
    push_event = tmp;
    
  // 0 = not turning, 1 = CW, 2 = CCW
  tmp = rotary.rotate();
  if ( tmp != 0 )       // only assign the rotation event, never clear the event here
    rotary_event = tmp;    
}

// Execute Events
void handle_events(void) {
  // 0 = not pushed, 1 = pushed  
  if ( push_event == 1 ) {
      mui.sendSelect();
      is_redraw = 1;
      push_event = 0;
  }
  
  // 0 = not turning, 1 = CW, 2 = CCW
  if ( rotary_event == 1 ) {
    mui.nextField();
    is_redraw = 1;
    rotary_event = 0;
  }
  
  if ( rotary_event == 2 ) {
    mui.prevField();
    is_redraw = 1;
    rotary_event = 0;
  }

  // Update Temperature Readings
  if(is_tc_running){
    tempC = thermocouple.readCelsius();
    tempF = (tempC* 1.8) + 32;
    is_redraw = 1;
  }
  
}

/* draw the current temperature value */
uint8_t mui_draw_current_temp(mui_t *ui, uint8_t msg) {
  if ( msg == MUIF_MSG_DRAW   ) {
      u8g2.setCursor(mui_get_x(ui), mui_get_y(ui));
      u8g2.print(tempC,1);
      u8g2.print("C / ");
      u8g2.print(tempF,1);
      u8g2.print("F");
  }
  return 0;
}

/* start the TC IC */
uint8_t mui_start_current_temp(mui_t *ui, uint8_t msg) {
  if ( msg == MUIF_MSG_FORM_START ) {
      is_tc_running = 1;
  }
  return 0;
}


// MUIF List
muif_t muif_list[]  MUI_PROGMEM = {

  MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tr),        /* regular font 8 pixel */
  MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_tr),        /* bold font 8 pixel*/
  MUIF_U8G2_FONT_STYLE(2, u8g2_font_tiny5_tf),        /* tiny 6 pixel */
  MUIF_U8G2_FONT_STYLE(3, u8g2_font_6x10_tf),        /* tiny 7 pixel */

  // Temperature Related MUIFs
  MUIF_RO("TD", mui_draw_current_temp), // Draws updated temp values
  MUIF_RO("TC", mui_start_current_temp), // Starts tc readings
  
  // Main Menu Related MUIFs
  MUIF_BUTTON("RF", mui_u8g2_btn_goto_wm_fi), // Button to Reflow Menu
  MUIF_U8G2_LABEL(),                          // General Label *
  MUIF_BUTTON("TT", mui_u8g2_btn_goto_wm_fi), // Button to Config

  // Reflow Menu MUIFs
  MUIF_BUTTON("BK", mui_u8g2_btn_back_wm_fi), // General Back Button *
  MUIF_BUTTON("ST", mui_u8g2_btn_goto_wm_fi), // Button to start
  MUIF_BUTTON("CF", mui_u8g2_btn_goto_wm_fi), // Button to Config Profiles
  MUIF_BUTTON("PI", mui_u8g2_btn_goto_wm_fi), // Button to PID Settings
  MUIF_VARIABLE("PF",&profileNUM, mui_u8g2_u8_opt_line_wa_mud_pf)
};

// FDS Data
fds_t fds_data[] = 

MUI_FORM(1)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_STYLE(1)
MUI_LABEL(20, 9, "Toasty Oven dev")
MUI_STYLE(0)
MUI_XYAT("RF", 40, 25, 2, " Reflow Menu ") 
MUI_XYAT("TT", 39, 42, 1, " Thermal Test ")


MUI_FORM(2)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_STYLE(3)
MUI_XYAT("ST", 46, 15, 2, " Start Profile")
MUI_XYAT("PF", 92, 15, 0, " 1 | 2 | 3 ")
MUI_XYAT("CF", 52, 30, 2, " Config Profile ")
MUI_XYT("BK", 22, 45, " Back ") 
MUI_XYAT("PI", 85, 45, 2, "PID Settings")


;


void setup() {

  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 9 as an output
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);

  u8g2.begin();
  mui.begin(u8g2, fds_data, muif_list, sizeof(muif_list)/sizeof(muif_t));
  mui.gotoForm(/* form_id= */ 1, /* initial_cursor_position= */ 0);
  
}

void loop() {

  /* check whether the menu is active */
  if ( mui.isFormActive() ) {

    /* update the display content, if the redraw flag is set */
    if ( is_redraw ) {
      u8g2.firstPage();
      do {
          detect_events();
          mui.draw();
          detect_events();
      } while( u8g2.nextPage() );
      is_redraw = 0;                    /* clear the redraw flag */
    }

    detect_events();
    handle_events();
      
  } else {
      /* the menu should never become inactive, but if so, then restart the menu system */
      mui.gotoForm(/* form_id= */ 1, /* initial_cursor_position= */ 0);
  }
}
  
