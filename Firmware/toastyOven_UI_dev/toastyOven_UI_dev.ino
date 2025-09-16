#include <Arduino.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>
#include "Adafruit_MAX31855.h"
#include <SimpleRotary.h>
#include <PID_v1_bc.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#define BIT(a) (1 << (a))
// SSR Pin
#define SSRpin 9 // D9 - PortB pin 2
//#define SSRpin 2 // D9 - PortB pin 2

// LED pins
#define statusLEDpin 7 // D7 - PortD pin 7
#define alertLEDpin 12 // D12 - PortB pin 4

// Buzzer Pins
#define buzzerPin 7 // D6 - PortD pin 6

// Quadrature Encoder Pins
#define A_CLK   15 //A0 (15), PortC pin 0
#define B_DT   14 //A1 (14), PortC pin 1
#define encoder_switch 8 //D8, PortB pin0

// Pin A, Pin B, Button Pin
SimpleRotary rotary(A_CLK,B_DT,encoder_switch);

// TC Amplifier Pins
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

float tempC = 0;
float tempF = 0;
uint8_t tcErrorMsg;
uint8_t is_tc_running = 1;// defines the current state of the tc amp: running or not running


// TC IC Constructor
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// PID & Reflow parameters //

//double preheatTemp = 180, soakTemp = 150, reflowTemp = 230, cooldownTemp = 25;
//float preheatTime = 120000, soakTime = 60000, reflowTime = 120000, cooldownTime = 120000, totalTime = preheatTime + soakTime + reflowTime + cooldownTime;

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

int Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, (double) Kp, (double) Ki,(double) Kd, DIRECT);

// LCD Constructor
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* CS=*/ 10, /* reset=*/ 16);

// U8g2 User Interface
MUIU8G2 mui;

uint8_t is_redraw = 1;
//uint8_t rotary_event = 0; // 0 = not turning, 1 = CW, 2 = CCW
uint8_t push_event = 0; // 0 = not pushed, 1 = pushed

// Profile Selection List & Reflow Graph
#define maxgraph_width 80
#define maxgraph_height 42
uint8_t profileNUM = 0; // 0 = profile 1, 1 = profile 2 and 2 = profile 3
uint8_t is_profiledisplay_running = 0;// defines the current state of the profile: running or not running
uint8_t is_reflow_running = 0;
float time_reflow_started = 0;
float time_since_reflow = 0;

uint8_t reflowState = 0; // 0 OFF, 1 preheat, 2 soak, 3 peak Climb, 4 Peak, 5 cooldown, 6 Done

// Time of each profile steptype in Milliseconds
float preheat_time = 120000;
float soak_time = 240000;
float peakclimb_time = 300000;
float peak_time = 330000;
float cooldown_time = 430000;

// Temp of each profile steptype in C
double preheat_temp = 150;
double soak_temp = 194;
double peakclimb_temp = 250;
double peak_temp = 250;
double cooldown_temp = 50;

uint8_t pixel_t1, pixel_t2, pixel_t3, pixel_t4, pixel_t5;
uint8_t pixel_temp1, pixel_temp2, pixel_temp3, pixel_temp4, pixel_temp5;
uint8_t pixel_currentTime=0;

float startTemp_reflow = 0;
// float graph_frame = 0;
// float graph_start_frame = 0;

#define PROFILE_MAX_TIME 255
#define PROFILE_MAX_TEMP 255


uint8_t muif_reflowprofile_init(mui_t *ui, uint8_t msg){
  uint8_t return_value = 0; 
  switch(msg){
    case MUIF_MSG_FORM_START:
      time_reflow_started = millis();
      //graph_start_frame = millis();
      is_reflow_running = 1;
      startTemp_reflow = tempC;

      pixel_t1 = (int)(preheat_time*(maxgraph_width/cooldown_time))+3;
      pixel_t2 = (int)(soak_time*(maxgraph_width/cooldown_time))+3;
      pixel_t3 = (int)(peakclimb_time*(maxgraph_width/cooldown_time))+3;
      pixel_t4 = (int)(peak_time*(maxgraph_width/cooldown_time))+3;
      pixel_t5 = (int)(cooldown_time*(maxgraph_width/cooldown_time))+3;

      pixel_temp1 = (int) (maxgraph_height+7)-(preheat_temp*((maxgraph_height+3) /peak_temp));
      pixel_temp2 = (int) (maxgraph_height+7)-(soak_temp*((maxgraph_height+3) /peak_temp));
      pixel_temp3 = (int) (maxgraph_height+7)-(peakclimb_temp*((maxgraph_height+3) /peak_temp));
      pixel_temp4 = (int) (maxgraph_height+7)-(peak_temp*((maxgraph_height+3) /peak_temp));
      pixel_temp5 = (int) (maxgraph_height+7)-(cooldown_temp*((maxgraph_height+3) /peak_temp));

      myPID.SetOutputLimits(0, 1);

      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // call the original MUIF
      break;
    case MUIF_MSG_FORM_END:
      time_reflow_started = 0;
      is_reflow_running = 0;
      reflowState = 0;
      // turn the PID OFF
      myPID.SetMode(MANUAL);
      digitalWrite(SSRpin,LOW);
      //PORTB = ~( (~PORTB) | BIT(SSRpin) );
      digitalWrite(buzzerPin,LOW);
      //PORTD = ~( (~PORTD) | BIT(buzzerPin) );

      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // finalise the form
      break;
    default:
      //digitalWrite(SSRpin,LOW);
      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // for any other messages, just call the original MUIF
  }
  return return_value;
}

void run_reflowprofile(void){
  uint8_t reminder = 0; 

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  time_since_reflow = millis() - time_reflow_started;

  Input = tempC;

  if(time_since_reflow <= 500){
    digitalWrite(buzzerPin,HIGH);
    //PORTD |= BIT(buzzerPin);
  } else{
    digitalWrite(buzzerPin,LOW);
    //PORTD = ~( (~PORTD) | BIT(buzzerPin) );
  }

  // Pre-Heat Cycle
  if(time_since_reflow <= preheat_time){
    Setpoint = preheat_temp * (time_since_reflow / preheat_time);
    reflowState = 1;
  // Soak Cycle
  } else if( (time_since_reflow > preheat_time) && (time_since_reflow <= soak_time)) { 
    Setpoint = soak_temp * (time_since_reflow / soak_time);
    reflowState = 2;
  // Peak Climb Cycle
  } else if( (time_since_reflow > soak_time) && (time_since_reflow <= peakclimb_time)) { 
    Setpoint = peakclimb_temp * (time_since_reflow / peakclimb_time);
    reflowState = 3;
  // Peak Cycle
  } else if( (time_since_reflow > peakclimb_time) && (time_since_reflow <= peak_time)) { 
    Setpoint = peak_temp;
    reflowState = 4;
  // Cooldown Cycle
  } else if( (time_since_reflow > peak_time) && (time_since_reflow <= cooldown_time)) { 
    Setpoint = cooldown_temp;
    reflowState = 5;

    if(time_since_reflow >= cooldown_time - 500 ){
      digitalWrite(buzzerPin,HIGH);
      //PORTD |= BIT(buzzerPin);
    } else{
      digitalWrite(buzzerPin,LOW);
      //PORTD = ~( (~PORTD) | BIT(buzzerPin) );
    }
  } else {
    Setpoint = 0;
    reflowState = 6;
    is_reflow_running = 0;
  }

  pixel_currentTime = (int)(time_since_reflow*(maxgraph_width/cooldown_time))+3;


  myPID.Compute();
  if(Output < 0.5){
    digitalWrite(SSRpin,LOW);
    //PORTB = ~( (~PORTB) | BIT(SSRpin) );
  }
  if(Output > 0.5){
    digitalWrite(SSRpin,HIGH);
    //PORTB = ~( (~PORTB) | BIT(SSRpin) );
  }


}



// Detect Encoder rotation or button press
void detect_events(void) {
  uint8_t tmp;
  
  // 0 = not pushed, 1 = pushed  
  tmp = rotary.push();
  if ( tmp != 0 )         // only assign the push event, never clear the event here
    push_event = tmp;
    
  // 0 = not turning, 1 = CW, 2 = CCW
  // tmp = rotary.rotate();
  // if ( tmp != 0 )       // only assign the rotation event, never clear the event here
  //   rotary_event = tmp;    
}

// Execute Events
void handle_events(void) {
  // 0 = not pushed, 1 = pushed  
  if ( push_event == 1 ) {
      mui.sendSelect();
      is_redraw = 1;
      push_event = 0;
  }
  
  // // 0 = not turning, 1 = CW, 2 = CCW
  // if ( rotary_event == 1 ) {
  //   mui.nextField();
  //   is_redraw = 1;
  //   rotary_event = 0;
  // }
  
  // if ( rotary_event == 2 ) {
  //   mui.prevField();
  //   is_redraw = 1;
  //   rotary_event = 0;
  // }

  // Update Temperature Readings
  if(is_tc_running){
    tempC = thermocouple.readCelsius();
    tempF = (tempC* 1.8) + 32;
    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(tempC);
    // Serial.print(",");
    // Serial.println(Setpoint);
    is_redraw = 1;
  }

  // Update Profile display number
  if(is_profiledisplay_running){
    is_redraw = 1;
  }

  if(is_reflow_running){
    run_reflowprofile();
    is_redraw = 1;
  }
  
}

/* draw the current profile value */
uint8_t mui_draw_current_profiledisplay(mui_t *ui, uint8_t msg) {
  if ( msg == MUIF_MSG_DRAW   ) {
      u8g2.setCursor(mui_get_x(ui), mui_get_y(ui));
      u8g2.print(profileNUM+1);
      is_profiledisplay_running = 0;
  }
  return 0;
}

/* profile display */
uint8_t mui_start_current_profiledisplay(mui_t *ui, uint8_t msg) {
  if ( msg == MUIF_MSG_FORM_START ) {
      is_profiledisplay_running = 1;
  }
  return 0;
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


/* draw the current temperature value */
uint8_t mui_draw_current_time(mui_t *ui, uint8_t msg) {
  if ( msg == MUIF_MSG_DRAW   ) {
      u8g2.setCursor(mui_get_x(ui), mui_get_y(ui));
      u8g2.print(Setpoint,1);
      u8g2.print("C / ");
      u8g2.print((Setpoint* 1.8) + 32,1);
      u8g2.print("F");

      u8g2.setCursor(mui_get_x(ui)+61, mui_get_y(ui)+9);
      u8g2.print("Time: ");
      u8g2.print(time_since_reflow/1000,1);
      u8g2.print(" S");

      u8g2.setCursor(mui_get_x(ui)+76, mui_get_y(ui)-45);
      u8g2.print("Profile ");
      u8g2.print(profileNUM+1);

      u8g2.setCursor(mui_get_x(ui)+70, mui_get_y(ui)-30);
      switch(reflowState){
        case 1:
          u8g2.print("Preheating");
          break;
        case 2:
          u8g2.print(" Soaking");
          break;
        case 3:
          u8g2.print("Climbing");
          break;
        case 4:
          u8g2.print(" Peak ");
          break;
        case 5:
          u8g2.print("Cooldown");
          break;
        case 6:
          u8g2.print(" Done ");
          break;
      }

      u8g2.drawFrame(2,2,84,46);


      u8g2.drawLine(pixel_currentTime,46,pixel_currentTime, 2); // Time line

      u8g2.drawLine(3,45,pixel_t1, pixel_temp1); // pre-heat climb
      u8g2.drawLine(pixel_t1,pixel_temp1,pixel_t2,pixel_temp2); // Soaking climb
      u8g2.drawLine(pixel_t2,pixel_temp2,pixel_t3,pixel_temp3); // Climb to Peak
      u8g2.drawLine(pixel_t3,pixel_temp3,pixel_t4,pixel_temp4); // Peak Temp
      u8g2.drawLine(pixel_t4,pixel_temp4,pixel_t5,pixel_temp5); // Cooldown

  }
  return 0;
}


// MUIF List
muif_t muif_list[]  MUI_PROGMEM = {

  MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tr),        /* regular font 8 pixel */
  MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_tr),        /* bold font 8 pixel*/
  // MUIF_U8G2_FONT_STYLE(2, u8g2_font_tiny5_tf),        /* tiny 6 pixel */
  // MUIF_U8G2_FONT_STYLE(3, u8g2_font_6x10_tf),        /* tiny 7 pixel */

  MUIF_U8G2_FONT_STYLE(2, u8g2_font_tinyface_tr ),        /* tiny 5 pixel */
  MUIF_U8G2_FONT_STYLE(3, u8g2_font_blipfest_07_tr  ),        /* tiny 5 pixel */

  // Temperature Related MUIFs
  MUIF_RO("TD", mui_draw_current_temp), // Draws updated temp values
  MUIF_RO("TC", mui_start_current_temp), // Starts tc readings
  
  // Main Menu Related MUIFs
  MUIF_U8G2_LABEL(),                          // General Label *

  // Reflow Menu MUIFs
  MUIF_BUTTON("B1", mui_u8g2_btn_goto_wm_fi), // Main Menu Back Button *
  MUIF_BUTTON("ST", mui_u8g2_btn_goto_wm_fi), // Button to start

  // Temperature Related MUIFs
  MUIF_RO("PD", mui_draw_current_profiledisplay), // Draws updated profile values
  MUIF_RO("PS", mui_start_current_profiledisplay), // Starts profile

  // Reflow MUIFs
  MUIF_RO("TI", mui_draw_current_time), // Draws setpoint & time values
  MUIF_BUTTON("SO", muif_reflowprofile_init), // Button to Stop Reflow

};

// FDS Data
fds_t fds_data[] = 

// Reflow Menu
MUI_FORM(1)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_STYLE(1)
MUI_LABEL(20, 9, "Toasty Oven V1.0")
MUI_STYLE(3)
MUI_XYAT("ST", 46, 25, 3, " Start Reflow ")


// Reflow Process Menu
MUI_FORM(3)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_LABEL(2, 54, "Set: ")
MUI_LABEL(2, 63, "Cur: ")
MUI_XY("TD", 18, 63) // Postion of temp values
MUI_XY("TI", 18, 54) // Postion of time values

MUI_STYLE(0)
MUI_XYAT("SO", 107, 40, 1, " STOP ") 

;


void setup() {
  //Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 9 as an output
  pinMode(SSRpin, OUTPUT); // Set ssr - pin 9 as an output
  pinMode(statusLEDpin, OUTPUT); // Set buzzer - pin 9 as an output
  pinMode(alertLEDpin, OUTPUT); // Set buzzer - pin 9 as an output
  digitalWrite(SSRpin, LOW);
  digitalWrite(statusLEDpin, LOW);
  digitalWrite(alertLEDpin, LOW);

  digitalWrite(buzzerPin, HIGH);
  //PORTD |= BIT(buzzerPin);
  delay(250);
  digitalWrite(buzzerPin, LOW);
  //PORTD = ~( (~PORTD) | BIT(buzzerPin) );
  

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

    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(tempC);
    // Serial.print(",");
    // Serial.println(Setpoint);

    detect_events();
    handle_events();
      
  } else {
      /* the menu should never become inactive, but if so, then restart the menu system */
      mui.gotoForm(/* form_id= */ 1, /* initial_cursor_position= */ 0);
  }
}
  
  

  
