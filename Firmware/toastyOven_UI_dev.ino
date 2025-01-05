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
//#define SSRpin 9 // D9 - PortB pin 2
#define SSRpin 2 // D9 - PortB pin 2

// LED pins
#define statusLEDpin 7 // D7 - PortD pin 7
#define alertLEDpin 12 // D12 - PortB pin 4

// Buzzer Pins
#define buzzerPin 6 // D6 - PortD pin 6

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

double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// LCD Constructor
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* CS=*/ 10, /* reset=*/ 16);

// U8g2 User Interface
MUIU8G2 mui;

uint8_t is_redraw = 1;
uint8_t rotary_event = 0; // 0 = not turning, 1 = CW, 2 = CCW
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

float preheat_time, soak_time, peakclimb_time, peak_time, cooldown_time;
double preheat_temp, soak_temp, peakclimb_temp, peak_temp, cooldown_temp;

uint8_t pixel_t1, pixel_t2, pixel_t3, pixel_t4, pixel_t5;
uint8_t pixel_temp1, pixel_temp2, pixel_temp3, pixel_temp4, pixel_temp5;
uint8_t pixel_graphline=3;
//uint8_t graph_array[2][80];
// uint8_t graph_index = 0;

// uint8_t time_pixelcoord = 3;
// uint8_t temp_pixelcoord = 3;

float startTemp_reflow = 0;
// float graph_frame = 0;
// float graph_start_frame = 0;

#define PROFILE_MAX_TIME 255
#define PROFILE_MAX_TEMP 255

struct array_element_struct
{
  uint8_t time1;
  uint8_t temp1;
  uint8_t time2;
  uint8_t temp2;  
  uint8_t time3;
  uint8_t temp3;  
  uint8_t time4;
  uint8_t temp4;  
  uint8_t time5;
  uint8_t temp5;    
};

/* array list  */

#define PROFILE_ELEMENT_CNT 2
struct array_element_struct profile_array[PROFILE_ELEMENT_CNT];

/* array editable local copy */

volatile uint8_t array_edit_pos = 0;                            // "volatile" might not be required, but still; array_edit_pos is modified by MUI callbacks and used in the extended MUIF
struct array_element_struct edit_profile_array_element;

// Initialize All Profile time and temp values
void profile_array_init(void)
{
  uint8_t i;
  for( i = 0; i < PROFILE_ELEMENT_CNT; i++ )
  {
      profile_array[i].time1 = 12;
      profile_array[i].temp1 = 150;
      profile_array[i].time2 = 24;
      profile_array[i].temp2 = 194;
      profile_array[i].time3 = 30;
      profile_array[i].temp3 = 250;
      profile_array[i].time4 = 33;
      profile_array[i].temp4 = 250;
      profile_array[i].time5 = 43;
      profile_array[i].temp5 = 50;
  }  
}

uint8_t muif_reflowprofile_init(mui_t *ui, uint8_t msg){
  uint8_t return_value = 0; 
  switch(msg){
    case MUIF_MSG_FORM_START:
      time_reflow_started = millis();
      //graph_start_frame = millis();
      is_reflow_running = 1;
      startTemp_reflow = tempC;

      // for(int i=0;i<80;i++){
      //   graph_array[0][i] = 3;
      //   graph_array[1][i] = 3;
      // }

      // Assign selected profile over to actual parameters and convert from hs to ms
      preheat_time = ((float) profile_array[profileNUM].time1) * 10000;
      soak_time = ((float) profile_array[profileNUM].time2) * 10000;
      peakclimb_time = ((float) profile_array[profileNUM].time3) * 10000;
      peak_time = ((float) profile_array[profileNUM].time4) * 10000;
      cooldown_time = ((float) profile_array[profileNUM].time5) * 10000;

      preheat_temp = (double) profile_array[profileNUM].temp1;
      soak_temp = (double) profile_array[profileNUM].temp2;
      peakclimb_temp = (double) profile_array[profileNUM].temp3;
      peak_temp = (double) profile_array[profileNUM].temp4;
      cooldown_temp = (double) profile_array[profileNUM].temp5;

      pixel_t1 = (int)(preheat_time*(maxgraph_width/cooldown_time))+3;
      pixel_t2 = (int)(soak_time*(maxgraph_width/cooldown_time))+3;
      pixel_t3 = (int)(peakclimb_time*(maxgraph_width/cooldown_time))+3;
      pixel_t4 = (int)(peak_time*(maxgraph_width/cooldown_time))+3;
      pixel_t5 = (int)(cooldown_time*(maxgraph_width/cooldown_time))+3;

      pixel_temp1 = (int) (maxgraph_height+5)-(preheat_temp*((maxgraph_height+3) /cooldown_temp));
      pixel_temp2 = (int) (maxgraph_height+5)-(soak_temp*((maxgraph_height+3) /cooldown_temp));
      pixel_temp3 = (int) (maxgraph_height+5)-(peakclimb_temp*((maxgraph_height+3) /cooldown_temp));
      pixel_temp4 = (int) (maxgraph_height+5)-(peak_temp*((maxgraph_height+3) /cooldown_temp));
      pixel_temp5 = (int) (maxgraph_height+5)-(cooldown_temp*((maxgraph_height+3) /cooldown_temp));

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
    Setpoint = preheat_temp;
    reflowState = 1;
  // Soak Cycle
  } else if( (time_since_reflow > preheat_time) && (time_since_reflow <= soak_time)) { 
    Setpoint = soak_temp;
    reflowState = 2;
  // Peak Climb Cycle
  } else if( (time_since_reflow > soak_time) && (time_since_reflow <= peakclimb_time)) { 
    Setpoint = peakclimb_temp;
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

  myPID.Compute();
  if(Output < 0.5){
    digitalWrite(SSRpin,LOW);
    //PORTB = ~( (~PORTB) | BIT(SSRpin) );
  }
  if(Output > 0.5){
    digitalWrite(SSRpin,HIGH);
    //PORTB = ~( (~PORTB) | BIT(SSRpin) );
  }

  // graph_frame = millis() - graph_start_frame;
  // if(graph_frame >= 5000) {
  //   graph_start_frame = millis();
  //   time_pixelcoord = graph_index + 3;

  //   if(((int)(tempC-startTemp_reflow))%10 >=5){
  //     temp_pixelcoord = ( (int)((tempC-startTemp_reflow)/10) ) + 1;
  //   } else {
  //     temp_pixelcoord = ( (int)((tempC-startTemp_reflow)/10) );
  //   }

  //   graph_array[0][graph_index] = time_pixelcoord;
  //   graph_array[1][graph_index] = temp_pixelcoord;
  //   if(graph_index <=80){
  //       graph_index++;
  //   }
  // }

}

// For Config profile menu, saves or resets edit values on screen
uint8_t muif_profilearray_edit_values(mui_t *ui, uint8_t msg){
  uint8_t return_value = 0; 
  switch(msg)
  {
    case MUIF_MSG_FORM_START:
      edit_profile_array_element = profile_array[profileNUM];                                          // copy local array element to the local editable copy
      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // call the original MUIF
      break;
    case MUIF_MSG_FORM_END:
      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // finalise the form
      profile_array[profileNUM] = edit_profile_array_element;                                         // store the current elements in the array before leaving the form
      break;
    default:
      return_value = mui_u8g2_btn_goto_wm_fi(ui, msg);                    // for any other messages, just call the original MUIF
  }
  return return_value;
}


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



      u8g2.drawLine(3,45,pixel_t1,pixel_temp1);
      u8g2.drawLine(pixel_t1,pixel_temp1,pixel_t2,pixel_temp2);
      u8g2.drawLine(pixel_t2,pixel_temp2,pixel_t3,pixel_temp3);
      u8g2.drawLine(pixel_t3,pixel_temp3,pixel_t4,pixel_temp4);
      u8g2.drawLine(pixel_t4,pixel_temp4,pixel_t4,pixel_temp4);

      // for(int i=0;i<80;i++){
      //   u8g2.drawPixel(graph_array[0][i], graph_array[1][i]);
      // }


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
  MUIF_BUTTON("B1", mui_u8g2_btn_goto_wm_fi), // Main Menu Back Button *
  MUIF_BUTTON("ST", mui_u8g2_btn_goto_wm_fi), // Button to start
  MUIF_BUTTON("CF", mui_u8g2_btn_goto_wm_fi), // Button to Config Profiles
  //MUIF_BUTTON("PI", mui_u8g2_btn_goto_wm_fi), // Button to PID Settings
  MUIF_VARIABLE("PF",&profileNUM, mui_u8g2_u8_opt_line_wa_mud_pi), // Profile Selection Variable

  // Profile Config MUIFs
  // Temperature Related MUIFs
  MUIF_RO("PD", mui_draw_current_profiledisplay), // Draws updated profile values
  MUIF_RO("PS", mui_start_current_profiledisplay), // Starts profile
  //MUIF_BUTTON("RS", muif_profile_reset_values), // Button to Reset values
  MUIF_BUTTON("SV", muif_profilearray_edit_values), // Button to Save Values
  MUIF_U8G2_U8_MIN_MAX("t1", &edit_profile_array_element.time1, 0, PROFILE_MAX_TIME, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("t2", &edit_profile_array_element.time2, 0, PROFILE_MAX_TIME, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("t3", &edit_profile_array_element.time3, 0, PROFILE_MAX_TIME, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("t4", &edit_profile_array_element.time4, 0, PROFILE_MAX_TIME, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("t5", &edit_profile_array_element.time5, 0, PROFILE_MAX_TIME, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_U8G2_U8_MIN_MAX("T1", &edit_profile_array_element.temp1, 0, PROFILE_MAX_TEMP, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T2", &edit_profile_array_element.temp2, 0, PROFILE_MAX_TEMP, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T3", &edit_profile_array_element.temp3, 0, PROFILE_MAX_TEMP, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T4", &edit_profile_array_element.temp4, 0, PROFILE_MAX_TEMP, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T5", &edit_profile_array_element.temp5, 0, PROFILE_MAX_TEMP, mui_u8g2_u8_min_max_wm_mud_pi),

  // Reflow MUIFs
  MUIF_RO("TI", mui_draw_current_time), // Draws setpoint & time values
  MUIF_BUTTON("SO", muif_reflowprofile_init), // Button to Stop Reflow

};

// FDS Data
fds_t fds_data[] = 

// Main Menu
MUI_FORM(1)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_STYLE(1)
MUI_LABEL(20, 9, "Toasty Oven dev")
MUI_STYLE(0)
MUI_XYAT("RF", 40, 25, 2, " Reflow Menu ") 
MUI_XYAT("TT", 39, 42, 1, " Thermal Test ")

// Reflow Menu
MUI_FORM(2)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_STYLE(3)
MUI_XYAT("ST", 46, 15, 3, " Start Profile")
MUI_XYAT("PF", 92, 15, 0, " 1 | 2 ")
MUI_XYAT("CF", 52, 30, 4, " Config Profile ")
MUI_XYAT("B1", 22, 45, 1, " Back ") 
//MUI_XYAT("PI", 85, 45, 2, "PID Settings")

// Config Profile Menu
MUI_FORM(4)
MUI_AUX("TC") // Starts TC readings
MUI_AUX("PS") // Displays selected profile
MUI_STYLE(2)
MUI_XY("TD", 2, 63) // Postion of temp values
MUI_LABEL(5, 8, "PROFILE ")
MUI_XY("PD", 38, 8) // Postion of profile values
MUI_LABEL(60, 8, "t1: ")
MUI_LABEL(60, 18, "t2: ")
MUI_LABEL(60, 28, "t3: ")
MUI_LABEL(60, 38, "t4: ")
MUI_LABEL(60, 48, "t5: ")

MUI_LABEL(90, 8, "T1: ")
MUI_LABEL(90, 18, "T2: ")
MUI_LABEL(90, 28, "T3: ")
MUI_LABEL(90, 38, "T4: ")
MUI_LABEL(90, 48, "T5: ")

MUI_STYLE(3)
MUI_XYAT("SV", 22, 20, 2, " Done ") 
MUI_STYLE(2)
MUI_XY("t1", 72, 8)
MUI_XY("t2", 72, 18)
MUI_XY("t3", 72, 28)
MUI_XY("t4", 72, 38)
MUI_XY("t5", 72, 48)

MUI_XY("T1", 103, 8)
MUI_XY("T2", 103, 18)
MUI_XY("T3", 103, 28)
MUI_XY("T4", 103, 38)
MUI_XY("T5", 103, 48)
//MUI_STYLE(3)
//MUI_XYAT("RS", 25, 35, 2, " Reset ")

// Reflow Process Menu
MUI_FORM(3)
MUI_AUX("TC") // Starts TC readings
MUI_STYLE(2)
MUI_LABEL(2, 54, "Set: ")
MUI_LABEL(2, 63, "Cur: ")
MUI_XY("TD", 18, 63) // Postion of temp values
MUI_XY("TI", 18, 54) // Postion of time values

MUI_STYLE(0)
MUI_XYAT("SO", 107, 40, 2, " STOP ") 

;


void setup() {
  profile_array_init(); // set up all profiles
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

    detect_events();
    handle_events();
      
  } else {
      /* the menu should never become inactive, but if so, then restart the menu system */
      mui.gotoForm(/* form_id= */ 1, /* initial_cursor_position= */ 0);
  }
}
  
