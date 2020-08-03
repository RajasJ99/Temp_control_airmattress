
#include <SPI.h>
#include <max6675.h>

//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  



//I/O
int PWM_pin = 3;  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int clk = 8;      //Pin 1 from rotary encoder
int data = 9;     //Pin 2 from rotary encoder


// pins for first thermocouple
int so1Pin = 4;// SO1=Serial Out
int cs1Pin = 5;// CS1 = chip select CS pin
int sck1Pin = 6;// SCK1 = Serial Clock pin

// pins for 2nd thermocouple
int so2Pin = 7;// SO1=Serial Out
int cs2Pin = 2;// CS1 = chip select CS pin
int sck2Pin = 12;// SCK1 = Serial Clock 

//Variables
float set_temperature = 0;            
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;

//Vraiables for rotary encoder state detection
int clk_State;
int Last_State;  
bool dt_State;  

//PID constants
int kp = 90;   int ki = 80;   int kd = 50;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed =0;

int previousMillis =0;
const long interval = 1000;
int previousMillis2 =0;
const long interval2 = 5000;

MAX6675 thermocouple1(sck1Pin, cs1Pin, so1Pin);
MAX6675 thermocouple2(sck2Pin, cs2Pin, so2Pin);

void setup() {
  
  Serial.begin(9600);
  pinMode(PWM_pin,OUTPUT);
  TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM frequency of 928.5 Hz
  Time = millis();
  
  Last_State = digitalRead(clk);      //Detect first state of the encoder

  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT3);  //Set pin D11 trigger an interrupt on state change.   
                           
  pinMode(11,INPUT);
  pinMode(9,INPUT);
  pinMode(8,INPUT);
  digitalWrite(PWM_pin,HIGH);
  lcd.init();
  lcd.backlight();
}



void loop() {


Serial.print(thermocouple2.readCelsius());
Serial.print("  ");
Serial.println(thermocouple1.readCelsius());

if(menu_activated==0)
{
  temperature_read = (thermocouple2.readCelsius()+thermocouple1.readCelsius())/2;
  PID_error = 30 - temperature_read + 3;
  PID_p = 0.01*kp * PID_error;
  PID_i = 0.01*PID_i + (ki * PID_error);
  

  timePrev = Time;                           
  Time = millis();                            
  elapsedTime = (Time - timePrev) / 1000; 
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  PID_value = PID_p + PID_i + PID_d;


  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  
  if((255-PID_value)>(255/2))
  {
  digitalWrite(PWM_pin,HIGH);
  }
  
  else{
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(PWM_pin,LOW);
    previousMillis = currentMillis; 
    
    }
   unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval2)
    {
    digitalWrite(PWM_pin,HIGH);
        previousMillis2 = currentMillis2; 
      }
  }
  previous_error = PID_error;     

  delay(250);  
  lcd.setCursor(0,0);
  lcd.print("PID TEMP control");
  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.setCursor(2,1);
  lcd.print(set_temperature,1);
  lcd.setCursor(9,1);
  lcd.print("R:");
  lcd.setCursor(11,1);
  lcd.print(temperature_read,1);
}//end of menu 0 (PID control)




//First page of menu (temp setpoint)
if(menu_activated == 1)
{
   analogWrite(PWM_pin,255);
  if(set_temperature != last_set_temperature)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set  temperature");    
  lcd.setCursor(0,1);
  lcd.print(set_temperature);  
  }
  last_set_temperature = set_temperature;
  
 
}






//Second page of menu (P set)
if(menu_activated == 2)
{
  
  if(kp != last_kp)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   P  value  ");    
  lcd.setCursor(0,1);
  lcd.print(kp);  
  }
  last_kp = kp;
  
 
}

//Third page of menu (I set)
if(menu_activated == 3)
{
  
  if(ki != last_ki)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   I  value  ");    
  lcd.setCursor(0,1);
  lcd.print(ki);  
  }
  last_ki = ki;
  
 
}




//Forth page of menu (D set)
if(menu_activated == 4)
{
  
  if(kd != last_kd)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   D  value  ");    
  lcd.setCursor(0,1);
  lcd.print(kd);  
  }
  last_kd = kd;
}
  
}


ISR(PCINT0_vect){
if(menu_activated==1)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     if (dt_State != clk_State) { 
       set_temperature = set_temperature+0.5 ;
     }
     else {
       set_temperature = set_temperature-0.5;
     } 
  }
  Last_State = clk_State; 
} 

if(menu_activated==2)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     if (dt_State != clk_State) { 
       kp = kp+1 ;
     }
     else {
       kp = kp-1;
     } 
  }
  Last_State = clk_State; 
} 


if(menu_activated==3)
   {
  clk_State =   (PINB & B00000001);
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     if (dt_State != clk_State) { 
       ki = ki+1 ;
     }
     else {
       ki = ki-1;
     } 
  }
  Last_State = clk_State;
}

 if(menu_activated==4)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     if (dt_State != clk_State) { 
       kd = kd+1 ;
     }
     else {
       kd = kd-1;
     } 
  }
  Last_State = clk_State; 
}
   



  if (PINB & B00001000) 
  {       
    button_pressed = 1;
  } 
  else if(button_pressed == 1)
  {
   
   if(menu_activated==4)
   {
    menu_activated = 0;  
    PID_values_fixed=1;
    button_pressed=0; 
    delay(1000);
   }

   if(menu_activated==3)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    kd = kd + 1; 
    delay(1000);
   }

   if(menu_activated==2)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    ki = ki + 1; 
    delay(1000);
   }

   if(menu_activated==1)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    kp = kp + 1; 
    delay(1000);
   }


   if(menu_activated==0 && PID_values_fixed != 1)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0;
    set_temperature = set_temperature+1;   
    delay(1000);
   }
   PID_values_fixed = 0;
   
  }  
}
