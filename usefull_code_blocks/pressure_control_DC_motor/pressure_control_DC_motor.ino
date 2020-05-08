#include <stdio.h>

// PID controller variables----------------------------------------------------------------------------------
#include <PID_v1.h>
double  Kp =0.03;
double  Ki = 0.00;
double  Kd = 0.0006;
double  Setpoint =90;
double  Input, Output; //Define Variables we'll be connecting to
boolean set_pid = false;

double CONTROL_INPUT_SIGNAL ;
unsigned long CONTROL_INPUT_SIGNAL_START_TIME;
double CONTROL_INPUT_SIGNAL_UPPER_ANGLE =140; //the angle where the pedal just touches the bag from above..there should be no need to change this value anywhere else in the code (maybe I should declare it as const in future versions)
double CONTROL_INPUT_SIGNAL_LOWER_ANGLE =0; //the angle the pedal will descend to ... it's measured relative to SERVO_VIRTUAL_ZERO .. this angle controls the tidal volume.


//just recived variable are used to detecte a change in the values sent by the GUI compared with the old values used in control
double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE;
double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_LOWER_ANGLE;



float CYCLE_PERCENTAGE = 80;     //variable used in the function that generats the control signal .. ( I should make it passed to the function as aparamter later..no need for it to be a global variable) 
PID myPID(&Input, &Output, &CONTROL_INPUT_SIGNAL, Kp, Ki, Kd, DIRECT);

        int START_SIGNAL = 1;

// PID controller variables----------------------------------------------------------------------------------


//NORMALIZED_SIGNAL_GENERATOR variabes SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS       
        double NORMALIZED_CONTROL_INPUT_SIGNAL ;
        unsigned long NORMALIZED_SIGNAL_START_TIME;
        float  I2E = 4 ;
        float  BREATH_PER_MIN= 30 ;  
        float  JUST_RECIEVED_I2E = I2E ;
        float  JUST_RECIEVED_BREATH_PER_MIN = BREATH_PER_MIN ;  
        int    END_OF_CYCLE_SIGNAL=0;
//NORMALIZED_SIGNAL_GENERATOR variabes eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee





//MAX_PRESSURE_DETECTOR SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    double LAST_CYCLE_MAX_PRESSURE;
    double CURRENT_CYCLE_MAX_PRESSURE;
    int old_EOC_signal;
    int new_EOC_signal;
//MAX_PRESSURE_DETECTOR eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

//P_PID variabls SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
  double P_ERROR=0;
  double P_SETPOINT;
  double P_ERROR_INTEGRAL=0;
  double P_KP;
  double P_KI;
  double MAX_ANGLE;

  int P_old_EOC_signal;
  int P_new_EOC_signal;
//P_PID variabls eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee




//Pressure Sensor Variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #include<Adafruit_Sensor.h>
      #include<Adafruit_BMP085_U.h>
      Adafruit_BMP085_Unified Pressure_Sensor=Adafruit_BMP085_Unified(555);
      sensors_event_t Pressure_Sensor_Event;
      float Pressure=0;
      float IINIT_PRESSURE = 0;
// Pressure Sensor Variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


//REAL_SIGNAL_GENERATOR SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
 double SCALED_UP_SIGNAL;
 //REAL_SIGNAL_GENERATOR eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


//MOTOR DRIVER variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    #define clock_wise_ENA 10
//    #define anti_clock_wise_ENA 11
   #define anti_clock_wise_ENA 9
//MOTOR DRIVER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee



// ENCODER variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
     #include "PinChangeInterrupt.h"     
//      #define pinA 3
//      #define pinB 2
//      #define pinZ 5

      #define pinA 19
      #define pinB 18
      #define pinZ 2

      volatile bool ZERO_FLAG=0; // this will be one when the zero is met
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile long int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      long int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee





 
//Main Setup Function--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{

  
      Setup_Pressure_Sensor();
      Serial.begin(9600);

  // motor driver setup
      pinMode(clock_wise_ENA, OUTPUT);
      pinMode(anti_clock_wise_ENA, OUTPUT);

    //Serial setup
      Serial.begin(9600);
      
    //turn the PID on
      myPID.SetOutputLimits(-1, 1);
      myPID.SetMode(AUTOMATIC);
      myPID.SetSampleTime(50);
            
  // Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      pinMode(pinZ, INPUT_PULLUP); 


      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).
//      attachPCINT(digitalPinToPCINT(pinZ), PinZ_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinZ), PinZ_ISR,  CHANGE); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).

      delay(100); // this must be here ..  if not the PinZ_ISR is called somehow .. it seems that arduino might read noise at startup of the code .. so we need some kind of delay for the signals to settle.
      ZERO_FLAG=0;  
      
   Serial.begin(9600);

   int found_zero =  discover_zero() ; //this is a blocking function .. it doesn't return untill it finds the encoders zero ... you might need to modify it to make it move a motor till it finds the zero of just move the encoder by hand to find it.
   move_to_angle(35);
   set_zero();
      
      Measure_Pressure();
      IINIT_PRESSURE =Pressure;
      initiate_normalized_signal();
      randomSeed(analogRead(0));

}



void loop()
{
    P_SETPOINT = 20;
    P_KP = 0.5;
    P_KI = 0.75;

    normalized_signal_generator( 2,20);
    Measure_Pressure();
    max_pressure_tracker();

    P_PID_compute();
 
    ANGLE = float(ENCODER_CLICKS) / 2000 * 360;
    Input = ANGLE;
    myPID.Compute();
    give_command_to_motor_drive(Output);
    generate_real_signal();

   CONTROL_INPUT_SIGNAL = SCALED_UP_SIGNAL;

//
//     P_ERROR ;
//     MAX_ANGLE ;
    Serial.print(Pressure);
    Serial.print(",");              //seperator
    Serial.print(SCALED_UP_SIGNAL);
    Serial.print(",");              //seperator
    Serial.print(ANGLE);
     Serial.print(",");       
    Serial.println(P_SETPOINT);
//    Serial.print(",");    
//    Serial.println(Pressure);
    
    
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
void P_PID_compute()
{
    double OUTPUT_ANGLE_LIMIT = 140;   
    P_new_EOC_signal = END_OF_CYCLE_SIGNAL ;
    if (P_new_EOC_signal==1 &&  P_old_EOC_signal ==0)
    {
         P_ERROR = P_SETPOINT  -  LAST_CYCLE_MAX_PRESSURE  ;
         P_ERROR_INTEGRAL = P_ERROR_INTEGRAL + P_ERROR;
         double integral_component =  P_KI * P_ERROR_INTEGRAL ;
         
             if(integral_component > OUTPUT_ANGLE_LIMIT )
             {
                    integral_component =  OUTPUT_ANGLE_LIMIT ; 
             }
             
         MAX_ANGLE = P_KP * P_ERROR +  integral_component ;

              if(MAX_ANGLE > OUTPUT_ANGLE_LIMIT )
             {
                    MAX_ANGLE =OUTPUT_ANGLE_LIMIT   ; 
             }
    }

    P_old_EOC_signal = P_new_EOC_signal;
}



void give_command_to_motor_drive(double pid_output)   
{

  double dead_band = 0.1;
      if(pid_output > 0)
      {
         pid_output=pid_output + dead_band;
      }
      else if(pid_output < 0)
      {
         pid_output=pid_output - dead_band;
      }

       if (pid_output > 1)
        {
          pid_output = 1;
        }
        if (pid_output < -1)
        {
          pid_output = -1;
        }
  
  int signed_pwm_output=pid_output/1*255;

  if (signed_pwm_output < 0 )
  {
      analogWrite(anti_clock_wise_ENA ,0);
      analogWrite(clock_wise_ENA , abs(signed_pwm_output));
  }

  if (signed_pwm_output > 0 )
  {
    analogWrite(anti_clock_wise_ENA , abs(signed_pwm_output));
    analogWrite(clock_wise_ENA , 0);
  }

}


void generate_real_signal()
{
   SCALED_UP_SIGNAL = NORMALIZED_CONTROL_INPUT_SIGNAL * MAX_ANGLE ; 
}


void max_pressure_tracker()
{
  new_EOC_signal = END_OF_CYCLE_SIGNAL ;

    if (new_EOC_signal==0 &&  old_EOC_signal ==0)
    {
           Measure_Pressure();
            if (Pressure > CURRENT_CYCLE_MAX_PRESSURE)
            {
                    CURRENT_CYCLE_MAX_PRESSURE = Pressure;
            }
      
    }
     else if (new_EOC_signal==1 &&  old_EOC_signal ==0)
    {
            LAST_CYCLE_MAX_PRESSURE =  CURRENT_CYCLE_MAX_PRESSURE ; 
    }

     else if (new_EOC_signal==0 &&  old_EOC_signal ==1)
    {
            CURRENT_CYCLE_MAX_PRESSURE =0;
    }

    old_EOC_signal = new_EOC_signal;
}



void normalized_signal_generator( float i2e ,float breath_per_min)
      {
      
            if( i2e !=  I2E ||  breath_per_min !=BREATH_PER_MIN)
            {
                NORMALIZED_SIGNAL_START_TIME=millis();
                I2E = i2e;
                BREATH_PER_MIN =breath_per_min ;
            }
      

            float Periodic_time;
            float I_time;
            float E_time;
      
            float first_point;
            float second_point;
            float third_point;
      
            unsigned long current_running_time;
            unsigned long current_cycle_position;
            
            Periodic_time = 1/breath_per_min*60*1000;
            I_time=Periodic_time/(1+I2E);
            E_time=Periodic_time-I_time;
            
            I_time=Periodic_time-I_time;
            E_time=Periodic_time/(1+I2E);

//            float temp = I_time ;  
//            I_time=E_time;
//            E_time=temp;
            
          current_running_time=millis()- NORMALIZED_SIGNAL_START_TIME;
          current_cycle_position= long(current_running_time) % long(Periodic_time);
          
           first_point= I_time * CYCLE_PERCENTAGE/100;
           second_point=I_time;
           third_point= I_time + E_time*CYCLE_PERCENTAGE/100;

//         long randNumber = random(50);
          long randNumber =1;

          if (current_cycle_position <= first_point)
          {
              NORMALIZED_CONTROL_INPUT_SIGNAL = 0 + ( current_cycle_position *  (1-0) / first_point)* randNumber;
              END_OF_CYCLE_SIGNAL =0;
          }
          else if (current_cycle_position > first_point && current_cycle_position <= second_point)
          {
              NORMALIZED_CONTROL_INPUT_SIGNAL = 1 * randNumber ;
          }
          else if (current_cycle_position > second_point && current_cycle_position <= third_point)
          {
              NORMALIZED_CONTROL_INPUT_SIGNAL = (1 - ((current_cycle_position-second_point) * (1-0) / (third_point-second_point))   )* randNumber;
          }
          else if (current_cycle_position > third_point)
          {
              
               END_OF_CYCLE_SIGNAL =1;
               
              NORMALIZED_CONTROL_INPUT_SIGNAL = 0;
          }
      }

void initiate_normalized_signal()
      {
         NORMALIZED_SIGNAL_START_TIME=millis();
      }


void measure_fake_pressure()
      {
         FAKE_PRESSURE=NORMALIZED_CONTROL_INPUT_SIGNAL*50;
      }



//encoder functions
      inline  void PinA_ISR()
              {  
                cli(); //stop interrupts happening before we read pin values
                int a= digitalRead(pinA);
                int b= digitalRead(pinB);
                if (a==1 && b==1 && aFlag)
                {
                  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
                  ENCODER_CLICKS --; //decrement the encoder's position count
                  bFlag = 0; //reset flags for the next turn
                  aFlag = 0; //reset flags for the next turn
                }
                else if (b==1 && a==0) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
                sei(); //restart interrupts
                
                ANGLE = float(ENCODER_CLICKS) / 2000 * 360;

              }
              
        inline void PinB_ISR()
              {
                
                  cli(); //stop interrupts happening before we read pin values
                  int a= digitalRead(pinA);
                  int b= digitalRead(pinB);
                  if (a && b && bFlag)
                  { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
                    ENCODER_CLICKS ++; //increment the encoder's position count
                    bFlag = 0; //reset flags for the next turn
                    aFlag = 0; //reset flags for the next turn
                  }
                  else if (a==1 && b==0) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
                  sei(); //restart interrupts
                  
                  ANGLE = float(ENCODER_CLICKS) / 2000 * 360;
                
              }

              
        inline void PinZ_ISR()
              {
                  ZERO_FLAG=1;
              }
              

        int discover_zero()   
              {
              
                 analogWrite(anti_clock_wise_ENA ,80);  //motor driver specific code .. change this if you have a different interface
                 while(ZERO_FLAG==0){     };//wait till the zchannel interrupt changes ZERO_FLAG to 1;
                 analogWrite(anti_clock_wise_ENA , 0);  //motor driver specific code .. change this if you have a different interface
                 ENCODER_CLICKS=0;
                 ZERO_FLAG=0;
                 return (1);
              
              }


         void move_to_angle(double goal_angle)
         {
              while(ANGLE < goal_angle)
              {  
                Serial.println(ANGLE);
                 analogWrite(anti_clock_wise_ENA ,80);  //motor driver specific code .. change this if you have a different interface  
              }
              
              analogWrite(anti_clock_wise_ENA ,0);  //motor driver specific code .. change this if you have a different interface  

         }

        void set_zero ()
        {
              ENCODER_CLICKS=0;
        }


//
//Pressure Sensor Functions
        void Setup_Pressure_Sensor()
                {
                   Pressure_Sensor.begin();
                }
        
        void Measure_Pressure()
                {
                    Pressure_Sensor.getEvent(&Pressure_Sensor_Event);
                    Pressure=(Pressure_Sensor_Event.pressure-993.0)*1.019744 - IINIT_PRESSURE ;
                }
