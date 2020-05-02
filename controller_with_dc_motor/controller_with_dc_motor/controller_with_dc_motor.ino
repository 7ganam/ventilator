// PID control of DC motor experiment.
#include <stdio.h>

// PID controller variables----------------------------------------------------------------------------------
#include <PID_v1.h>
double  Kp =0.03750;
double  Ki = 0.00;
double  Kd = .001;
double  Setpoint =90;
double  Input, Output; //Define Variables we'll be connecting to
boolean set_pid = false;

double CONTROL_INPUT_SIGNAL ;
unsigned long CONTROL_INPUT_SIGNAL_START_TIME;
double CONTROL_INPUT_SIGNAL_UPPER_ANGLE =150; //the angle where the pedal just touches the bag from above..there should be no need to change this value anywhere else in the code (maybe I should declare it as const in future versions)
double CONTROL_INPUT_SIGNAL_LOWER_ANGLE =0; //the angle the pedal will descend to ... it's measured relative to SERVO_VIRTUAL_ZERO .. this angle controls the tidal volume.
float I2E =3;
float BREATH_PER_MIN= 35 ;  


        //just recived variable are used to detecte a change in the values sent by the GUI compared with the old values used in control
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE;
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_LOWER_ANGLE;
        float  JUST_RECIEVED_I2E = I2E ;
        float  JUST_RECIEVED_BREATH_PER_MIN = BREATH_PER_MIN ;  



float CYCLE_PERCENTAGE = 80;     //variable used in the function that generats the control signal .. ( I should make it passed to the function as aparamter later..no need for it to be a global variable) 
PID myPID(&Input, &Output, &CONTROL_INPUT_SIGNAL, Kp, Ki, Kd, DIRECT);

        int START_SIGNAL = 1;

// PID controller variables----------------------------------------------------------------------------------


//MOTOR DRIVER variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    #define clock_wise_ENA 10
    #define anti_clock_wise_ENA 11

    #define in1 6
    #define in2 7
    #define button 4
    int rotDirection = 0;
    int pressed = false;
//MOTOR DRIVER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


// ENCODER variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #define pinA 3
      #define pinB 2
      #define pinZ 5
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile long int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      long int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee



//#######################################################################################//

void setup()
{
  // Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).

  // motor driver setup
      pinMode(clock_wise_ENA, OUTPUT);
      pinMode(anti_clock_wise_ENA, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(button, INPUT);
      // Set initial rotation direction
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      

    Serial.begin(9600);
      //turn the PID on
      myPID.SetOutputLimits(-2, 2);
      myPID.SetMode(AUTOMATIC);
      myPID.SetMode(AUTOMATIC);
      myPID.SetSampleTime(5);

      initiate_control(); //start the timer used in generating the control signal
         ENCODER_CLICKS = float(CONTROL_INPUT_SIGNAL_UPPER_ANGLE) / 360 * 2000;


}

//#######################################################################################//
void loop()
{

    generate_control_signal_point( JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE , JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE , JUST_RECIEVED_I2E ,JUST_RECIEVED_BREATH_PER_MIN );

    ANGLE = float(ENCODER_CLICKS) / 2000 * 360;
    Input = ANGLE;
    myPID.Compute();
    give_command_to_motor_drive(Output);
    Serial.print(CONTROL_INPUT_SIGNAL);
    Serial.print(",");              //seperator
    Serial.println(ANGLE);


}

//#######################################################################################//



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


//encoder functions
      inline  void PinA_ISR()
              {  
                cli(); //stop interrupts happening before we read pin values
                int a= digitalRead(pinA);
                int b= digitalRead(pinB);
                if (a==1 && b==1 && aFlag)
                {
                  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
                  ENCODER_CLICKS ++; //decrement the encoder's position count
                  analogWrite(5 , ENCODER_CLICKS*2/360*255);
                  bFlag = 0; //reset flags for the next turn
                  aFlag = 0; //reset flags for the next turn
                }
                else if (b==1 && a==0) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
                sei(); //restart interrupts
              }
              
        inline void PinB_ISR()
              {
                
                  cli(); //stop interrupts happening before we read pin values
                  int a= digitalRead(pinA);
                  int b= digitalRead(pinB);
                  if (a && b && bFlag)
                  { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
                    ENCODER_CLICKS --; //increment the encoder's position count
                    analogWrite(5 , ENCODER_CLICKS*2/360*255);
                    bFlag = 0; //reset flags for the next turn
                    aFlag = 0; //reset flags for the next turn
                  }
                  else if (a==1 && b==0) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
                  sei(); //restart interrupts
              }


void initiate_control()
      {
        CONTROL_INPUT_SIGNAL_START_TIME=millis();
      }

void generate_control_signal_point( int max_angle , int min_angle ,float i2e ,float breath_per_min)
      {
      
            //detect if we recieved a new paramaters
            if(max_angle != CONTROL_INPUT_SIGNAL_UPPER_ANGLE || min_angle != CONTROL_INPUT_SIGNAL_LOWER_ANGLE ||  i2e !=  I2E ||  breath_per_min !=BREATH_PER_MIN)
            {
                CONTROL_INPUT_SIGNAL_START_TIME=millis();
                CONTROL_INPUT_SIGNAL_UPPER_ANGLE = max_angle ; 
                CONTROL_INPUT_SIGNAL_LOWER_ANGLE = min_angle;
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
            
          current_running_time=millis()- CONTROL_INPUT_SIGNAL_START_TIME;
          current_cycle_position= long(current_running_time) % long(Periodic_time);
          
           first_point= I_time * CYCLE_PERCENTAGE/100;
           second_point=I_time;
           third_point= I_time + E_time*CYCLE_PERCENTAGE/100;
         
          if (current_cycle_position <= first_point)
          {
              CONTROL_INPUT_SIGNAL = min_angle + ( current_cycle_position *  (max_angle-min_angle) / first_point);
          }
          else if (current_cycle_position > first_point && current_cycle_position <= second_point)
          {
              CONTROL_INPUT_SIGNAL = max_angle ;
          }
          else if (current_cycle_position > second_point && current_cycle_position <= third_point)
          {
              CONTROL_INPUT_SIGNAL = max_angle - ((current_cycle_position-second_point) * (max_angle-min_angle) / (third_point-second_point));
          }
          else if (current_cycle_position > third_point)
          {
              CONTROL_INPUT_SIGNAL=min_angle;
          }
      }
