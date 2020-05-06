
//MOTOR DRIVER variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    #define clock_wise_ENA 10
    #define anti_clock_wise_ENA 9

//MOTOR DRIVER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


// ENCODER variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #include "PinChangeInterrupt.h"

      #define pinA 19
      #define pinB 18
      #define pinZ 2
////uno
//      #define pinA 3
//      #define pinB 2
//      #define pinZ 5

      volatile bool ZERO_FLAG=0;
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile long int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      long int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

   unsigned long timeer=millis();


//#######################################################################################//

void setup()
{
    
  // Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      pinMode(pinZ, INPUT_PULLUP); 

           
      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).
    //attachPCINT(digitalPinToPCINT(pinZ), PinZ_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinZ), PinZ_ISR,  CHANGE); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).
      delay(100); // this must be here ..  if not the PinZ_ISR is called somehow .. it seems that arduino might read noise at startup of the code .. so we need some kind of delay for the signals to settle.
      ZERO_FLAG=0;
      
   Serial.begin(9600);

   int found_zero =  discover_zero() ; //this is a blocking function .. it doesn't return untill it finds the encoders zero ... you might need to modify it to make it move a motor till it finds the zero of just move the encoder by hand to find it.
//   move_to_angle(50);
//   set_zero();
   
    Serial.println(found_zero);
}

//#######################################################################################//
void loop()
{




    
}

//#######################################################################################//



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
              
                 analogWrite(clock_wise_ENA ,80);  //motor driver specific code .. change this if you have a different interface
                 while(ZERO_FLAG==0){    };//wait till the zchannel interrupt changes ZERO_FLAG to 1;
                 analogWrite(clock_wise_ENA , 0);  //motor driver specific code .. change this if you have a different interface
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
