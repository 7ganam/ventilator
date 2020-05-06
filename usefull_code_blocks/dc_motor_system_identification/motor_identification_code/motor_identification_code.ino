
//communication variables ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss

int RECIEVE_TAG=222;
float RECIVE_VAR1=0;
float RECIVE_VAR2=0;
float RECIVE_VAR3=0;

int SEND_TAG=111;
float SEND_VAR1=0;
float SEND_VAR2=0;
float SEND_VAR3=0;
//communication variabse eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


// ENCODER variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #define pinA 3
      #define pinB 2
      #define pinZ 5
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile long int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      long int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


//MOTOR DRIVER variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    #define enA 10
    #define in1 6
    #define in2 7
    #define button 4
    int rotDirection = 0;
    int pressed = false;

//MOTOR DRIVER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

//GENERAL VARIABLES SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
unsigned long star_time = millis();
unsigned long sample_time_in_ms = 5;
unsigned long current_period_start_time = millis();




//GENERAL VARIABLES SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS



//Main Setup Function--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Main Setup Function--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Main Setup Function--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
      // Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).


    //Serial setup
      Serial.begin(2000000);


    // motor driver setup
      pinMode(enA, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(button, INPUT);
      // Set initial rotation direction
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  // put your main code here, to run repeatedly:
                        ANGLE = float(ENCODER_CLICKS) / 2000 * 360;
//                      Serial.println(ANGLE);
                       
                          float analog_reading = analogRead(A0);
                          
                          float function_generator_signal = analog_reading/1024*5;
                          float pwm_float = analog_reading/1024*255;
                          float motor_voltag_input = analog_reading/1024*12;
                          int pwm_out= int(pwm_float);
                          analogWrite(enA, pwm_out); // Send PWM signal to L298N Enable pin
                         unsigned long time_stamp = millis()-star_time;


                          SEND_VAR1=ANGLE;
                          SEND_VAR2=pwm_out;
                          SEND_VAR3=time_stamp;
             if((millis()-current_period_start_time)>sample_time_in_ms) 
             {
              
                      send_send_variables();
                      current_period_start_time=millis();
              }            

                  
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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
                    ENCODER_CLICKS ++; //increment the encoder's position count
                    analogWrite(5 , ENCODER_CLICKS*2/360*255);
                    bFlag = 0; //reset flags for the next turn
                    aFlag = 0; //reset flags for the next turn
                  }
                  else if (a==1 && b==0) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
                  sei(); //restart interrupts
              }

// communication functions 
          void update_recieve_variables()
          {
           if ( Serial.available() )
           {
              int tag;
              tag =int(Serial.parseFloat()); 
              if (tag==RECIEVE_TAG)
              {
                      RECIVE_VAR1=Serial.parseFloat();
                      RECIVE_VAR2= Serial.parseFloat();
                      RECIVE_VAR3= Serial.parseFloat();
              }
          
          }
          }
          void send_send_variables()
          {
                Serial.println(SEND_TAG); 
                Serial.println(SEND_VAR1);
                Serial.println(SEND_VAR2);
                Serial.println(SEND_VAR3);

}

              
