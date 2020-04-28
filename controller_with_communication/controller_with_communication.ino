//Communication Variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      String Received_Data_String="";
      String String_To_Send;
      String Received_Variables[5];
      int Data_To_Send[5];
      int Received_Data[5];
      char Dummy_Character;
      int Sending_Data_Rate=10;
      long int Current_Time=0;
      long int Previous_Time=0;
// communication variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


//Pressure Sensor Variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #include<Adafruit_Sensor.h>
      #include<Adafruit_BMP085_U.h>
      Adafruit_BMP085_Unified Pressure_Sensor=Adafruit_BMP085_Unified(555);
      sensors_event_t Pressure_Sensor_Event;
      float Pressure=0;
      float IINIT_PRESSURE = 0;
// Pressure Sensor Variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

// ENCODER variables SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      #define pinA 19
      #define pinB 18
      #define pinZ 2
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

//SERVO varialbles SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
        #include <Servo.h>
        Servo myservo;  // create servo object to control a servo
        int SERVO_VIRTUAL_ZERO = 50;  // the sevro angle where the pedal presses the bag to its maximum value..set this value manually 
// SERVO varialbles eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

//PID variables and Control SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS       
        #include <PID_v1.h>
        #define PIN_PID_SENSOR_INPUT 0
        #define PIN_PID_OUTPUT 3
        double  PID_SENSOR_INPUT, PID_OUTPUT;
        double CONTROL_INPUT_SIGNAL ;
        unsigned long CONTROL_INPUT_SIGNAL_START_TIME;

        double User_tuned_kp= .35 , User_tuned_ki=2 , User_tuned_kd=0;
        double Kp=.35, Ki=32, Kd=0;
        
        
        double CONTROL_INPUT_SIGNAL_UPPER_ANGLE =120; //the angle where the pedal just touches the bag from above..there should be no need to change this value anywhere else in the code (maybe I should declare it as const in future versions)
        double CONTROL_INPUT_SIGNAL_LOWER_ANGLE =10; //the angle the pedal will descend to ... it's measured relative to SERVO_VIRTUAL_ZERO .. this angle controls the tidal volume.
        float I2E = 1 ;
        float BREATH_PER_MIN= 30 ;  

        //just recived variable are used to detecte a change in the values sent by the GUI compared with the old values used in control
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE;
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_LOWER_ANGLE;
        float  JUST_RECIEVED_I2E = I2E ;
        float  JUST_RECIEVED_BREATH_PER_MIN = BREATH_PER_MIN ;  

        
        float CYCLE_PERCENTAGE = 80;     //variable used in the function that generats the control signal .. ( I should make it passed to the function as aparamter later..no need for it to be a global variable) 
        PID myPID(&PID_SENSOR_INPUT, &PID_OUTPUT, &CONTROL_INPUT_SIGNAL, Kp, Ki, Kd, DIRECT);  //construct an instance of the PID class named myPID
    
        int START_SIGNAL = 0;  // the variable that controls if the device is working or not .. (should be bool)
//PID variables and Control eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
















 
//Main Setup Function--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  
      Setup_Communication();

      Setup_Pressure_Sensor();
 
      Serial.begin(9600);
      
    // Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).

    //Serial setup
      Serial.begin(9600);
      
    //servo setup  
      myservo.attach(9);  // attaches the servo on pin 9 to the servo object

    //PID setup
      myPID.SetMode(AUTOMATIC);
      myPID.SetOutputLimits(-30,360);
      myPID.SetSampleTime(5);

      initiate_control(); //start the timer used in generating the control signal
      align_encoder_with_servo(CONTROL_INPUT_SIGNAL_UPPER_ANGLE);  //move to the angle CONTROL_INPUT_SIGNAL_UPPER_ANGLE+SERVO_VIRTUAL_ZERO and set the encoder to CONTROL_INPUT_SIGNAL_UPPER_ANGLE*2000/360 .. after setting always use  servo_write_gh() not servowrit()
      
      Measure_Pressure();
      IINIT_PRESSURE =Pressure;


}



void loop()
{

       if(START_SIGNAL == 0)
       {
                      servo_write_gh( CONTROL_INPUT_SIGNAL_UPPER_ANGLE    ,  SERVO_VIRTUAL_ZERO);
                      Receive_Data_gh();
                      Measure_Pressure();
                      ANGLE = float(ENCODER_CLICKS) / 2000 * 360;

// debugging prints.. un comment those and comment Send_Data_If_Needed line to use serial plotter with highes resolution;                      
//                     Serial.print(CONTROL_INPUT_SIGNAL);
//                     Serial.print(",");              //seperator
//                     Serial.print(ANGLE);
//                     Serial.print(",");              //seperator
//                     Serial.println(Pressure-IINIT_PRESSURE);
                     
                      Send_Data_If_Needed();
                      Ki=0;  // close the integrator when the controller isn't working
       }
        else
        {
          

                     Ki=User_tuned_ki;  // open the integrator 
                     Receive_Data_gh();
                     generate_control_signal_point( JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE , JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE , JUST_RECIEVED_I2E ,JUST_RECIEVED_BREATH_PER_MIN );
              
                     ANGLE = float(ENCODER_CLICKS) / 2000 * 360;
                     PID_SENSOR_INPUT = ANGLE;
                     myPID.Compute();
                     
                   servo_write_gh(CONTROL_INPUT_SIGNAL   ,  SERVO_VIRTUAL_ZERO);
//                     servo_write_gh(PID_OUTPUT    ,  SERVO_VIRTUAL_ZERO);
              
                   
                     Measure_Pressure();
                     
// debugging prints.. un comment those and comment Send_Data_If_Needed line to use serial plotter with highes resolution;                                           
//                     Serial.print(CONTROL_INPUT_SIGNAL);
//                     Serial.print(",");              //seperator
//                     Serial.print(ANGLE);
//                     Serial.print(",");              //seperator
//                     Serial.println(Pressure-IINIT_PRESSURE);
//              
                    
                     Send_Data_If_Needed();

        }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------















//control functions
void align_encoder_with_servo( int resetting_angle)
      {
           servo_write_gh(resetting_angle+5    ,  SERVO_VIRTUAL_ZERO);
           delay(500);
           servo_write_gh(resetting_angle    ,  SERVO_VIRTUAL_ZERO);
           delay(2000);
           ENCODER_CLICKS = float(resetting_angle)*2000/360;
      }



void servo_write_gh(int goal_angle , int servo_virtual_zero)
      {
           myservo.write(goal_angle + servo_virtual_zero);
      }



void initiate_control()
      {
        CONTROL_INPUT_SIGNAL_START_TIME=millis();
      }

void generate_control_signal_point( int max_angle , int min_angle ,float i2e ,float breath_per_min)
      {
      
      
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


//Pressure Sensor Functions
        void Setup_Pressure_Sensor()
                {
                   Pressure_Sensor.begin();
                }
        
        void Measure_Pressure()
                {
                    Pressure_Sensor.getEvent(&Pressure_Sensor_Event);
                    Pressure=(Pressure_Sensor_Event.pressure-993.0)*1.019744;
                }


//Communication Functions
        void Setup_Communication()
            {
              Serial.begin(9600);
              Flush_Serial();  
              Data_To_Send[0]=0;
              Data_To_Send[1]=0;
              Data_To_Send[2]=0;
              Data_To_Send[3]=0;
              Data_To_Send[4]=0;
            }
            
        void Calculate_Inputs()
            {
              JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE - (  float(CONTROL_INPUT_SIGNAL_UPPER_ANGLE*Received_Data[2]) /100  );
              JUST_RECIEVED_BREATH_PER_MIN =  Received_Data[3];
              JUST_RECIEVED_I2E =  Received_Data[4];
            }
            
        void Calculate_Outputs()
            {
              Data_To_Send[0]=0;
              Data_To_Send[1]=int(Pressure-IINIT_PRESSURE);
              Data_To_Send[2]=ANGLE;
              Data_To_Send[3]=0;
              Data_To_Send[4]=0;
            }
            
        void Flush_Serial()
            {
              while(Serial.available()>0)
              {
                Dummy_Character=Serial.read();
              }
            }
            
        void Receive_Data_gh()
            {
              while(Serial.available()>0)
              {
                float tag1=Serial.parseFloat();
                if(tag1==111)
                {
                  Received_Data[0]=Serial.parseFloat();
                  Received_Data[1]=Serial.parseFloat();
                  Received_Data[2]=Serial.parseFloat();
                  Received_Data[3]=Serial.parseFloat();
                  Received_Data[4]=Serial.parseFloat();
                  
                  START_SIGNAL=Received_Data[0];
                  Calculate_Inputs();
                }
              }
              
            }
            
            
            
        void Send_Data()
            {
              String_To_Send=String(Data_To_Send[0]);
              String_To_Send.concat(",");
              String_To_Send.concat(String(Data_To_Send[1]));
              String_To_Send.concat(",");
              String_To_Send.concat(String(Data_To_Send[2]));
              String_To_Send.concat(",");
              String_To_Send.concat(String(Data_To_Send[3]));
              String_To_Send.concat(",");
              String_To_Send.concat(String(Data_To_Send[4]));
              String_To_Send.concat('\n');
              Serial.print(String_To_Send); 
            }
            
            
        void Send_Data_If_Needed()
            {
              Current_Time=millis();
              if(Current_Time-Previous_Time>=Sending_Data_Rate)
              {
                Previous_Time=Current_Time;
                Calculate_Outputs();
                Send_Data();
              }  
            }
