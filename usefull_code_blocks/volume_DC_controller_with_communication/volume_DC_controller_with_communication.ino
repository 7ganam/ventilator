//Communication Variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
      String   Received_Data_String="";
      String   String_To_Send;
      String   Received_Variables[5];
      int      Data_To_Send[5];
      int      Received_Data[5];
      char     Dummy_Character;
      int      Sending_Data_Rate=10;
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
////uno
//      #define pinA 3
//      #define pinB 2
//      #define pinZ 5

      volatile bool  ZERO_FLAG=0; // used in the function that detects the zero of the encoder
      volatile int   aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
      volatile int   bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
      volatile int   ENCODER_CLICKS = 0;//this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
      int ANGLE;
// ENCODER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

//PID variables and Control SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS       
        #include <PID_v1.h>
        #define PIN_PID_SENSOR_INPUT 0
        #define PIN_PID_OUTPUT 3
        double  PID_SENSOR_INPUT, PID_OUTPUT;
        double  CONTROL_INPUT_SIGNAL ;

        double User_tuned_kp= 0.06 ,     User_tuned_ki=0 ,      User_tuned_kd=0.0012;
        
        double Kp = User_tuned_kp,     Ki = User_tuned_ki,      Kd = User_tuned_kd;

        unsigned long CONTROL_SIGNAL_START_TIME;

        double max_angle =140;
        double CONTROL_INPUT_SIGNAL_UPPER_ANGLE = 140; //the angle where the pedal just touches the bag from above..there should be no need to change this value anywhere else in the code (maybe I should declare it as const in future versions)
        double CONTROL_INPUT_SIGNAL_LOWER_ANGLE = 0; //the angle the pedal will descend to ... it's measured relative to SERVO_VIRTUAL_ZERO .. this angle controls the tidal volume.
        float  I2E = 1 ;
        float  BREATH_PER_MIN= 30 ;  

        //just recived variable are used to detecte a change in the values sent by the GUI compared with the old values used in control
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE;
        double JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_LOWER_ANGLE;
        float  JUST_RECIEVED_I2E = I2E ;
        float  JUST_RECIEVED_BREATH_PER_MIN = BREATH_PER_MIN ;  
        
        float  CYCLE_PERCENTAGE = 80;     //variable used in the function that generats the control signal .. ( I should make it passed to the function as aparamter later..no need for it to be a global variable) 
        PID    myPID(&PID_SENSOR_INPUT, &PID_OUTPUT, &CONTROL_INPUT_SIGNAL, Kp, Ki, Kd, DIRECT);  //construct an instance of the PID class named myPID
    
        int START_SIGNAL = 0;  // the variable that controls if the device is working or not .. (should be bool)
//PID variables and Control eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee


//MOTOR DRIVER variables  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS

////uno
//  #define clock_wise_ENA 10
//  #define anti_clock_wise_ENA 11
    #define clock_wise_ENA 10
    #define anti_clock_wise_ENA 9

//MOTOR DRIVER variables eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

 
void setup()
{
  
      Setup_Communication();
      Serial.begin(9600);
      
// Setup encoder 
      pinMode(pinA, INPUT_PULLUP); // set pinA as an PID_SENSOR_INPUT, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
      pinMode(pinB, INPUT_PULLUP); // set pinB as an PID_SENSOR_INPUT, pulled HIGH to thxxe logic voltage (5V or 3.3V for most cases)
      pinMode(pinZ, INPUT_PULLUP); 

      attachInterrupt(digitalPinToInterrupt(pinB), PinA_ISR,  RISING); // set an interrupt on PinA_ISR, looking for a rising edge signal and executing the "PinA_ISR" Interrupt Service Routine (below)
      attachInterrupt(digitalPinToInterrupt(pinA), PinB_ISR,  RISING); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).
      ////uno
      //attachPCINT(digitalPinToPCINT(pinZ), PinZ_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinZ), PinZ_ISR,  CHANGE); // set an interrupt on PinB_ISR, looking for a rising edge signal and executing the "PinB_ISR" Interrupt Service Routine (below).
      
      delay(100); // this must be here ..  if not the PinZ_ISR is called somehow .. it seems that arduino might read noise at startup of the code .. so we need some kind of delay for the signals to settle.
      ZERO_FLAG=0; 
      
// motor driver setup
      pinMode(clock_wise_ENA, PID_OUTPUT);
      pinMode(anti_clock_wise_ENA, PID_OUTPUT);



//PID setup
      myPID.SetOutputLimits(-1, 1);
      myPID.SetMode(AUTOMATIC);
      myPID.SetSampleTime(50);

      initiate_control(); //start the timer used in generating the control signal // i think there is no need to call this any more after the last update..should try deleting it when i'm on the hardware soon.
     

//setup pressure sensor
      Setup_Pressure_Sensor();
      Measure_Pressure();
      IINIT_PRESSURE =Pressure;


//reset zero
     move_to_angle(20);
     int found_zero =  discover_zero() ; //this is a blocking function .. it doesn't return untill it finds the encoders zero ... you might need to modify it to make it move a motor till it finds the zero of just move the encoder by hand to find it.
     //Serial.println(found_zero);
     move_to_angle(53);
     set_zero();

}




//### MAIN LOOP #####################################################################################################################
//### MAIN LOOP #####################################################################################################################
//### MAIN LOOP #####################################################################################################################
void loop()
{

       Receive_Data_gh();
       if(START_SIGNAL == 0)
       {
                      PID_OUTPUT=0;
                      CONTROL_INPUT_SIGNAL=0;
                      give_command_to_motor_drive(0);
       }
        else if(START_SIGNAL ==1)
        {
                     generate_control_signal_point( JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE , 0 , JUST_RECIEVED_I2E ,JUST_RECIEVED_BREATH_PER_MIN );
                     PID_SENSOR_INPUT = ANGLE;
                     myPID.Compute();
                     give_command_to_motor_drive(PID_OUTPUT);
        }
        
                    Measure_Pressure();

                    //// debugging prints.. un comment those and comment Send_Data_If_Needed line to use serial plotter with highes resolution;                                           
                           Serial.print(ANGLE);
                           Serial.print(",");              //seperator
                           Serial.print(CONTROL_INPUT_SIGNAL);
                           Serial.print(",");              //seperator
                           Serial.println(PID_OUTPUT*100);
             
//                    Send_Data_If_Needed();
}


//### MAIN LOOP end #####################################################################################################################
//### MAIN LOOP end #####################################################################################################################
//### MAIN LOOP end #####################################################################################################################




//control functions ******************************************************************************************************************

void give_command_to_motor_drive(double pid_PID_OUTPUT)   
      {
            double dead_band = 0.1;
//            if( pid_PID_OUTPUT > 0 )      {pid_PID_OUTPUT=pid_PID_OUTPUT + dead_band;  }
//            else if(pid_PID_OUTPUT < 0) {pid_PID_OUTPUT=pid_PID_OUTPUT - dead_band;  }
      
            if (pid_PID_OUTPUT > 1) { pid_PID_OUTPUT = 1; }
            if (pid_PID_OUTPUT < -1){ pid_PID_OUTPUT = -1;}
            
            int signed_pwm_PID_OUTPUT=pid_PID_OUTPUT/1*255;
            if (signed_pwm_PID_OUTPUT > 0 )
            {
                analogWrite(anti_clock_wise_ENA ,0);
                analogWrite(clock_wise_ENA , abs(signed_pwm_PID_OUTPUT));
            }
          
            if (signed_pwm_PID_OUTPUT < 0 )
            {
              analogWrite(anti_clock_wise_ENA , abs(signed_pwm_PID_OUTPUT));
              analogWrite(clock_wise_ENA , 0);
            }

            if (signed_pwm_PID_OUTPUT == 0 )
            {
              analogWrite(anti_clock_wise_ENA , 0);
              analogWrite(clock_wise_ENA , 0);
            }

            
      }


void initiate_control()
      {
        CONTROL_SIGNAL_START_TIME=millis();
      }

void generate_control_signal_point( int max_angle , int min_angle ,float i2e ,float breath_per_min)
      {
            if(max_angle != CONTROL_INPUT_SIGNAL_UPPER_ANGLE || min_angle != CONTROL_INPUT_SIGNAL_LOWER_ANGLE ||  i2e !=  I2E ||  breath_per_min !=BREATH_PER_MIN)
            {
                CONTROL_SIGNAL_START_TIME=millis();
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

          // we might need to swap inhale time with exhale time definition based on our definition of zero position relative to the ambu bag
          //            I_time=Periodic_time-I_time;
          //            E_time=Periodic_time/(1+I2E);
            
           current_running_time   = millis()- CONTROL_SIGNAL_START_TIME;
           current_cycle_position = long(current_running_time) % long(Periodic_time);
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
//control functions -------------------------------------------------------------------------------------------------------------------


//encoder functions******************************************************************************************************************
inline  void PinA_ISR()
      {  
          cli(); //stop interrupts happening before we read pin values
          int a= digitalRead(pinA);
          int b= digitalRead(pinB);
          if (a==1 && b==1 && aFlag)
          {
            //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
            ENCODER_CLICKS ++; //decrement the encoder's position count
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
              ENCODER_CLICKS --; //increment the encoder's position count
              bFlag = 0; //reset flags for the next turn
              aFlag = 0; //reset flags for the next turn
            }
            else if (a==1 && b==0) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
            sei(); //restart interrupts
            ANGLE = float(ENCODER_CLICKS) / 2000 * 360;

      }

inline void PinZ_ISR()
      {
          ZERO_FLAG = 1;
      }

 void set_zero ()
      {
          ENCODER_CLICKS = 0;
      }

int discover_zero()   
      {
         analogWrite(anti_clock_wise_ENA ,80);  //motor driver specific code .. change this if you have a different interface
         while(ZERO_FLAG==0){    };//wait till the zchannel interrupt changes ZERO_FLAG to 1;
         analogWrite(anti_clock_wise_ENA , 0);  //motor driver specific code .. change this if you have a different interface
         ENCODER_CLICKS=0;
         ZERO_FLAG=0;
         return (1);

      }
              
void move_to_angle(double goal_angle)
      {
          while(ANGLE < goal_angle)
          {  
             //Serial.println(ANGLE);
             analogWrite(clock_wise_ENA ,80);  //motor driver specific code .. change this if you have a different interface  
          }
          
          analogWrite(clock_wise_ENA ,0);  //motor driver specific code .. change this if you have a different interface  
      }
//encoder functions  -----------------------------------------------------------------------------------------------------------------


//Pressure Sensor Functions ***********************************************************************************************************
        void Setup_Pressure_Sensor()
                {
                   Pressure_Sensor.begin();
                }
        
        void Measure_Pressure()
                {
                    Pressure_Sensor.getEvent(&Pressure_Sensor_Event);
                    Pressure=(Pressure_Sensor_Event.pressure-993.0)*1.019744 -IINIT_PRESSURE;
                }
//Pressure Sensor Functions -------------------------------------------------------------------------------------------------------------

//Communication Functions ***********************************************************************************************************
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
      JUST_RECIEVED_CONTROL_INPUT_SIGNAL_UPPER_ANGLE =  float(Received_Data[2])/100*max_angle;
//      JUST_RECIEVED_CONTROL_INPUT_SIGNAL_LOWER_ANGLE = CONTROL_INPUT_SIGNAL_UPPER_ANGLE - (  float(CONTROL_INPUT_SIGNAL_UPPER_ANGLE*Received_Data[2]) /100  );
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
//Communication Functions --------------------------------------------------------------------------------------------------
            
