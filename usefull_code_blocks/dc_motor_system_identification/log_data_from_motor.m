delete(instrfindall);
com_name = find_arduino_com_windowsOS();
connection = serial(com_name,'BaudRate',2000000);
fopen(connection);
pause(1) % withouth this the serial print dont have enough time to reach arduino

send_tag = 222;
send_variable1=12;
send_variable2=44.4;
send_variable3=5542.2;

recieve_tag =111;
encoder_in_degress=0;
pwm=0;
time_stamp_im_ms=0;

% send_send_variables(connection,send_tag,send_variable1,send_variable2,send_variable3)
encoder_in_degress_Array=[]
pwm_Array=[]
time_stamp_im_ms_Array=[]

start_time = 0
sample_time=.005
tic
while(1==1)
    current_time=toc;
    po=current_time- start_time;
    if (( current_time - start_time)>sample_time)
        [encoder_in_degress, pwm, time_stamp_im_ms]=update_recive_variables(connection,recieve_tag,encoder_in_degress,pwm,time_stamp_im_ms)
        encoder_in_degress_Array  =[encoder_in_degress_Array encoder_in_degress ];
        pwm_Array                 =[pwm_Array pwm ];
        time_stamp_im_ms_Array     =[time_stamp_im_ms_Array encoder_in_degress ];
         start_time = toc
    end
end

