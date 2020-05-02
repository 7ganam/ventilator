function send_send_variables(connection,send_tag,send_variable1,send_variable2,send_variable3)

 send_variable1
 send_variable2
 send_variable3

    pause(1);
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',num2str(send_tag),'async');
    pause(.01);
    
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',"*",'async');
    pause(.01);
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',num2str(send_variable1),'async');
    pause(.01);
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',"*",'async');
    pause(.01);
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',num2str(send_variable2),'async');
    pause(.01);
    fprintf(connection,'%s',"*",'async');
    pause(.01);
    while(get(connection,'TransferStatus')~="idle")end
    fprintf(connection,'%s',num2str(send_variable3),'async');
    pause(.01);
    pause(1);
