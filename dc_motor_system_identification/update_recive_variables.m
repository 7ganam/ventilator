function [recieve_variable1, recieve_variable2, recieve_variable3]=update_recive_variables(connection,recieve_tag,recieve_variable1,recieve_variable2,recieve_variable3)

new_reading="";
if (connection.BytesAvailable > 0)
        new_reading=(fscanf(connection));
        erase(new_reading,newline);
        in_tag=str2num(new_reading)
        if (in_tag==recieve_tag)
            %-----------------------%
             new_reading=(fscanf(connection))
             recieve_variable1=str2num(new_reading)
             %-----------------------%
             new_reading=(fscanf(connection));
             recieve_variable2=str2num(new_reading)             
             %-----------------------%
             new_reading=(fscanf(connection));
             recieve_variable3=str2num(new_reading)
             %-----------------------%

        end

    
end