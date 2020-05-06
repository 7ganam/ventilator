load("static_test_data")
time_stamps_in_seconds=recieve_variable3_Array/1000;
voltag_levels=recieve_variable2_Array;
encoder_data_in_degrees=recieve_variable1_Array;

time_stamps_in_seconds

noise_speed_data= diff(encoder_data_in_degrees)./diff(time_stamps_in_seconds);
filter1_data=smoothdata(speed,'movmedian',50);
filter2_data = movmean(filter1_data,100);

plot(encoder_data_in_degrees)
% plot(noise_speed_data)
% plot(filter1_data*1000,'LineWidth',5)
% plot(filter2_data)

% B = smoothdata(speed,'movmedian',50)