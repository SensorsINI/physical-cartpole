%%
SERIAL_PORT         = "/dev/ttyUSB1";
BAUD_RATE           = 230400;
CALIBRATE           = false;
CONTROL_LOOP_MS     = 5;
ANGLE_SET_POINT     = 3110;
POSITION_SET_POINT  = 10000;
RUN_TIME_S          = 5;

%%
fprintf("printing state\n");

try
    p = Pendulum;
    p.open(SERIAL_PORT, BAUD_RATE, CALIBRATE);
    p.clear_buffer();

    for ii = 1:1000
        [angle ,position] = p.get_state();
        p.clear_buffer();
        fprintf("angle %d\tposition %d\n",angle, position);
       pause(.1);
     end
    p.close();

catch err
    p.close();
    rethrow(err);
end


%%
fprintf("p control for angle\n");
try
    p = Pendulum;
    p.open(SERIAL_PORT, BAUD_RATE, CALIBRATE);
    p.clear_buffer();

    for ii = 1:round((RUN_TIME_S * 1000)/CONTROL_LOOP_MS)
        [angle ,position] = p.get_state();

        cmd = -100 * (angle - ANGLE_SET_POINT);
        
        if mod((ii-1), round(300/CONTROL_LOOP_MS)) == 0
            fprintf("angle: %+05d position: %+06d motor: %+05d\n", angle, position, cmd);
        end
        
        
        cmd = round(cmd);
        p.set_motor(cmd);
    end
    p.close();

catch err
    p.close();
    rethrow(err);
end
  
