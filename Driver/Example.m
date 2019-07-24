SERIAL_PORT         = "/dev/ttyUSB0";
BAUD_RATE           = 230400;
CALIBRATE           = true;
CONTROL_LOOP_MS     = 5;
ANGLE_SET_POINT     = 3110;
POSITION_SET_POINT  = 10000;
RUN_TIME_S          = 5;

p = Pendulum;
p.open(SERIAL_PORT, BAUD_RATE, CALIBRATE);
p.clear_buffer();

for ii = 1:round((RUN_TIME_S * 1000)/CONTROL_LOOP_MS)
    [angle ,position] = p.get_state();

    cmd = -100 * (angle - ANGLE_SET_POINT);
    cmd = round(cmd);

    if mod((ii-1), round(1000/CONTROL_LOOP_MS)) == 0
        fprintf("%+05d %+06d %+05d\n", angle, position, cmd);
    end
   
    p.set_motor(cmd);
end

p.close();