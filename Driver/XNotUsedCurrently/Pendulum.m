classdef Pendulum < handle
    properties
        device
        prev_pkt_num
        msg
        rx_cnt
    end
    methods
        function open(obj, port, baud, calibrate)
            obj.device = serialport(port,baud,'Timeout',1);
%             fopen(obj.device);
            obj.prev_pkt_num = 1000;
            obj.msg = zeros(16,1);
            obj.rx_cnt = 0;

            % Disable the onboard controller
            cmd    = [170, 195, 5, 0, 0];
            cmd(5) = obj.crc8(cmd(1:4));
            write(obj.device, cmd,"uint8");
            flush(obj.device);

            % Disable streaming output
            cmd    = [170, 193, 5, 0, 0];
            cmd(5) = obj.crc8(cmd(1:4));
            write(obj.device, cmd,"uint8");
            flush(obj.device);

            % Run calibration
            if calibrate
                cmd    = [170, 194, 4, 0];
                cmd(4) = obj.crc8(cmd(1:3));
                write(obj.device, cmd,"uint8");
                flush(obj.device);

                while true
                    obj.msg(obj.rx_cnt+1) = read(obj.device, 1,"uint8");
                    obj.rx_cnt = obj.rx_cnt + 1;

                    if obj.rx_cnt == 4
                        if sum(cmd == obj.msg) ~= 4
                            obj.msg(1:end-1) = obj.msg(2:end);
                            obj.rx_cnt = obj.rx_cnt - 1;
                            continue
                        end
                        break
                    end
                end
            end

            % Enable streaming output
            cmd    = [170, 193, 5, 1, 0];
            cmd(5) = obj.crc8(cmd(1:4));
            write(obj.device, cmd,"uint8");
            flush(obj.device);

            obj.clear_buffer()
        end
      
        function close(obj)
%             close(obj.device); // not needed with serialport
            delete(obj.device);
            clear obj.device
        end

        function clear_buffer(obj)
            flush(obj.device)
            obj.prev_pkt_num = 1000;
            obj.rx_cnt = 0;
        end

        function set_motor(obj, speed)
            maxSpeed = round(7199 * 0.95);
            if speed > maxSpeed
                speed = maxSpeed;
            elseif speed < -maxSpeed
                speed = -maxSpeed;
            end
            val    = typecast(int16(speed), 'uint8');
            cmd    = [170, 200, 6, val(1), val(2), 0];
            cmd(6) = obj.crc8(cmd(1:5));
            write(obj.device, cmd,"uint8");
            flush(obj.device);
        end

        function [angle,position] = get_state(obj)
            while true
                obj.msg(obj.rx_cnt+1) = read(obj.device, 1,"uint8");
                obj.rx_cnt = obj.rx_cnt + 1;
                
                if obj.rx_cnt >= 11
                    % Message must start with SOF character 
                    if obj.msg(1) ~= 170
                        obj.msg(1:end-1) = obj.msg(2:end);
                        obj.rx_cnt = obj.rx_cnt - 1;
                        continue
                    % Check command
                    elseif obj.msg(2) ~= 204
                        obj.msg(1:end-1) = obj.msg(2:end);
                        obj.rx_cnt = obj.rx_cnt - 1;
                        continue
                    % Check message packet length
                    elseif obj.msg(3) ~= 11
                        obj.msg(1:end-1) = obj.msg(2:end);
                        obj.rx_cnt = obj.rx_cnt - 1;
                        continue
                    % Verify integrity of message
                    elseif obj.msg(11) ~= obj.crc8(obj.msg(1:10))
                        obj.msg(1:end-1) = obj.msg(2:end);
                        obj.rx_cnt = obj.rx_cnt - 1;
                        continue
                    end
                    
                    % Check for dropped packets
                    if obj.prev_pkt_num ~= 1000
                        if obj.msg(4) > obj.prev_pkt_num
                            diff = obj.msg(4) - obj.prev_pkt_num;
                        else
                            diff = (256 + obj.msg(4)) - obj.prev_pkt_num;
                        end
                        
                        if diff > 1
                            warning('Skipped packets [prev=%u now=%u]',obj.prev_pkt_num,obj.msg(4));
                        end
                    end
                    obj.prev_pkt_num = obj.msg(4);
                      
                    % Process data
                    angle    = double(typecast(uint16(bitshift(obj.msg(6),8) + obj.msg(5)), 'int16'));
                    position = double(typecast(uint16(bitshift(obj.msg(8),8) + obj.msg(7)), 'int16'));
                    
                    obj.msg(1:end-11) = obj.msg(12:end);
                    obj.rx_cnt = obj.rx_cnt - 11;
                    return
                end
            end
        end
 
        function result = crc8(obj, msg)
            result = 0;

            for ii = 1:length(msg)
                val = msg(ii);
              
                for b = 1:8
                    sum    = bitand(bitxor(result, val), 1);
                    result = bitshift(result, -1);
                    if sum > 0
                        result = bitxor(result, 140); 
                    end
                    val = bitshift(val, -1);
                end
            end
        end    
    end
end