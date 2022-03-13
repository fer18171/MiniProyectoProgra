clear;
s = serialport('COM3',115200);
%%
flush(s);
while(1)
    if read(s,1,'uint16')==778
        if read(s,1,'uint8')==3
            a = read(s,1,'uint16')
        end
    end
end
%%
flush(s)
pause(0.01)
s.NumBytesAvailable