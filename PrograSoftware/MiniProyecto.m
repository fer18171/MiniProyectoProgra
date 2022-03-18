clear;
s = serialport('COM4',115200);
%%
flush(s);
while(1)
    write(s,2,'uint8');
    pause(0.01);
    if (s.NumBytesAvailable)>4
        if read(s,1,'uint16')==1026
            if read(s,1,'uint8')==4
                a = read(s,1,'int16')
            end
        end
    end
end
%%
flush(s)
pause(0.01)
s.NumBytesAvailable