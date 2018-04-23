clear
close all

buf_size = 1000;

buf = inf(buf_size,1);
buf1 = inf(buf_size,1);
buf2 = inf(buf_size,1);
figure;
i = 0;

while true
    [pos, but] = mat_joy(0);
    [pos1, but1] = mat_joy(1);
    [pos2, but2] = mat_joy(2);
    pos1
    plot(1:buf_size, buf);
    axis([0 1000 -1 1]);
    buf = [buf(2:buf_size); pos1(3)];
    buf1= [buf(2:buf_size); pos1(1)];
    buf2= [buf(2:buf_size); pos1(2)];
    i = i + 1;
    pause(0.01);
end