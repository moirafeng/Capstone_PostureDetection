clear all;
close all;
instrreset;
disp('Press Ctrl+C to stop collecting data!')
s=serial('/dev/cu.HC-06-DevB','baudrate',115200) ;fopen(s) ; %Open Com Port
f = 20; %DataFrequency
cnt = 1;
aa=[0 0 0];
ww=[0 0 0];
AA = [0 0 0];
tt = 0;
a=[0 0 0]';
w=[0 0 0]';
A=[0 0 0]';
tstart = tic;
while(1)
    t = toc(tstart)
    Head = fread(s,2,'uint8');
    if (Head(1)~=uint8(85))
        continue;
    end    
    switch(Head(2))
        case 81 
            a = fread(s,3,'int16')/32768*16 ;     
            End = fread(s,3,'uint8');
        case 82 
            w = fread(s,3,'int16')/32768*2000 ;    
            End = fread(s,3,'uint8');
        case 83 
            A = fread(s,3,'int16')/32768*180;
            aa=[aa;a'];
            ww = [ww;w'];
            AA = [AA;A'];
            tt = [tt;t];
            if (cnt>(f/5)) %Plot in low frequce, 
                subplot(3,1,1);plot(tt,aa);title(['Acceleration = ' num2str(a') ' m/s^2']);ylabel('m/s^2');
                legend('x','y','z');
                subplot(3,1,2);plot(tt,ww);title(['Gyro = ' num2str(w') ' degree/s']);ylabel('degree/s');
                legend('x','y','z');
                subplot(3,1,3);plot(tt,AA);title(['Angle = ' num2str(A') ' degrees']);ylabel('degrees');  
                legend('x','y','z');
                cnt = 0;
                drawnow;
                if (size(aa,1)>5*f)%clear history data
                    aa = aa(f:5*f,:);
                    ww = ww(f:5*f,:);
                    AA = AA(f:5*f,:);
                    tt = tt(f:5*f,:);
                end
            end
            cnt=cnt+1;
            End = fread(s,3,'uint8');
    end    
end
fclose(s);
