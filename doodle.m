% test code to simulate readIMU

clear all;
close all;
instrreset;
disp('Press Ctrl+C to stop collecting data!')
f = 20; % DataFrequency
cnt = 1;
aa= 0;
tt = 0;
a= 0;
l = 0;
ll = 0;
tstart = tic;

% keyboard status
g_prsd = 0;
p_prsd = 0;

while(1)
    t = toc(tstart); % time stamp   
    a = sin(t) + 0.01;
    aa = [aa;a];
    tt = [tt;t]
    ll = [ll;l]
    if (cnt>(f/5)) % Plot in low frequcy
        plot(tt,aa,tt,ll);
        fig = gcf;
        cnt = 0;
        drawnow;
        value = set(fig,'KeyPressFcn', @label);
        l = value
%         if (size(aa,1)>5*f) % clear history data
%             aa = aa(f:5*f,:);
%         end
    end
    cnt=cnt+1; 
end


function temp = label(~,event)
    keypresd = event.Key;
    temp = 0;
    switch keypresd
        case 'g'
            if g_prsd == 0
                pos = 'good posture starts';
                temp = 1;
                g_prsd = 1;
            else
                pos = 'good posture ends';
                temp = 0;
                g_prsd = 0;
            end
        case 's'
            if s_prsd == 0
                pos = 'good posture starts';
                temp = 2;
                s_prsd = 1;
            else
                pos = 'good posture ends';
                temp = 0;
                s_prsd = 0;
            end
    end
    disp(pos)
end