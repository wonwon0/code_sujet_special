% t is the Socket Connection handle
% P is translation in mm and rotation vector
function P = readrobotspeed_j(t)

if t.BytesAvailable>0
    fscanf(t,'%c',t.BytesAvailable);
end
fprintf(t,'(5)'); % task = 2 : reading task
while t.BytesAvailable==0
end
rec = fscanf(t,'%c',t.BytesAvailable);
if ~strcmp(rec(1),'[') || ~strcmp(rec(end),']')
    error('robotpose_j read error')
end
rec(end) = '';
rec(1) = '';
P=str2num(rec);
% Curr_c = 2;
% for i = 1 : 6
%     C = [];
%     Done = 0;
%     while(Done == 0)
%         Curr_c = Curr_c + 1;
%         if strcmp(rec(Curr_c) , ',')
%             Done = 1;
%         else
%             C = [C,rec(Curr_c)];
%         end
%     end
%     P(i) = str2double(C);   
% end
for i = 1 : length(P)
    if isnan(P(i))
        error('robotpose read error (Nan)')
    end
end
end