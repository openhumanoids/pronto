% 350 seconds erro 

clear all, close all
fid = fopen('results-with-vo--keyframe-at-1x.txt');
d=[];

tline = fgets(fid);
counter =1
while ischar(tline)
    
    counter=counter+1;
    if mod(counter,1000) ==0
        counter
        d
    end
    tline = fgets(fid);
    if (tline == -1)
        break
    end
    
    if (tline(1) == '#')
        keyboard
    end
    fields = strsplit(tline,', ');
    vals = str2double(fields(2:end));
    channel = strtrim(fields{1});
    if isfield(d,channel)
        d.(channel) = [d.(channel) ; vals];
    else
        d.(channel) = vals;
    end
end

fclose(fid);


tmin = d.('POSE_BODY')(1,1)

stringnames = {'POSE_VICON','POSE_BODY', 'POSE_BODY_ALT'}
figure
subplot(2,1,1)
hold on
cols = 'gcm'
for i=1:3
  s = stringnames{i};
  d.(s)(:,1)  = (d.(s)(:,1) - tmin)*1E-6;
  plot(d.(s)(:,1), d.(s)(:,2),cols(i))
end

subplot(2,1,2)
hold on
error = sqrt(sum(power(d.('POSE_VICON')(:,2:4) - d.('POSE_BODY')(:,2:4),2),2));
plot(d.(s)(:,1), error,'c')
error = sqrt(sum(power(d.('POSE_VICON')(:,2:4) - d.('POSE_BODY_ALT')(:,2:4),2),2));
plot(d.(s)(:,1), error,'m')