clear; clf; clc;

addpath('/home/manish/progs/matlab/File-Exchange/SpinCalc/');

load('VUA_Data_MattDeadlift2D','traj');
fileOut = '../../OCP/DAT/matt_deadlift2d.csv';

orderOfSegments = [
    3  0;  % Pelvis
    7  3;  % R Thigh
    8  7;  % R Shank
    9  8;  % R Foot
    10 3; % L Thigh
    11 10  % L Shank
    12 11; % L Foot
    4  3;  % Mid Trunk
    5  4;  % Upper Trunk
    6  5;  % Head
    13 5; % R UpperArm
    14 13; % R LowerArm
    15 14; % R Hand
    16 5 % L UpperArm
    17 16 % L LowerArm
    18 17];% L Hand

pelvis_len = 0.1641; % This is used to offset the pelvis center by the correct amount in the z direction
nStart = 1135; nEnd = 1385;
bSave = 1;
outAngles = zeros (nEnd-nStart,length(orderOfSegments));

for segmentID = 1:length(orderOfSegments)
    
    rotMatVec = traj.segment(orderOfSegments(segmentID,1)).R;
    if segmentID ~= 1
        rotMatVec_parent = traj.segment(orderOfSegments(segmentID,2)).R;
    end
    
    for i = 1:(nEnd-nStart)
        RotMat_global = [rotMatVec(nStart+i,1:3); rotMatVec(nStart+i,4:6); rotMatVec(nStart+i,7:9)];
        if segmentID == 1
            RotMat_local = RotMat_global;
        else
            parent_global = [rotMatVec_parent(nStart+i,1:3); rotMatVec_parent(nStart+i,4:6); rotMatVec_parent(nStart+i,7:9)];
            RotMat_local = RotMat_global*transpose(parent_global);
        end
        if segmentID == 2 || segmentID == 5
            in_EulerRot(segmentID, i,:) = SpinCalc ('DCMtoEA213', RotMat_local, 1e-06, 1);
        else
            in_EulerRot(segmentID, i,:) = SpinCalc ('DCMtoEA123', RotMat_local, 1e-06, 1);
        end
    end
    
    EulerRot(segmentID,:,1)  = fnc_remove360Jumps(in_EulerRot(segmentID,:,1)')*pi/180;
    EulerRot(segmentID,:,2)  = fnc_remove360Jumps(in_EulerRot(segmentID,:,2)')*pi/180;
    EulerRot(segmentID,:,3)  = fnc_remove360Jumps(in_EulerRot(segmentID,:,3)')*pi/180;
   
    if segmentID == 1 % Pelvis Rotation
        outAngles(1:(nEnd-nStart),segmentID) = -squeeze(EulerRot(segmentID,:,1))' + 2*pi;        
    elseif segmentID == 8 % Mid Trunk
        outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))' - 2*pi;
    elseif segmentID == 9 % Upper Trunk
        outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))';
    elseif segmentID == 10 % Head
        outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))' - 2*pi;
    elseif segmentID >= 11 && segmentID <= 15
        outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))' - 2*pi;
    elseif segmentID == 16
         outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))';
    elseif segmentID == 4 || segmentID == 7
          outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))' - 2*pi;
    elseif segmentID == 2 || segmentID == 5
         outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,1))' - 2*pi;
    elseif segmentID == 3 || segmentID == 6
         outAngles(1:(nEnd-nStart),segmentID) = squeeze(EulerRot(segmentID,:,2))';
    else        
        outAngles(1:(nEnd-nStart),segmentID) = zeros(250,1);
    end
    
    clear rotMatVec rotMatVec_parent
end

% Using 100 Hz instead of 50Hz to speed up motion 2x
timeStamp = [0:1/100:(nEnd-nStart)/100-1/100];
outCSV = [timeStamp' traj.segment(3).origin(nStart:nEnd-1,2) traj.segment(3).origin(nStart:nEnd-1,3)-pelvis_len/2 outAngles];

hold on;
[a,b] = size(outCSV);
for i = 2:b
    plot(outCSV(:,1), outCSV(:,i));
end

if bSave
    [a,b] = size(outCSV);
    fid = fopen (fileOut,'w');
    for i = 1:a
        for j=1:b
            if j < b
                fprintf (fid, '%f, ',outCSV(i,j));
            else
                fprintf (fid, '%f\n',outCSV(i,j));
            end
        end
    end
    fclose(fid);
    disp(['Wrote file ' ,fileOut]);
end


