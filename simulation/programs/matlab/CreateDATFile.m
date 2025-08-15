clear; 
clc;
close all;

disp('----------------------------------------');
disp('Important Notes');
disp('1. Start the OCP using the Lua model with Baumgarte enabled');
disp('   or else MUSCOD will not find an initial solution.');
disp('2. At kkt=1e-3 you can safely switch to the Baumgarte ');
disp('   disabled Lua model if you wish.');
disp('3. At kkt=5e-5 I noticed that the solution did not seem');
disp('   to converge. It only converged when I changed the ');
disp('   Levenberg-Marquardt coefficient from 0.1 to 0.4');
disp('MMillard 6 March 2017');
disp('----------------------------------------');

%% DAT File Configuration
bPlot_States = 1;
stoopIdx     = 2;

bMinActSq = 1; %Solving a min. activation squared problem?
bMinTauSq = 0; %Solving a min. torque squared problem?
               %At the moment 

assert((bMinActSq || bMinTauSq) == 1 ...
    && (bMinActSq && bMinTauSq) == 0,...
      'Error: can only choose 1 cost function');

nPosExo      = 9;
nControlsExo = 2;
nStatesTauDotExo = 0;

%MM 16 Feb 2016
% If you want to add activation dynamics 
% 1. Set the flag below to be 1.
% 2. Go into wholebodylifter2d_Enums.h 
% uncomment the #define _ACTIVATION_DYNAMICS_ line and recompile
flag_addActivationDynamics = 0;
nStatesActivation = 18;

%assert( (flag_addActivationDynamics==1 && nStatesTauDotExo == 2) || ...
%        (flag_addActivationDynamics==0 && nStatesTauDotExo == 0),...
%        'Incompatible configuration');

nPhases  = 3;
nShoot   = [15, 12, 15];
idxPhase = [1,2,3];

nParams = 2;
idxParams = [1,2];

flag_fixStartPose = 0;
flag_fixEndPose   = 0;

fz = 100;
fx = 10;
d  = 0.1;
v  = 1;

qC           = ones(1,15).*d;
qDotC        = ones(1,15).*v;
qFootZ       =  ones(1,2).*d;
qFootXOffset = d;
pXYVel       = ones(1,2).*v; % pelvis XY vel

feetVel  = ones(1,3).*v;
feetFxz  = ones(1,4).*fz;
legBox   = ones(1,2).*d;

handXY    = ones(1,2).*d;
handXYVel = ones(1,2).*v;

handBoxRotY    = ones(1,1).*d;
handBoxRotVelY = ones(1,1).*v;
handBoxVel     = ones(1,3).*v;
handBoxFX      = ones(1,3).*fz;

bodyExoPos     = ones(1,nPosExo).*d;
bodyExoVel     = ones(1,nPosExo).*v;

boxPos   = ones(1,3).*d;
boxVel   = ones(1,3).*v;

qDotPelvis = ones(1,3).*v;

lumbarC = d;
timeId  = 1;

                            
rd_sca(1).S = [qC bodyExoPos boxVel bodyExoVel feetVel]; %Eq
rd_sca(1).i = [lumbarC, ...                              %Eq
                    feetFxz];                            %Ieq                
rd_sca(2).S = [handXY handXYVel handBoxRotY handBoxRotVelY lumbarC timeId, ...    %Eq
                    feetFxz legBox];                     %Ieq
rd_sca(2).i = [lumbarC  handBoxFX,...                    %Eq
                    feetFxz legBox];                     %Ieq                
rd_sca(3).S = [timeId,  lumbarC, fz,fz,fz, ...           %Eq
                    feetFxz legBox];                     %Ieq
rd_sca(3).i = [lumbarC, ...                              %Eq
                    feetFxz legBox ];                    %Ieq
rd_sca(3).E = [ d,d,d, v,v,v, v,v,v, lumbarC, ...        %Eq
                    feetFxz  legBox];                    %Ieq

assert(length(rd_sca) == nPhases,'Mismatch between nPhases and rd_sca');

%% INPUT
idFolder       = '../../OCP/PRE/ID/2D/';
vua2uheiFolder = '../../OCP/PRE/VUA2UHEI/2D/';

timeExp = csvread(sprintf('%stime_stoop%d.csv', idFolder,stoopIdx));
qExp    = csvread(sprintf('%sq_stoop%d.csv'   , idFolder,stoopIdx));
qDotExp = csvread(sprintf('%sqDot_stoop%d.csv', idFolder,stoopIdx));  

assert(size(timeExp,1) == size(qExp,1) ...
    && size(timeExp,1) == size(qDotExp,1), ...
    'q and qdot sense of time differs');
assert(size(qExp,2) == size(qDotExp,2), ...
    'q and qdot have different number of columns');


qIdxExo            = [1,2,3,4,5,6,7,8,9];
qIdxExoFreeFlyer   = qIdxExo(1:3);

qIdxBox            = [10,11,12];
qIdxBoxFreeFlyer   = qIdxBox(1:3);

qIdxHuman          = [13,14,15, 16,17,18, 19,20,21, 22,23,24];
qIdxHumanFreeFlyer = qIdxHuman(1:3); 

qIdIdxBox            = [1,2,3];
qIdIdxBoxFreeFlyer   = qIdIdxBox(1:3);

qIdIdxHuman          = [4,5,6, 7,8,9, 13,14,15, 16,17,18];
qIdIdxHumanFreeFlyer = qIdIdxHuman(1:3); 

%% Muscle Data
musclePropsFileName = [idFolder, 'muscleProperties.csv'];

jointMuscleProps = struct('header',{''},'data',zeros(3,3));
jointMuscleProps.data = csvread(musclePropsFileName,1,0);
jointMuscleProps.header = {'qIdx','omegaMax','damping',...
    'mIdxE','torqueSignE','tauMaxE','omegaMaxE',...
    'mIdxF','torqueSignF','tauMaxF','omegaMaxF'};
assert(length(jointMuscleProps.header) == size(jointMuscleProps.data,2),...
       'Muscle property data and header do not match');


fpExp     = csvread(sprintf('%sstoop%d_F1Cop1F2Cop2_2D.csv',...
                             vua2uheiFolder,stoopIdx));
assert(size(fpExp,1)==size(qExp,1),...
       'fp and q files have different sense of time');
assert(size(fpExp,2)== 13,...
       'fp has different number of columns than expected.');
                            
assert(length(qIdxBox)   ==  3,'qIdxBox has the incorrect length');
assert(length(qIdxHuman) == 12,'qIdxHuman has the incorrect length');
assert(length(qIdxExo)   == nPosExo, 'qIdxExo has the incorrect length'); 

%% DAT Path
dat_path = '../../OCP/DAT';
fid_dat  = fopen([dat_path,'/wholebodylifter2d.dat'],'w');

idxFpBoxFz      = 4;
idxStart        = 1;
idxEnd          = size(qExp,1);
idxHandOnBox    = 0;
idxBoxOffGround = 0;

%The Kistler force plate for the box did not have its bias set to 0.
%Doing that here.
boxFzWeight         = (fpExp(end,idxFpBoxFz));
fpExp(:,idxFpBoxFz) = fpExp(:,idxFpBoxFz) -boxFzWeight;
boxFzWeight         = mean(fpExp(1:20,idxFpBoxFz));
boxFzWeightRange    = max(fpExp(1:20,idxFpBoxFz)) ...
                    - min(fpExp(1:20,idxFpBoxFz));

for i=1:1:size(qExp,1)
    
    boxFzDist = (fpExp(i,idxFpBoxFz) - boxFzWeight)/boxFzWeightRange;
    
    if( abs(boxFzDist) > 2 && idxHandOnBox == 0)
       idxHandOnBox = i; 
    end
    if( fpExp(i,idxFpBoxFz) < boxFzWeightRange*2 ...
            && idxHandOnBox > 0 && idxBoxOffGround == 0)
       idxBoxOffGround = i;
    end
    
end

events_all = [idxStart idxHandOnBox idxBoxOffGround idxEnd];

events = events_all(min(idxPhase):1:(max(idxPhase)+1));

%% OUTPUT
RootDir  = '../../';
dat_path = [RootDir,'OCP/DAT'];


%% PHASE TIMES

h_all      = [(timeExp(events_all(2))-timeExp(events_all(1))), ...
              (timeExp(events_all(3))-timeExp(events_all(2))), ...
              (timeExp(events_all(4))-timeExp(events_all(3)))];

h_all(3) = h_all(3).*0.8;

h_allNames = {'Bend','Pause','Lift'};

h       = h_all(idxPhase);
h_name  = h_allNames(idxPhase);

h_min = [];
if(bMinTauSq)
  h_min = [1.1 0.5 0.75];
end
if(bMinActSq)
  h_min = [1.1 0.5 0.5];
end

h_max = [];
if(bMinTauSq)
  h_max = h*1.2;
  h_max(1) = 2.5;
end
if(bMinActSq)
  h_max = h*1.2;
end

h_fix   = [1 1 1];
%The loading phase timing is not of particular interest
%So that it does not affect the comparision its duration is 
%fixed to match the experimental duration of phase 2.


%% Human states
humanPoseStart   = qExp(events_all(min(idxPhase)),   qIdIdxHuman);             
boxPoseStart     = qExp(events_all(min(idxPhase)),   qIdIdxBox);
exoPoseStart     = [humanPoseStart(1:3),...
                    humanPoseStart(4),0,0, ...
                sum(humanPoseStart(7:8)),0,0];
            
humanPoseEnd     = qExp(events_all(max(idxPhase)+1), qIdIdxHuman);             
boxPoseEnd       = qExp(events_all(max(idxPhase)+1), qIdIdxBox);
exoPoseEnd    = [humanPoseEnd(1:3),...
                    humanPoseEnd(4),0,0, ...
                sum(humanPoseEnd(7:8)),0,0];




startState = [exoPoseStart   , boxPoseStart   , humanPoseStart, ...
              exoPoseStart.*0, boxPoseStart.*0, humanPoseStart.*0];
          
endState   = [exoPoseEnd   , boxPoseEnd   , humanPoseEnd, ...
              exoPoseEnd.*0, boxPoseEnd.*0, humanPoseEnd.*0];

%%
%Get the initial states at each shooting node. Here we 
% 1. Set all velocities to 0
% 2. Linearly interpolate between positions at the event locations
%%
nStatesHuman = length(qIdxHuman)*2;
nStatesBox   = length(qIdxBox)*2;
nStatesExo   = length(qIdxExo)*2;

actStates = [];
if(flag_addActivationDynamics == 1)
   actStates = ones(size(qExp,1), nStatesActivation).*0.11;
else
   nStatesActivation = 0; 
end

nStates      = nStatesBox + nStatesExo + nStatesHuman + nStatesActivation + nStatesTauDotExo;

colMapExp2Model = [qIdIdxBox,qIdIdxHuman];
colMapModel2Exp = [qIdxBox,qIdxHuman];

c0 = zeros(size(qExp,1),1);
qExpExo = [qExp(:,qIdIdxHumanFreeFlyer), ...
           qExp(:, qIdIdxHuman(4)) c0 c0 ...
           sum(qExp(:, qIdIdxHuman(7:8)),2) c0 c0];

exoTauInitial = [];
if(nStatesTauDotExo > 0)
    exoTauInitial = ones(size(timeExp,1),nStatesTauDotExo).*0.05;
end       
       
       
[statesAll, statesShoot, timeStampAll] = ...
      fnc_formatStateVectors (timeExp, ...
                              [qExpExo,    qExp(:,colMapExp2Model)],    ...
                              [qExpExo,   qDotExp(:,colMapExp2Model)].*0, ... 
                              [actStates, exoTauInitial], ...
                              events, nPhases, nShoot, nStates);
        
[nFrames, ~] = size(statesAll);

statesAll_aug = [statesAll];
statesAll_aug(end) = 0;
statesShoot_aug = [statesShoot];
statesShoot_aug(end) = 0;

if(flag_fixStartPose)
   statesShoot_aug(1,:)   =  startState;
end
if(flag_fixEndPose)
   statesShoot_aug(end,:) =  endState;
end


fid_initialStates = fopen([dat_path, '/initialStates.csv'], 'w');
for i=1:length(events)
    idx = events(i)-min(events(i)) + 1;
    fprintf(fid_initialStates,'%f, ',timeStampAll(idx));
    for j=1:nStates/2
        if j < nStates/2
            fprintf(fid_initialStates,'%f, ',statesAll(idx,j));
        else
            fprintf(fid_initialStates,'%f\n',statesAll(idx,j));
        end
    end
end
res = fclose(fid_initialStates);


%% STATE LIMITS
getSdMinSdMaxSdScaSdLabels;


%% CONTROLS
getUminUMaxUScaULabels;

%% PARAMETERS
p_all = [timeExp(idxHandOnBox) timeExp(idxBoxOffGround)] ...
        - timeExp(min(events));

p = p_all(idxParams);


p_min = p.*0.1;
p_max = p.*2.0;
p_sca = ones(nParams,1).*0.1;
p_fix = ones(nParams,1).*0;

p_name_all = {'ParamTimeHandsOnBoxNoForce','ParamTimeHandsOnBoxFullForce'};
p_name = p_name_all(idxParams);

%% DECOUPLED i.p.c SCALE FACTORS
%rd_sca(1).S = [ones(1,3).*0.01 ones(1,1).*0.01 ones(1,9).*0.01 ones(1,nStates/2).*0.1 ones(1,4).*0.001 ones(1,2).*0.001 100.*ones(1,4) 10.*ones(1,8)];
%rd_sca(1).S = [100.*ones(1,4) 10.*ones(1,8)];


% Write interpolator data info
fprintf(fid_dat, 'interpolator_data\n');
fprintf(fid_dat, 'DAT/initialStates.csv\n');
fprintf(fid_dat, '\n');
 
% Write nShoot
fprintf(fid_dat, '* # of multiple shooting intervals on model stages\n');
fprintf(fid_dat, 'nshoot\n');
for i=1:nPhases
    fprintf(fid_dat, ' %d: %d\n', i-1, nShoot(i));
end
fprintf(fid_dat, '\n');
 
fprintf(fid_dat, '* specification mode for state variable start values\n');
fprintf(fid_dat, '* 0 : all values sd(*,*), sa(*,*) specified in data file\n');
fprintf(fid_dat, '* 1 : sd(*,S), sd(*,E), sd(M-1,S), sd(M-1,E) are given (M = number of stages), all others generated by interpolation\n');
fprintf(fid_dat, '* 2 : only sd(0,S), sa(0,S) given, all others generated by integration\n');
fprintf(fid_dat, '* 3 : only sd(0,S), sa(0,S) given, all others generated by integration, integration repeated replacing initial values until steady state acheived\n');
fprintf(fid_dat, 's_spec\n');
fprintf(fid_dat, ' 1\n');
fprintf(fid_dat, '\n');

%% WRITE PHASE TIME RELATED %%
% Write phase times, h
fprintf(fid_dat, 'h\n');
for i=1:nPhases
    fprintf(fid_dat, ' %d: %f\n', i-1, h(i));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, '*  model stage duration start values, scale factors, and bounds\n');
fprintf(fid_dat, 'h_sca\n');
fprintf(fid_dat, 'ALL: 1.0\n');
fprintf(fid_dat, '\n');

% Write minimum phase times, h_min
fprintf(fid_dat, 'h_min\n');
for i=1:nPhases
    fprintf(fid_dat, ' %d: %f\n', i-1, h_min(i));
end
fprintf(fid_dat, '\n');

% Write maximum phase times, h_max
fprintf(fid_dat, 'h_max\n');
for i=1:nPhases
    fprintf(fid_dat, ' %d: %f\n', i-1, h_max(i));
end
fprintf(fid_dat, '\n');

% Write phase time fixes, h_fix
fprintf(fid_dat, 'h_fix\n');
for i=1:nPhases
    fprintf(fid_dat, ' %d: %d\n', i-1, h_fix(i));
end
fprintf(fid_dat, '\n');

% Write phase time names, h_name
fprintf(fid_dat, 'h_name\n');
for i = 1:nPhases
    fprintf(fid_dat, ' %d: %s\n', i-1, char(h_name(i)));
end
fprintf(fid_dat, '\n');

%% WRITE STATES RELATED %%
count = 0;
for i = 1:nPhases
    for j = 1:nShoot(i)
        
        fprintf(fid_dat, 'sd(%d,%d) ! %d\n', i-1, j-1, count);           
        for k=1:nStates
            fprintf(fid_dat, ' %d: %f\n', k-1, statesShoot_aug(count+1,k));
        end
        fprintf(fid_dat, '\n');
        count = count + 1;
    end
end
% Write final state
fprintf(fid_dat, 'sd(%d,%d) ! %d\n', i-1, j, count);
for k=1:nStates
    fprintf(fid_dat, ' %d: %f\n', k-1, statesShoot_aug(count,k));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'sd_sca(*,*)\n');
for i=1:nStates
    fprintf(fid_dat, ' %d: %f\n', i-1, sd_sca(i));
end
fprintf(fid_dat, '\n');

% Write minimum parameters, sd_min
fprintf(fid_dat, 'sd_min(*,*)\n');
for i=1:nStates
    fprintf(fid_dat, ' %d: %f\n', i-1, sd_min(i));
end
fprintf(fid_dat, '\n');

% Write maximum parameters, sd_max
fprintf(fid_dat, 'sd_max(*,*)\n');
for i=1:nStates
    fprintf(fid_dat, ' %d: %f\n', i-1, sd_max(i));
end
fprintf(fid_dat, '\n');

% Write parameter names, sd_name
fprintf(fid_dat, 'xd_name\n');
for i=1:nStates
    fprintf(fid_dat, ' %d: %s\n', i-1, char(xd_name(i)));
end
fprintf(fid_dat, '\n');


if(flag_fixStartPose == 1)
    fprintf(fid_dat, 'sd_fix(%d,%d) ! %d\n', 0, 0, 0);
    for i=1:nStates                  
        fprintf(fid_dat, ' %d: %f\n', i-1, 1.0);        
    end 
    fprintf(fid_dat, '\n');
end
if(flag_fixEndPose == 1)
    fprintf(fid_dat, '\n');
    fprintf(fid_dat, 'sd_fix(%d,%d) ! %d\n', nPhases-1, nShoot(nPhases), count);
    for i=1:nStates                  
        fprintf(fid_dat, ' %d: %f\n', i-1, 1.0);        
    end 
    fprintf(fid_dat, '\n');
end

%% WRITE CONTROLS RELATED %%
fprintf(fid_dat, '* control parameterization types, start values, scale factors, and bounds\n');

% 0 = piecewise constant, 1 = piecewise linear, 2 = piecewise continuous
% linear, 3 = piecewise cubic, 4 = piecewise continuous cubic
fprintf(fid_dat, 'u_type(*)\n');
fprintf(fid_dat, 'ALL: 2\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'u(*,*)\n');
for i = 1:nControls
    fprintf(fid_dat, ' %d: %f\n', i-1, u(i));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'u_sca(*,*)\n');
for i = 1:nControls
    fprintf(fid_dat, ' %d: %f\n', i-1, u_sca(i));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'u_min(*,*)\n');
for i = 1:nControls
    fprintf(fid_dat, ' %d: %f\n', i-1, u_min(i));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'u_max(*,*)\n');
for i = 1:nControls
    fprintf(fid_dat, ' %d: %f\n', i-1, u_max(i));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'u_name\n');
for i = 1:nControls
    fprintf(fid_dat, ' %d: %s\n', i-1, char(u_name(i)));
end
fprintf(fid_dat, '\n');

%% WRITE PARAMETER RELATED %%
% Write params
fprintf(fid_dat, 'p\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %f\n', i-1, p(i));
end
fprintf(fid_dat, '\n');

% Write minimum parameters, p_min
fprintf(fid_dat, 'p_min\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %f\n', i-1, p_min(i));
end
fprintf(fid_dat, '\n');

% Write maximum parameters, p_max
fprintf(fid_dat, 'p_max\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %f\n', i-1, p_max(i));
end
fprintf(fid_dat, '\n');

% Write fixed parameters, p_fix
fprintf(fid_dat, 'p_fix\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %d\n', i-1, p_fix(i));
end
fprintf(fid_dat, '\n');

% Write fixed parameters, p_sca
fprintf(fid_dat, 'p_sca\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %d\n', i-1, p_sca(i));
end
fprintf(fid_dat, '\n');

% Write parameter names, p_name
fprintf(fid_dat, 'p_name\n');
for i=1:nParams
    fprintf(fid_dat, ' %d: %s\n', i-1, char(p_name(i)));
end
fprintf(fid_dat, '\n');

%% WRITE DECOUPLED i.p.c RELATED %%
fprintf(fid_dat, '*  decoupled i.p.c. scale factors\n');
for i=1:nPhases
    if ~isempty(rd_sca(i).S)
        fprintf(fid_dat, 'rd_sca(%d,S)\n', i-1);
        for j=1:length(rd_sca(i).S)
            fprintf(fid_dat, ' %d: %f\n', j-1, rd_sca(i).S(j));
        end
        fprintf(fid_dat, '\n');
    end
    if ~isempty(rd_sca(i).i)
        fprintf(fid_dat, 'rd_sca(%d,i)\n', i-1);
        for j=1:length(rd_sca(i).i)
            fprintf(fid_dat, ' %d: %f\n', j-1, rd_sca(i).i(j));
        end
        fprintf(fid_dat, '\n');
    end
end
fprintf(fid_dat, 'rd_sca(%d,E)\n', i-1);
for j=1:length(rd_sca(i).E)
    fprintf(fid_dat, ' %d: %f\n', j-1, rd_sca(i).E(j));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'of_name\n');
fprintf(fid_dat, '!\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, '*  objective scale and expected range; # of values in history plot\n');
fprintf(fid_dat, 'of_sca\n');
fprintf(fid_dat, '1.0\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'of_min\n');
fprintf(fid_dat, '0.0\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'of_max\n');
fprintf(fid_dat, '5.0\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, 'nhist\n');
fprintf(fid_dat, '200\n');
fprintf(fid_dat, '\n');

fprintf(fid_dat, '**********************\n* Choosing libraries *\n**********************\n');
fprintf(fid_dat, 'libmodel\nlibwholebodylifter2d\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libhessian\nhess_update\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libsolve\nsolve_slse\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libcond\ncond_stable\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libtchk\ntchk\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libmssqp\nmssqp_standard\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libeval\neval_ind_pt\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libqps\nqps_qpopt\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libplot\nplot_noplot\n');
fprintf(fid_dat, '\n');
fprintf(fid_dat, 'libind\n');

for(i=1:1:nPhases)
    fprintf(fid_dat, sprintf(' %d: ind_rkf45\n',(i-1)));
end
fprintf(fid_dat, '\n');

fprintf(fid_dat, '**********************\n* Setting algorithmic parameters *\n**********************\n');
fprintf(fid_dat, 'options_acc\n1e-5\n');
fprintf(fid_dat, 'options_ftol\n1e-8\n');
fprintf(fid_dat, 'options_itol\n1e-8\n');
fprintf(fid_dat, 'options_rfac\n0.0\n');
fprintf(fid_dat, 'options_levmar\n0.10\n');
fprintf(fid_dat, 'options_qp_featol\n1e-8\n');
fprintf(fid_dat, 'options_qp_relax\n1.1\n');
fprintf(fid_dat, 'options_frstart\n0\n');
fprintf(fid_dat, 'options_frmax\n0\n');
fprintf(fid_dat, 'options_itmax\n1000\n');
fprintf(fid_dat, 'options_plevel_screen\n0\n');
fprintf(fid_dat, 'options_plevel_file\n1\n');
fprintf(fid_dat, 'options_plevel_matlab\n1\n');
fprintf(fid_dat, 'options_bflag\n-1\n');
fprintf(fid_dat, 'options_qp_itmax\n30000\n');
fprintf(fid_dat, 'options_qp_expand\n99999999\n');
fprintf(fid_dat, 'options_sflag\n0\n');
fprintf(fid_dat, 'options_options_wflag\n0\n');
fprintf(fid_dat, 'options_cflag\n0\n');
fprintf(fid_dat, 'options_output_ps\n0\n');
fprintf(fid_dat, 'options_output_gif\n0\n');
fprintf(fid_dat, 'options_plot_stage_marker\n1\n');
fprintf(fid_dat, 'options_num_cores\n6\n');

res = fclose(fid_dat);
display(['Wrote DAT file: ', dat_path,'/wholebodylifter2d.dat']);
 
if bPlot_States
    clf;
    count = 1;
    for i = 1:length(h)
        if nShoot(i) > 1
            timeStamp_Shoot(count:count+nShoot(i)-1) = repmat(h(i)/nShoot(i),length(nShoot(i)-1),1);
        else
            timeStamp_Shoot(count) = 0;
        end
        count = length(timeStamp_Shoot)+1;
    end
    timeStamp_Shoot(count) = 0;
    timeStamp_Shoot = cumsum(timeStamp_Shoot);
    timeStamp_phases = cumsum(h);
    for i = 1:(nStatesHuman+nStatesBox+nStatesExo)/2
        subplot(5,5,i); hold on;
        plot(timeStamp_Shoot, statesShoot(:,i), 'ob', 'linewidth', 1);
        plot(timeStampAll, statesAll(:,i), '-k', 'linewidth', 1);
        plot([timeStamp_Shoot(1) timeStamp_Shoot(end)], [sd_min(i) sd_min(i)], '--r');
        plot([timeStamp_Shoot(1) timeStamp_Shoot(end)], [sd_max(i) sd_max(i)], '--r');
        for j = 1:length(timeStamp_phases)
            plot([timeStamp_phases(j) timeStamp_phases(j)], [sd_min(i) sd_max(i)], '--k');
        end
        title(char(xd_name(i)));
        xlim([timeStamp_Shoot(1) timeStamp_Shoot(end)]);
        ylim([sd_min(i) sd_max(i)]);
    end
end
