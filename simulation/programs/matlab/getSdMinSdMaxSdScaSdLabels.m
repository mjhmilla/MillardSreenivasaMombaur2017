

qBoxMin = [-2,-2,-2];
qBoxMax = [2,  2, 2];



qPelvisMin = [-1,-0.5, -pi];
qPelvisMax = [ 1, 1.5,  pi];
%                   HipExt KneeFlex  AnklePlantarFlex
qHipKneeAnkleMax = [ pi/6, pi*(3/2), pi/3]; 
%                   HipFlex KneeExt AnkleDorsiFlex
qHipKneeAnkleMin = [ -pi,    0,     -pi/4];   

qLumbarMin = [-30].*(pi/180); % extension
qLumbarMax = [ 57].*(pi/180); % flexion

qHeadMin = -pi/3; %Chin up
qHeadMax =  pi/3; %Chin down

%                         Sh.Flex      Elbow Flex    Wrist (ulnar dev.)
qShoulderElbowWristMin = [-pi*(7/6)   -pi*(3/2)        -pi/3];
%                         Sh.Ext     Elbow Ext       Wrist (radial dev.)
qShoulderElbowWristMax = [ pi/3         0.1            pi/3];

qExoMin = [qPelvisMin, -pi, -0.5, -pi/2, -pi/2, -0.5, -pi/2]; 
qExoMax = [qPelvisMax,  pi,  0.5,  pi/2,  pi/2,  0.5,  pi/2]; 

assert(nPosExo == 9,'Update qMin, qDotMin, qDotMax, qMax for the exo');

q_min =    [qExoMin(1), ...                 %1   StateExoX
            qExoMin(2), ...                 %2   StateExoZ
            qExoMin(3), ...                 %3   StateExoRotY
            qExoMin(4), ...                 %4   StateExoThighBarRotY
            qExoMin(5), ...                 %5   StateExoThighModulePosZ
            qExoMin(6), ...                 %6   StateExoThighModuleRotY
            qExoMin(7), ...                 %7   StateExoTorsoBarRotY
            qExoMin(8), ...                 %8   StateExoTorsoModulePosZ
            qExoMin(9), ...                 %9   StateExoTorsoModuleRotY
            qBoxMin(1),...                  %10  StateBoxPosX ,
            qBoxMin(2),...                  %11  StateBoxPosZ,
            qBoxMin(3),...                  %12  StateBoxRotY,
            qPelvisMin(1),...               %13  StatePelvisPosX',
            qPelvisMin(2) ,...              %14  StatePelvisPosZ',
            qPelvisMin(3),...               %15  StatePelvisRotY',
            qHipKneeAnkleMin(1),...         %16  StateRightHipRotY',
            qHipKneeAnkleMin(2) ,...        %17  StateRightKneeRotY',
            qHipKneeAnkleMin(3),...         %18  StateRightAnkleRotY',
            qLumbarMin/2 ,...               %19  StateMiddleTrunkRotY',
            qLumbarMin/2 ,...               %20  StateUpperTrunkRotY',...
            qHeadMin ,...                   %21  StateHeadRotY',
            qShoulderElbowWristMin(1) ,...  %22  StateRightUpperArmRotY',
            qShoulderElbowWristMin(2) ,...  %23  StateRightLowerArmRotY',...
            qShoulderElbowWristMin(3) ];    %24  StateRightHandRotY',
                                        
        
     
q_max =    [qExoMax(1), ...                 %1   StateExoX
            qExoMax(2), ...                 %2   StateExoZ
            qExoMax(3), ...                 %3   StateExoRotY
            qExoMax(4), ...                 %4   StateExoThighBarRotY
            qExoMax(5), ...                 %5   StateExoThighModulePosZ
            qExoMax(6), ...                 %6   StateExoThighModuleRotY
            qExoMax(7), ...                 %7   StateExoTorsoBarRotY
            qExoMax(8), ...                 %8   StateExoTorsoModulePosZ
            qExoMax(9), ...                 %9   StateExoTorsoModuleRotY
            qBoxMax(1),...                  %10  StateBoxPosX ,
            qBoxMax(2),...                  %11  StateBoxPosZ,
            qBoxMax(3),...                  %12  StateBoxRotY,
            qPelvisMax(1),...               %13  StatePelvisPosX',
            qPelvisMax(2) ,...              %14  StatePelvisPosZ',
            qPelvisMax(3),...               %15  StatePelvisRotY',
            qHipKneeAnkleMax(1),...         %16  StateRightHipRotY',
            qHipKneeAnkleMax(2) ,...        %17  StateRightKneeRotY',
            qHipKneeAnkleMax(3),...         %18  StateRightAnkleRotY',
            qLumbarMax/2 ,...               %19  StateMiddleTrunkRotY',
            qLumbarMax/2 ,...               %20  StateUpperTrunkRotY',...
            qHeadMax ,...                   %21  StateHeadRotY',
            qShoulderElbowWristMax(1) ,...  %22  StateRightUpperArmRotY',
            qShoulderElbowWristMax(2) ,...  %23  StateRightLowerArmRotY',...
            qShoulderElbowWristMax(3) ];    %24  StateRightHandRotY',
        
        
qDotBoxMin = [-10,-10,-10];
qDotBoxMax = [ 10, 10,10];


qDotPelvisMin = [-10,-10, -10*pi];
qDotPelvisMax = [ 10, 10,  10*pi];        
        
qDotHumanJointOmegaMax = jointMuscleProps.data(:,2);

qDotExoMin = [qDotPelvisMin, -10*pi, -4, -10*pi, -10*pi, -4, -10*pi];
qDotExoMax = [qDotPelvisMax,  10*pi,  4,  10*pi,  10*pi,  4,  10*pi];

for k=1:1:size(jointMuscleProps.data,1)
    assert(jointMuscleProps.data(k,1) == 5+k,...
           'Joint property index mismatch');
end


qdot_min = [qDotExoMin(1), ...              %  StateExoX
            qDotExoMin(2), ...              %  StateExoZ
            qDotExoMin(3), ...              %  StateExoRotY
            qDotExoMin(4), ...              %  StateExoThighBarRotY
            qDotExoMin(5), ...              %  StateExoThighModulePosZ
            qDotExoMin(6), ...              %  StateExoThighModuleRotY
            qDotExoMin(7), ...              %  StateExoTorsoBarRotY
            qDotExoMin(8), ...              %  StateExoTorsoModulePosZ
            qDotExoMin(9), ...              %  StateExoTorsoModuleRotY				
            qDotBoxMin(1),...               %  StateBoxPosX ,
            qDotBoxMin(2),...				%  StateBoxPosZ,
            qDotBoxMin(3),...				%  StateBoxRotY,
            qDotPelvisMin(1),...			% 'StatePelvisVelX',
            qDotPelvisMin(2),...			% 'StatePelvisVelZ',
            qDotPelvisMin(3),...			% 'StatePelvisRotVelY',...
           -qDotHumanJointOmegaMax(1),...	% 'StateRightHipRotVelY',
           -qDotHumanJointOmegaMax(2),...	% 'StateRightKneeRotVelY',
           -qDotHumanJointOmegaMax(3),...	% 'StateRightAnkleRotVelY',
           -qDotHumanJointOmegaMax(7),...	% 'StateMiddleTrunkRotVelY',
           -qDotHumanJointOmegaMax(8),...	% 'StateUpperTrunkRotVelY',
           -qDotHumanJointOmegaMax(9),... % 'StateHeadRotVelY',
           -qDotHumanJointOmegaMax(10),...% 'StateRightUpperArmRotVelY',
           -qDotHumanJointOmegaMax(11),...% 'StateRightLowerArmRotVelY',
           -qDotHumanJointOmegaMax(12)];  % 'StateRightHandRotVelY',...
           
qdot_min = qdot_min.*0.25;       
        
qdot_max = [qDotExoMax(1), ...              %  StateExoX
            qDotExoMax(2), ...              %  StateExoZ
            qDotExoMax(3), ...              %  StateExoRotY
            qDotExoMax(4), ...              %  StateExoThighBarRotY
            qDotExoMax(5), ...              %  StateExoThighModulePosZ
            qDotExoMax(6), ...              %  StateExoThighModuleRotY
            qDotExoMax(7), ...              %  StateExoTorsoBarRotY
            qDotExoMax(8), ...              %  StateExoTorsoModulePosZ
            qDotExoMax(9), ...              %  StateExoTorsoModuleRotY
            qDotBoxMax(1),...				%  StateBoxPosX ,
            qDotBoxMax(2),...				%  StateBoxPosZ,
            qDotBoxMax(3),...				%  StateBoxRotY,
            qDotPelvisMax(1),...			% 'StatePelvisVelX',
            qDotPelvisMax(2),...			% 'StatePelvisVelZ',
            qDotPelvisMax(3),...			% 'StatePelvisRotVelY',...
            qDotHumanJointOmegaMax(1),...	% 'StateRightHipRotVelY',
            qDotHumanJointOmegaMax(2),...	% 'StateRightKneeRotVelY',
            qDotHumanJointOmegaMax(3),...	% 'StateRightAnkleRotVelY',
            qDotHumanJointOmegaMax(7),...	% 'StateMiddleTrunkRotVelY',
            qDotHumanJointOmegaMax(8),...	% 'StateUpperTrunkRotVelY',
            qDotHumanJointOmegaMax(9),... % 'StateHeadRotVelY',
            qDotHumanJointOmegaMax(10),...% 'StateRightUpperArmRotVelY',
            qDotHumanJointOmegaMax(11),...% 'StateRightLowerArmRotVelY',
            qDotHumanJointOmegaMax(12)];% 'StateRightHandRotVelY',...
       
qdot_max = qdot_max.*0.25;       
       
disp('Note: limiting qdot to be 0.25 omega_max of each joint');

%%
%Check all of the limits agains the experimental kinematic data 
%%
qMinExp = min(qExp);
qMaxExp = max(qExp);
qDotMinExp = min(qDotExp);
qDotMaxExp = max(qDotExp);

% disp('=========================================');
% disp('Using Experimental data for qmin and qmax');
% disp('=========================================');
% qdot_max_mag = max( [abs(qDotMinExp).*3; abs(qDotMaxExp).*3]);
% qdot_min = -1.*qdot_max_mag;
% qdot_max =  1.*qdot_max_mag;

disp('Checking q_min and q_max against experimental kinematics');
disp('joints where the limits on q_min and q_max are too tight are starred *');
disp(sprintf('Idx,\tqMin,\tqMinE,\tqMaxE,\tqMax'));
for i=1:1:length(colMapExp2Model)
   note = '';
   if(qMinExp(colMapExp2Model(i)) < q_min(colMapModel2Exp(i)) || ...
      qMaxExp(colMapExp2Model(i)) > q_max(colMapModel2Exp(i)))
      note = '*'; 
   end
   disp(sprintf('%s%i\t%.2f\t%.2f\t%.2f\t%.2f', ...
        note,i,q_min(colMapModel2Exp(i)), qMinExp(colMapExp2Model(i)), ...
        qMaxExp(colMapExp2Model(i)), q_max(colMapModel2Exp(i)))); 
end
disp('Checking qdot_min and qdot_max against experimental kinematics');
disp('joints where the limits on qdot_min and qdot_max are too tight are starred *');
disp(sprintf('Idx, qDotMin, qDotMinE, qDotMaxE, qDotMax'));
for i=1:1:length(colMapExp2Model)
   note = '';
   if(qDotMinExp(colMapExp2Model(i)) < qdot_min(colMapModel2Exp(i)) || ...
      qDotMaxExp(colMapExp2Model(i)) > qdot_max(colMapModel2Exp(i)))
      note = '*'; 
   end    
   disp(sprintf('%i\t%.2f\t%.2f\t%.2f\t%.2f', ...
        i,qdot_min(colMapModel2Exp(i)), qDotMinExp(colMapExp2Model(i)), ...
        qDotMaxExp(colMapExp2Model(i)), qdot_max(colMapModel2Exp(i)))); 
end
        
act_min = zeros(1,nStatesActivation);
act_max = ones(1,nStatesActivation);        

exoTauDot_min = ones(1,nStatesTauDotExo).*-1;
exoTauDot_max = ones(1,nStatesTauDotExo).*-1;
exoTauDot_sca = (exoTauDot_max -  exoTauDot_min).*0.125;

%Works but is slow
%q_sca    = 0.25*(q_max-q_min);
%qdot_sca = 0.125*(qdot_max-qdot_min);

qIdxExo = [1:1:nPosExo];


q_sca          = 0.125.*(q_max-q_min);
q_sca(qIdxExo) = 0.125.*(q_max(qIdxExo)-q_min(qIdxExo));

qdot_sca          = 0.0625.*(qdot_max-qdot_min);
qdot_sca(qIdxExo) = 0.0625.*(qdot_max(qIdxExo)-qdot_min(qIdxExo));

act_sca  = 0.0625.*(act_max - act_min);



  sd_min = [q_min, qdot_min];               
  sd_max = [q_max, qdot_max];   
  sd_sca = [q_sca, qdot_sca];  
  
if(flag_addActivationDynamics == 1)
  sd_min = [sd_min, act_min];               
  sd_max = [sd_max, act_max ];   
  sd_sca = [sd_sca, act_sca ];  
end

if(nStatesTauDotExo > 0)
  sd_min = [sd_min, exoTauDot_min];               
  sd_max = [sd_max, exoTauDot_max ];   
  sd_sca = [sd_sca, exoTauDot_sca ];  
end

xd_name = { '> StateExoPosX',...
            'StateExoPosZ',...
            'StateExoRotY',...
            'StateExoThighBarRotY',...
            'StateExoThighModulePosZ',...
            'StateExoThighModuleRotY',...
            'StateExoTorsoBarRotY',...
            'StateExoTorsoModulePosZ',...
            'StateExoTorsoModuleRotY',...
            '> StateBoxPosX',...
            'StateBoxPosZ',...
            'StateBoxRotY',...
            'StatePelvisPosX',...
            'StatePelvisPosZ',...
            'StatePelvisRotY',...
            'StateRightHipRotY',...
            'StateRightKneeRotY',...
            'StateRightAnkleRotY',...
            'StateMiddleTrunkRotY',...
            'StateUpperTrunkRotY',...
            'StateHeadRotY',...
            'StateRightUpperArmRotY',...
            'StateRightLowerArmRotY',...
            'StateRightHandRotY',...
            '> StateExoVelX',...
            'StateExoVelZ',...
            'StateExoRotVelY',...
            'StateExoThighBarRotVelY',...
            'StateExoThighModuleVelZ',...
            'StateExoThighModuleRotVelY',...
            'StateExoTorsoBarRotVelY',...
            'StateExoTorsoModuleVelZ',...
            'StateExoTorsoModuleRotVelY',...
            'StateBoxVelX',...
            'StateBoxVelZ',...
            'StateBoxVelY',...
            'StatePelvisVelX',...
            'StatePelvisVelZ',...
            'StatePelvisRotVelY',...
            'StateRightHipRotVelY',...
            'StateRightKneeRotVelY',...
            'StateRightAnkleRotVelY',...
            'StateMiddleTrunkRotVelY',...
            'StateUpperTrunkRotVelY',...
            'StateHeadRotVelY',...
            'StateRightUpperArmRotVelY',...
            'StateRightLowerArmRotVelY',...
            'StateRightHandRotVelY'};  

if(flag_addActivationDynamics == 1 && nStatesActivation == 18 )
          
  xd_name = {xd_name{:}, ...
              '> StateActRightHipExtensionRotY',...      
              'StateActRightHipFlexionRotY',...        
              'StateActRightKneeExtensionRotY',...     
              'StateActRightKneeFlexionRotY',...       
              'StateActRightAnkleExtensionRotY',...    
              'StateActRightAnkleFlexionRotY',...      
              'StateActMiddleTrunkExtensionRotY',...   
              'StateActMiddleTrunkFlexionRotY',...     
              'StateActUpperTrunkExtensionRotY',...    
              'StateActUpperTrunkFlexionRotY',...      
              'StateActHeadExtensionRotY',...          
              'StateActHeadFlexionRotY',...            
              'StateActRightUpperArmExtensionRotY',... 
              'StateActRightUpperArmFlexionRotY',...   
              'StateActRightLowerArmExtensionRotY',... 
              'StateActRightLowerArmFlexionRotY',...   
              'StateActRightHandExtensionRotY',...     
              'StateActRightHandFlexionRotY'};           
end

if(nStatesTauDotExo == 2)
    xd_name = {xd_name{:}, ...
              '> StateExoThighBarTauRotY',...      
              'StateExoTorsoBarTauRotY'};
end


if(flag_addActivationDynamics == 1 && ...
        (nStatesActivation ~= 18))
    assert(0,['Error: code is not yet configured to handle this number',...
              ' of activation states, only 18 activation states are permitted']);
end

