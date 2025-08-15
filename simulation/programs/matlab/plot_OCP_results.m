clear; 
%clf; 
clc;
close all;



bPlot_q    = 1;
bPlot_qvel = 1;
bPlot_u    = 0;
bPlot_tau = 1;
bPlot_muscleTorques = 0;
bPlot_muscleActivations = 0;
bPlot_footContactInfo = 0;
bPlot_handContactInfo = 0;

flag_addActivationDynamics = 0;
nStatesActivation = 18;

timeOfInterest = 0.1;


ocpPath = '../../OCP/';

[ocpData ocpColNames] = ...
    getFileAndColumnNames([ocpPath,'build/RES/'],...
                         'wholebodylifter2d_augmented.txt',...
                         'wholebodylifter2d_augmented_columnNames.txt');

idxStage = getColumnIndex('Phase', ocpColNames);
stage_change_idx = find(diff(ocpData(:,idxStage)) > 0);
                     
                     
stoopIdx     = 2;
%%Experimental data
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

qIdxBox            = [getColumnIndex('StateBoxPosX', ocpColNames),...
                      getColumnIndex('StateBoxPosZ', ocpColNames),...
                      getColumnIndex('StateBoxRotY', ocpColNames)];                  
qIdxBoxFreeFlyer   = qIdxBox;

qIdxHuman          = [getColumnIndex('StatePelvisPosX', ocpColNames):1:...
                      getColumnIndex('StateRightHandRotY',ocpColNames)];                  
qIdxHumanFreeFlyer = [getColumnIndex('StatePelvisPosX', ocpColNames),...
                      getColumnIndex('StatePelvisPosZ', ocpColNames),...
                      getColumnIndex('StatePelvisRotY', ocpColNames)]; 

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
   

                         
assert(length(qIdxBox)   ==  3,'qIdxBox has incorrect length');
assert(length(qIdxHuman) == 12,'qIdxHuman has incorrect length');


NoOfPositions           = 15;
NoOfStates              = 30;
NoOfControls            = 18;
NoOfGenForces           = NoOfPositions;
NoOfMusclesActiveTq     = NoOfControls;
NoOfMusclesPassiveTq    = NoOfControls;
NoOfActivationStates    = 0;
NoOfFootEntries         = 6;
NoOfHandEntries         = 3;

configureAugmentedFileIndices;

qExpIdxBox            = [1,2,3];
qExpIdxBoxFreeFlyer   = qExpIdxBox;
qExpIdxHuman          = [4,5,6, 7,8,9, 13,14,15, 16,17,18];
qExpIdxHumanFreeFlyer = [4,5,6]; 
colMapExp2Model    = [qExpIdxBox,qExpIdxHuman];

getSdMinSdMaxSdScaSdLabels;

if bPlot_q
    fig_q = figure;           
    csv_dat = ocpData(:,[timeIdx,qIdx]);%dlmread([ocp_path,'build/RES/wholebodylifter2d.csv'],',',0,0);
    csv_orig = dlmread([ocpPath,'DAT/initialStates.csv'],',');
    q = csv_dat(1:end,2:end);
    q_orig = csv_orig(1:end,2:end);
    timeStamp = csv_dat(1:end,1);
    timeStamp_orig = csv_orig(1:end,1);
    dim_q = NoOfPositions;
    assert(dim_q == length(qIdx));

    labels_q = {...
        'StateBoxPosX',...
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
        'StateRightHandRotY'};

    for i = 1:dim_q
        subplot(6,4,i); hold on;
        plot(timeStamp, q(:,i), '-k');
        plot(timeStamp_orig, q_orig(:,i), '--b');
        plot([timeStamp(stage_change_idx) timeStamp(stage_change_idx)]', repmat([-2.5 2.5], length(stage_change_idx),1)', '--k');
        plot([0 timeStamp(end)],[sd_min(i) sd_min(i)],'--r');
        plot([0 timeStamp(end)],[sd_max(i) sd_max(i)],'--r');
        xlim([0 timeStamp(end)]); ylim([sd_min(i) sd_max(i)]);
        title(labels_q(i),'fontsize',10);
    end
    
    idxOfInterestVector = find(timeStamp >= timeOfInterest);
    idxOfInterest = idxOfInterestVector(1);
    
    disp(['q at ', num2str(timeOfInterest)]);
    
    
    for k=1:1:length(labels_q)
        disp(sprintf('%s\t\t%e',labels_q{k}, q(idxOfInterest,k)));
    end
    
end

if bPlot_qvel
    fig_qvel = figure;
    qvel = ocpData(:,dqIdx);
    timeStamp = ocpData(:,timeIdx);
    dim_qvel = NoOfPositions;
    assert(dim_qvel == length(dqIdx));
        
    labels_qvel = {...
        'StateBoxVelX',...
        'StateBoxVelZ',...
        'StateBoxRotVelY',...
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

    wc3db = ones(size(labels_qvel)).*8;
    wc3db(13) = 2.5;
    wc3db(14) = 2.5;
    t0  = timeStamp(1);
    t1  = timeStamp(end);
    n01 = 200;
    timeStampEven = [t0:((t1-t0)/(n01-1)):t1]';
    freq = n01/(t1-t0);
        
    tmp = diff(timeStamp);
    idxDoubled = find(tmp == 0);
    idxMonotonic = [1:1:length(timeStamp)];
    idxMonotonic(idxDoubled) = -1;
    idxMontonic = find(idxMonotonic > 0);
    
    for i = 1:dim_qvel
        subplot(6,4,i); hold on;
        plot(timeStamp, qvel(:,i), '-k');
        hold on;
        
        
        
        qvelEven = interp1(timeStamp(idxMontonic),qvel(idxMontonic,i),timeStampEven);
        [b,a] = butter(2, wc3db(i)/(0.5*freq),'low');
        qvelFilt = filtfilt(b,a,qvelEven);
        
        plot(timeStampEven, qvelFilt,'-r');
        hold on;
        
        plot([timeStamp(stage_change_idx) timeStamp(stage_change_idx)]', repmat([-20 20], length(stage_change_idx),1)', '--k');
        hold on;
        if(abs(min(qvel(:,i))-max(qvel(:,i))) > 1e-6)
            ylim([min(qvel(:,i)) max(qvel(:,i))])
        end
        xlim([0 timeStamp(end)])
        title(labels_qvel(i),'fontsize',10);
    end
end

if bPlot_tau
    fig_tau = figure;           
    csv_dat = ocpData(:,[timeIdx,tauIdx]);%dlmread([ocp_path,'build/RES/wholebodylifter2d.csv'],',',0,0);
    dim_tau = NoOfPositions;
    
    csv_orig = dlmread([ocpPath,'DAT/initialStates.csv'],',');
    q = csv_dat(1:end,2:end);
    q_orig = csv_orig(1:end,2:end);
    timeStamp = csv_dat(1:end,1);
    timeStamp_orig = csv_orig(1:end,1);
    


    labels_tau = {...
        'TauBoxPosX',...
        'TauBoxPosZ',...
        'TauBoxRotY',...
        'TauPelvisPosX',...
        'TauPelvisPosZ',...
        'TauPelvisRotY',...
        'TauRightHipRotY',...
        'TauRightKneeRotY',...
        'TauRightAnkleRotY',...
        'TauMiddleTrunkRotY',...
        'TauUpperTrunkRotY',...
        'TauHeadRotY',...
        'TauRightUpperArmRotY',...
        'TauRightLowerArmRotY',...
        'TauRightHandRotY'};

    for i = 1:dim_tau
        subplot(6,4,i); hold on;
        plot(csv_dat(:,1), csv_dat(:,1+i), '-k');
        title(labels_tau(i),'fontsize',10);
    end
    
    
end


if bPlot_u
    fig_u = figure;
    assert(NoOfControls > 0,'No controls set in this model');
    controls = ocpData(:,uIdx);
    timeStamp = ocpData(:,1);
    dim_controls = NoOfControls;
    labels_u = {...
     'ControlRightHipExtension',...
     'ControlRightHipFlexion',...
     'ControlRightKneeExtension',...
     'ControlRightKneeFlexion',...
     'ControlRightAnkleExtension',...
     'ControlRightAnkleFlexion',...
     'ControlMiddleTrunkExtension',...
     'ControlMiddleTrunkFlexion',...
     'ControlUpperTrunkExtension',...
     'ControlUpperTrunkFlexion',...
     'ControlHeadExtension',...
     'ControlHeadFlexion',...
     'ControlRightUpperArmExtension',...
     'ControlRightUpperArmFlexion',...
     'ControlRightLowerArmExtension',...
     'ControlRightLowerArmFlexion',...
     'ControlRightHandExtension',...
     'ControlRightHandFlexion'};

     assert(dim_controls == size(controls,2));

    for i = 1:dim_controls
        subplot(5,6,i); hold on;
        plot(timeStamp, controls(:,i), '-r');
        plot([timeStamp(stage_change_idx) timeStamp(stage_change_idx)]', repmat([-250 250], length(stage_change_idx),1)', '--k');
        plot([0 timeStamp(end)], [0 0], '--k');
        plot([0 timeStamp(end)], [1 1], '--k');
        xlim([0 timeStamp(end)])
        ylim([0 1.1])
        title(labels_u(i),'fontsize',10);
    end
end

if bPlot_muscleTorques
    fig_mcl = figure;
    muscleTorques_active  = ocpData(:,mclActIdx);
    muscleTorques_passive = ocpData(:,mclPasIdx);
    muscleTorques_total   = muscleTorques_active + muscleTorques_passive;
    timeStamp = ocpData(:,1);
    dim_muscleTorques = NoOfControls;
    labels_muscleTorques = {...
        'RightHipExtension','RightHipFlexion', ...
        'RightKneeExtension','RightKneeFlexion',...
        'RightAnkleExtension','RightAnkleFlexion',...
        'MiddleTrunkExtension','MiddleTrunkFlexion',...
        'UpperTrunkExtension','UpperTrunkFlexion',...
        'HeadExtension','HeadFlexion',...
        'RightUpperArmExtension','RightUpperArmFlexion',...
        'RightLowerArmExtension','RightLowerArmFlexion',...
        'RightHandExtension','RightHandFlexion'};
    for i = 1:dim_muscleTorques
        subplot(5,6,i); hold on;
        plot(timeStamp, muscleTorques_total(:,i), '-k','linewidth',1);
        plot(timeStamp, muscleTorques_active(:,i), '-r');
        plot(timeStamp, muscleTorques_passive(:,i), '-g');        
        plot([timeStamp(stage_change_idx) timeStamp(stage_change_idx)]', repmat([-250 250], length(stage_change_idx),1)', '--k');
        % ylim([min(muscleTorques_total(:,i)) max(muscleTorques_total(:,i))]);
        title(labels_muscleTorques(i),'fontsize',10);
        xlim([0 timeStamp(end)])
    end
    
    idxOfInterestVector = find(timeStamp >= timeOfInterest);
    idxOfInterest = idxOfInterestVector(1);
    
    disp(['Total Muscle Torques at ', num2str(timeOfInterest)]);
    
    
    for k=1:1:length(labels_muscleTorques)
        disp(sprintf('%s\t\t%e',labels_muscleTorques{k}, muscleTorques_total(idxOfInterest,k)));
    end
    
end

if bPlot_footContactInfo
    fig_foot = figure;    
    stage_change_name = {'Bend', 'Wait', 'Lift'};
    stage_change_time = [ocpData(stage_change_idx,1);ocpData(end,1)];
    
    footForce = ocpData(:,footIdx);
    
    rHeelForce  = footForce(:,1:3);
    rToeForce   = footForce(:,4:6);
    lHeelForce  = footForce(:,7:9);
    lToeForce   = footForce(:,10:12);
    timeStamp   = ocpData(:,timeIdx);
    
    patchColor = [0.6 0.8 0.6; 0.6 0.6 0.8; 0.8 0.6 0.6];
    
    subplot(1,2,1); hold on; axis square;
    plot(timeStamp(:), lHeelForce(:,3), '-r');
    plot(timeStamp(:), lToeForce( :,3), '--r');
    plot(timeStamp(:), lHeelForce(:,3)  + lToeForce(:,3), '-r', 'linewidth', 2);
    %plot(timeStamp, lHeelForce(:,3), '-r');
    %plot(timeStamp, lToeForce(:,3), '--r');
    %plot(timeStamp, lHeelForce(:,3)+lToeForce(:,3), '-r', 'linewidth', 2);
    legend('lHeel','lHalx', 'L GRF');
    plot([0 timeStamp(end)], [0 0], '--k');
    xlim([0 timeStamp(end)]);
    plot([stage_change_time stage_change_time]', [-100 800], '--k');
    text_y_height = [850 850 850];
    for i = 1:length(stage_change_time)
        text(stage_change_time(i)-1.0, text_y_height(i), stage_change_name{i});
    end
    ylim([-15 800]);
    title('Left Foot Contact Forces');
    
    subplot(1,2,2); hold on; axis square;
    plot(timeStamp(:), rHeelForce(:,3), '-b');
    plot(timeStamp(:), rToeForce( :,3), '--b');
    plot(timeStamp(:), rHeelForce(:,3)+ rToeForce(:,3), '-b', 'linewidth', 2);
    %plot(timeStamp, rHeelForce(:,3), '-b');
    %plot(timeStamp, rToeForce(:,3), '--b');
    %plot(timeStamp, rHeelForce(:,3)+rToeForce(:,3), '-b', 'linewidth', 2);
    legend('rHeel','rHalx', 'R GRF');
    plot([0 timeStamp(end)], [0 0], '--k');
    xlim([0 timeStamp(end)]);
    plot([stage_change_time stage_change_time]', [-100 800], '--k');
    text_y_height = [850 850 850];
    for i = 1:length(stage_change_time)
        text(stage_change_time(i)-1.0, text_y_height(i), stage_change_name{i});
    end
    ylim([-15 800]);
    title('Right Foot Contact Forces');
end

if bPlot_handContactInfo
    fig_hand = figure;
    timeStamp = ocpData(:,timeIdx);
    handPos  = ocpData(:,handIdx);
    rHandPos = handPos(:,1:3);
    lHandPos = handPos(:,4:6);

    hold on;
    plot(timeStamp, rHandPos(:,3), '-r');
    plot(timeStamp, lHandPos(:,3), '-b');
    plot([timeStamp(stage_change_idx) timeStamp(stage_change_idx)]', repmat([-250 250], length(stage_change_idx),1)', '--k');
    legend('rHand','lHand');
    plot([0 timeStamp(end)], [0 0], '--k');
    ylim([0 1.0]);
    xlim([0 timeStamp(end)]);
end
