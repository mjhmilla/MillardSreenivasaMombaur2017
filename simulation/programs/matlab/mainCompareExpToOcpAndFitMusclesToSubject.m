%clc;
close all;
clear all;

datFolderID      = '../../OCP/PRE/ID/2D/';
datFolderVUA2UHEI= '../../OCP/PRE/VUA2UHEI/2D/';

flag_plotKinematics = 0;
flag_plotResiduals = 0;
flag_plotTau  = 1;
flag_plotGRF  = 0;
flag_plotL5S1 = 0;
flag_plotMuscleScaling = 0;
flag_printActivationDerivativeLimits = 0;

numQ = 21;
numM = 30;

qCols2Plot = [1:numQ];
residualForceIndex = [4:6];

mCols2Plot = [1,7,13,15];

stoopIdx =2;

data       = csvread([datFolderID,sprintf('stoop%d.csv',stoopIdx)]);

timeV      = csvread([datFolderID,sprintf('time_stoop%d.csv',stoopIdx)]);

qExp       = csvread([datFolderID,sprintf('q_stoop%d.csv',stoopIdx)]);
qDotExp    = csvread([datFolderID,sprintf('qDot_stoop%d.csv',stoopIdx)]);
qDotDotExp = csvread([datFolderID,sprintf('qDotDot_stoop%d.csv',stoopIdx)]);
tauExp     = csvread([datFolderID,sprintf('tau_stoop%d.csv',stoopIdx)]);

actExp       = csvread([datFolderID,sprintf('actAnalysis_act_stoop%d.csv',stoopIdx)]);
actTaExp     = csvread([datFolderID,sprintf('actAnalysis_ta_stoop%d.csv',stoopIdx)]);
actTvExp     = csvread([datFolderID,sprintf('actAnalysis_tv_stoop%d.csv',stoopIdx)]);
actTpExp     = csvread([datFolderID,sprintf('actAnalysis_tp_stoop%d.csv',stoopIdx)]);


mclSfExp     = csvread([datFolderID,sprintf('muscleStrengthScaleFactor_stoop%d.csv',stoopIdx)]);

vuaTauL5S1Bu = csvread([datFolderVUA2UHEI,sprintf('stoop%d_l5s1Bu.csv',stoopIdx)]);
vuaTauL5S1Td = csvread([datFolderVUA2UHEI,sprintf('stoop%d_l5s1Td.csv',stoopIdx)]);

grfExp     = csvread([datFolderVUA2UHEI, sprintf('stoop%d_F1Cop1F2Cop2_2D.csv',stoopIdx)]);


fileResidualReport = [datFolderID,sprintf('residual_stats_stoop%d.txt',stoopIdx)];

phaseTimes = [];
switch stoopIdx
    case 1
        phaseTimes = [0, 1.9; 2.42,3.6];
    case 2
        phaseTimes = [0 1.98; 2.4 3.9];
    case 3
        phaseTimes = [0 2.12; 2.4 3.94];

    otherwise
        phaseTimes = zeros(1,2);
end        


phaseIdx   = zeros(size(phaseTimes));
deltaT     = timeV(2)-timeV(1);

for i=1:1:size(phaseTimes,1)
   for j=1:1:length(timeV)
       for k=1:1:2
           if(abs(phaseTimes(i,k)-timeV(j))<= deltaT)
              phaseIdx(i,k) = j; 
           end
       end       
   end    
end




names = {...
'BoxPosX',...
'BoxPosZ',...
'BoxRotY',...
'PelvisPosX',...
'PelvisPosZ',...
'PelvisRotY',...
'RightHipRotY',...
'RightKneeRotY',...
'RightAnkleRotY',...
'LeftHipRotY',...
'LeftKneeRotY',...
'LeftAnkleRotY',...
'MiddleTrunkRotY',...
'UpperTrunkRotY',...
'HeadRotY',...
'RightUpperArmRotY',...
'RightLowerArmRotY',...
'RightHandRotY',...
'LeftUpperArmRotY',...
'LeftLowerArmRotY',...
'LeftHandRotY'};

muscleNames = {
'RightHipExtensionRotY',...
'RightHipFlexionRotY',...
'RightKneeExtensionRotY',...
'RightKneeFlexionRotY',...
'RightAnkleExtensionRotY',...
'RightAnkleFlexionRotY',...
'LeftHipExtensionRotY',...
'LeftHipFlexionRotY',...
'LeftKneeExtensionRotY',...
'LeftKneeFlexionRotY',...
'LeftAnkleExtensionRotY',...
'LeftAnkleFlexionRotY',...
'MiddleTrunkExtensionRotY',...
'MiddleTrunkFlexionRotY',...
'UpperTrunkExtensionRotY',...
'UpperTrunkFlexionRotY',...
'HeadExtensionRotY',...
'HeadFlexionRotY',...
'RightUpperArmExtensionRot',...
'RightUpperArmFlexionRotY',...
'RightLowerArmExtensionRot',...
'RightLowerArmFlexionRotY',...
'RightHandExtensionRotY',...
'RightHandFlexionRotY',...
'LeftUpperArmExtensionRotY',...
'LeftUpperArmFlexionRotY',...
'LeftLowerArmExtensionRotY',...
'LeftLowerArmFlexionRotY',...
'LeftHandExtensionRotY',...
'LeftHandFlexionRotY'};

grfNames = {'Time','grfXBox','grfYBox','grfZBox', ...
                   'copXBox','copYBox','copZBox', ...
                   'grfX','grfY','grfZ', ...
                   'copX','copY','copZ'};

grfUnits = {'s','N','N','N', ...
                'm','m','m', ...
                'N','N','N', ...
                'm','m','m'};               
               
if(flag_plotMuscleScaling == 1)
   
    fig(size(actExp,2)/2) = struct('h',[]);
    
    if(flag_printActivationDerivativeLimits==1)
       disp(sprintf('max(|d/dt act|)\tmin(|actTau|)\ti\tName')); 
    end
    
    for i=mCols2Plot
       fig(k).h = figure;       
       subplot(2,2,1);
        plot(timeV, actExp(:,i),'k','LineWidth',2);
        hold on;
        plot(timeV, actTaExp(:,i),'r','LineWidth',1);
        hold on;
        plot(timeV, actTvExp(:,i),'m','LineWidth',1);
        hold on;
        plot(timeV, actTpExp(:,i),'b','LineWidth',1);
        hold on;
        ylim([-0.1, max(1.5, max(actExp(:,i)))]);        
        xlabel('Time');
        ylabel('Unitless');
        title([muscleNames{i},': Activation Analysis']); 
        legend('Act.','Ta','Tv','Tp');


       subplot(2,2,2);
        plot(timeV, mclSfExp(:,i),'k','LineWidth',2);
        hold on;
        ylim([-0.1, max(1.5, max(mclSfExp(:,i)))]);
        xlabel('Time');
        ylabel('Nm/Nm');        
        title([muscleNames{i} ,': TauMax scaling']);
       
       dact = calcCentralDifference(timeV, actExp(:,i));
       actTau = (ones(size(actExp(:,i)))-actExp(:,i))./dact;
       subplot(2,2,3);
        plot(timeV,dact,'k','LineWidth',2);
        xlabel('Time');
        ylabel('d/dt activation');        
        title([muscleNames{i} ,': d/dt Act.']);
        
        
        
        outText = sprintf('%f\t%f\t%i\t%s',...
                        max(abs(dact)),min(abs(actTau)),i,muscleNames{i});
        disp(outText);
        
        here=1;
    end
    
end
            
if(flag_plotGRF == 1)
   figFP(2) = struct('h',[]);
   for i=1:1:2
       figFP(1).h = figure;
       for j=1:1:6
           subplot(2,3,j);
           plot(timeV,grfExp(:,1+j+(i-1)*6),'b');          
           xlabel(grfNames{1});
           ylabel(grfUnits{1+j+(i-1)*6});
           title(grfNames{1+j+(i-1)*6});
       end
   end
    
end

if(flag_plotL5S1 ==1)
   fig = figure;
   idxL5S1= 13;
   plot(timeV,tauExp(:,idxL5S1),'b');
   hold on;
   plot(vuaTauL5S1Bu(:,1)-vuaTauL5S1Bu(1,1),-1.*vuaTauL5S1Bu(:,3),'r');
   hold on;
   plot(vuaTauL5S1Td(:,1)-vuaTauL5S1Td(1,1),vuaTauL5S1Td(:,3),'--r');
   hold on;
   
   hold on;
   plot([timeV(phaseIdx(1,2)),timeV(phaseIdx(1,2))],...
     [-250,50],'--k');
   hold on;
   plot([timeV(phaseIdx(2,1)),timeV(phaseIdx(2,1))],...
     [-250,50],'--k');
   text( timeV(floor(mean(phaseIdx(1,:)))), -100, 'No Box');
    hold on
   text( timeV(floor(mean(phaseIdx(2,:)))),-100, 'Lifting 15kg box');  
    hold on
   text( timeV(phaseIdx(1,2)),-210, 'Transition'); 
   
   xlabel('Time');
   ylabel('Nm');
   %title([ sprintf('stoop%d',stoopIdx),':', names{idxL5S1}]);
   title('L5/S1 Net Moment');
   
   legend('UHEI:ID','VUA:ID (bottom up)','VUA:ID (top down)');
   saveas(fig,[datFolderID,sprintf('fig_L5S1_UHEI_vs_VUA_stoop%i.pdf',stoopIdx)],'pdf');
end
               
if(flag_plotKinematics == 1)
    idx = 1;
    for i = qCols2Plot
       figH = figure;
       subplot(1,3,1);    
        plot(timeV,qExp(:,i));
        hold on;
        %if(i > 3)
            plot(data(:,1), data(:,1+i),'--r');
            idx = idx+1;
        %end

        ylabel('q');
        title(names{i});


       subplot(1,3,2);   
        plot(timeV,qDotExp(:,i));
        hold on;
        qDotFD = diff(qExp(:,i));
        timeFD = diff(timeV);
        qDotFD = qDotFD./timeFD;
        qDotFD = [0;qDotFD];
        plot(timeV, qDotFD,'--r');
        hold on;
        ylabel('dq/dt');


       subplot(1,3,3);   
        plot(timeV,qDotDotExp(:,i));
        hold on;
        qDotDotFD = diff(qDotExp(:,i));
        qDotDotFD = qDotDotFD./timeFD;
        qDotDotFD = [0;qDotDotFD];
        plot(timeV, qDotDotFD,'--r');
        hold on;

        ylabel('d2q/dt2');
        xlabel('time');
        here=1;
    end
end

if(flag_plotTau == 1)
    for i = qCols2Plot
       figH = figure;
        plot(timeV,tauExp(:,i));
        hold on;
        xlabel('Time');
        ylabel('tau');
        title(names{i});
    end
    

end

if(flag_plotResiduals == 1)
    figH = figure;
    j = 1;
    for i = residualForceIndex
        subplot(2,2,j)
        plot(timeV,tauExp(:,i));
        hold on;
        
        hold on;
        plot([timeV(phaseIdx(1,2)),timeV(phaseIdx(1,2))],...
             [-30,30],'--k');
        hold on;
        plot([timeV(phaseIdx(2,1)),timeV(phaseIdx(2,1))],...
             [-30,30],'--k');
        hold on
        text( timeV(1), 18, 'No box');
        hold on
        text( timeV(phaseIdx(2,1)),18, 'Lifting 15kg box');
        hold on
        text( timeV(phaseIdx(1,2)),-8, 'Transition');
        hold on
        
        xlabel('Time');
        ylabel('Gen. Force');
        title(['ID Residual Forces: ',names{i}]);
        ylim([-20 20]);
        j = j+1;
    end
    
   saveas(figH, [datFolderID, ...
        sprintf('fig_ID_Residuals_stoop%i.pdf',stoopIdx)],'pdf');
    
    %Compute statistics on the residual forces during the two
    %phases and print this to screen
    fid = fopen(fileResidualReport,'w');
    for i=1:1:size(phaseIdx,1)
       line = sprintf('Phase %d',i);
       fprintf(fid,'%s\n',line);
       disp(line);
       line = sprintf('Mean\tStd\tMax\tName');
       fprintf(fid,'%s\n',line);
       disp(line);
              
       for j=1:1:length(residualForceIndex)
          idxRange = [phaseIdx(i,1):1:phaseIdx(i,2)];
          idxForce = residualForceIndex(j);
          resf = tauExp(idxRange, idxForce);           
          
          meanResF = mean(abs(resf));
          stdResF  = std(abs(resf));
          maxResF  = max(abs(resf));
          
          line = sprintf('%.1f\t%.1f\t%.1f\t%s', ...
                       meanResF,stdResF,maxResF,names{idxForce});
          fprintf(fid,'%s\n',line);
          disp(line);
       end        
    end    
    fclose(fid);  
    
    disp('Overall fit');
    line = sprintf('Mean\tStd\tMax\tName');
    overall = zeros(size(phaseIdx,1),3);
    for j=1:1:length(residualForceIndex)
        idxRange = [min(min(phaseIdx)):1:max(max(phaseIdx))];
        idxForce = residualForceIndex(j);
        resf = tauExp(idxRange, idxForce);           

        meanResF = mean(abs(resf));
        stdResF  = std(abs(resf));
        maxResF  = max(abs(resf));

        line = sprintf('%.1f\t%.1f\t%.1f\t%s', ...
                   meanResF,stdResF,maxResF,names{idxForce});
        disp(line);
        
        overall(j,1) = meanResF;
        overall(j,2) = stdResF;
        overall(j,3) = maxResF;
    end
    line = sprintf('%.1f\t%.1f\t%.1f\t%s', ...
                   mean(overall(:,1)),...
                   mean(overall(:,2)),...
                   max(overall(:,3)),'ALL');
        disp(line);
end