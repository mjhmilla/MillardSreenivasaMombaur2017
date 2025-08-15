clc;
close all;
clear all;

flag_usingOctave = 0;

outputFigureFolder = 'figures/';
outputTableFolder = 'tables/';


boxMass      = 1;
costFcn      = 'minActSq';
ocpFolder    = [];
ocpFolderExo = ['data/',sprintf('box%dkg',boxMass),'/',costFcn,'Exo_converged/'];

flag_ocpNoExoAvailable = 0;
flag_IdDataAvailable   = 0;
if(boxMass == 15)
    ocpFolder    = ['data/',sprintf('box%dkg',boxMass),'/',costFcn,'_converged/'];
    flag_ocpNoExoAvailable = 1;
    flag_IdDataAvailable   = 1;    
end

if(flag_ocpNoExoAvailable==1)
    [ocpData ocpColNames] = ...
        getFileAndColumnNames(ocpFolder,...
                             'wholebodylifter2d_augmented.txt',...
                             'wholebodylifter2d_augmented_columnNames.txt');
else
    %This is a small hack: I need to have the size of ocpData and
    %ocpColNames for the plotting script to work. The data never 
    %gets plotted, but the script fails without it.
    [ocpData ocpColNames] = ...
        getFileAndColumnNames(['data/',sprintf('box%dkg',15),'/',costFcn,'_converged/'],...
                             'wholebodylifter2d_augmented.txt',...
                             'wholebodylifter2d_augmented_columnNames.txt');
     ocpData =ocpData.*0.;
     
end

[ocpDataExo ocpColNamesExo] = ...
    getFileAndColumnNames(ocpFolderExo,...
                         'wholebodylifter2d_augmented.txt',...
                         'wholebodylifter2d_augmented_columnNames.txt');                     

                   
loadInverseDynamicsData;


if(flag_IdDataAvailable == 1)
    hipAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightHipRotY) ...
                                    -idData.q(:,idxIdLeftHipRotY))).*(180/pi);
    kneeAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightKneeRotY) ...
                                     -idData.q(:,idxIdLeftKneeRotY))).*(180/pi);
    ankleAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightAnkleRotY) ...
                                      -idData.q(:,idxIdLeftAnkleRotY))).*(180/pi);                            

    shoulderAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightUpperArmRotY) ...
                                    -idData.q(:,idxIdLeftUpperArmRotY))).*(180/pi);
    elbowAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightLowerArmRotY) ...
                                     -idData.q(:,idxIdLeftLowerArmRotY))).*(180/pi);
    wristAngleDiffLeftRight = max( abs(idData.q(:,idxIdRightHandRotY) ...
                                      -idData.q(:,idxIdLeftHandRotY))).*(180/pi);                               

    disp('Max. Kinematic errors between left and right');
    disp(sprintf('%e: Hips',hipAngleDiffLeftRight));
    disp(sprintf('%e: Knees',kneeAngleDiffLeftRight));
    disp(sprintf('%e: Ankles',ankleAngleDiffLeftRight));
    disp(sprintf('%e: Shoulders',shoulderAngleDiffLeftRight));
    disp(sprintf('%e: Elbow',elbowAngleDiffLeftRight));
    disp(sprintf('%e: Wrist',wristAngleDiffLeftRight));
end

fig1=figure;
fig2=figure;
fig3=figure;
fig4=figure;
fig5=figure;
fig6=figure;


figList(6) = struct('h',[]);
figList(1).h  = fig1;
figList(2).h  = fig2;
figList(3).h  = fig3;
figList(4).h  = fig4;
figList(5).h  = fig5;
figList(6).h  = fig6;


%%
%Plot config
%%
numberOfHorizontalPlotColumns = 3;
plotConfig;

%idData = [idData.time idData.q idData.tau];

numOfPlots = 6;



dataIndices.x = zeros(numOfPlots,3);
dataIndices.y = zeros(numOfPlots,3);

dataType = zeros(numOfPlots,1);
    typePositions = 0;
    typeVelocities= 1;
    typeTau       = 2;

    
dataType(1:3) = typePositions;
dataType(4:6) = typeTau;

dataScaling = zeros(numOfPlots,3);
dataScaling(1:3,:) = 180/pi;
dataScaling(4:6,:) =1;

dataIndices.x(1,:) = [1 ...
                      getColumnIndex('time', ocpColNames) ...
                      getColumnIndex('time', ocpColNamesExo)];                  

for i=2:1:size(dataIndices.x,1)
    dataIndices.x(i,:) = dataIndices.x(1,:);    
end


%%
%Angles
%%
dataIndices.y(1,:) = [idxIdRightHipRotY,...
        getColumnIndex('StateRightHipRotY', ocpColNames) ...
        getColumnIndex('StateRightHipRotY', ocpColNamesExo)];

dataIndices.y(2,:) = [idxIdRightKneeRotY,...
        getColumnIndex('StateRightKneeRotY', ocpColNames) ...
        getColumnIndex('StateRightKneeRotY', ocpColNamesExo)];

dataIndices.y(3,:) = [idxIdRightAnkleRotY,...
        getColumnIndex('StateRightAnkleRotY', ocpColNames) ...
        getColumnIndex('StateRightAnkleRotY', ocpColNamesExo)];

%%
%Torques
%%
dataIndices.y(4,:) = [idxIdRightHipRotY,...
        getColumnIndex('TauRightHipRotY', ocpColNames) ...
        getColumnIndex('TauRightHipRotY', ocpColNamesExo)];

dataIndices.y(5,:) = [idxIdRightKneeRotY,...
        getColumnIndex('TauRightKneeRotY', ocpColNames) ...
        getColumnIndex('TauRightKneeRotY', ocpColNamesExo)];

dataIndices.y(6,:) = [idxIdRightAnkleRotY,...
        getColumnIndex('TauRightAnkleRotY', ocpColNames) ...
        getColumnIndex('TauRightAnkleRotY', ocpColNamesExo)];

%%
%
%%

    
dataSigns = zeros(6,1);
dataSigns(1) = -1;
dataSigns(2) =  1;
dataSigns(3) = -1;
dataSigns(4) =  1;
dataSigns(5) = -1;
dataSigns(6) =  1;


xLabelText = {'Time (s)','Time (s)','Time (s)',...
              'Time (s)','Time (s)','Time (s)'};
          
yLabelText = {'Angle ($^\circ$)','Angle ($^\circ$)','Angle ($^\circ$)',...
              'Torque (Nm)','Torque (Nm)','Torque (Nm)'};
          
titleText  = {'Hip Flexion','Knee Flexion','Ankle Dorsiflexion',...
              'Hip Extension','Knee Extension','Ankle Plantarflexion'};

addLegend = zeros(numOfPlots,1);
addLegend(1) = 0;
addLegend(4) = 1;

peakTable = zeros(3,6);


%%
%Generate the plots.
%%
for(i=1:1:length(figList))
  figure(figList(i).h);
  subplot('Position',subPlotSquare);
 
    xmin = min(idData.time);
    xmax = max(idData.time);
    width = xmax-xmin;
    
    if(flag_IdDataAvailable==1)
        idDataVector = [];
        switch dataType(i)
            case typePositions
                disp([titleText{i},' Peak Angle']);
                idDataVector = idData.q(:,dataIndices.y(i,1)).*(dataSigns(i)*dataScaling(i,1));
            case typeVelocities
                disp([titleText{i},' Peak Velocities']);
                idDataVector = idData.qDot(:,dataIndices.y(i,1)).*(dataSigns(i)*dataScaling(i,1));
            case typeTau
                disp([titleText{i},' Peak Torques']);
                idDataVector = idData.tau(:,dataIndices.y(i,1)).*(dataSigns(i)*dataScaling(i,1));  
                if(dataIndices.y(i,1) == idxIdRightHipRotY)
                    idDataVector = idDataVector ...
                        + idData.tau(:,idxIdLeftHipRotY).*(dataSigns(i)*dataScaling(i,1)); 
                end
                if(dataIndices.y(i,1) == idxIdRightKneeRotY)
                    idDataVector = idDataVector ...
                        + idData.tau(:,idxIdLeftKneeRotY).*(dataSigns(i)*dataScaling(i,1));                 
                end
                if(dataIndices.y(i,1) == idxIdRightAnkleRotY)
                    idDataVector = idDataVector ...
                        + idData.tau(:,idxIdLeftAnkleRotY).*(dataSigns(i)*dataScaling(i,1)); 

                end
            otherwise 
                assert(0,'Not a defined type');
        end

        lineID = ...
            plot( idData.time, ...
              idDataVector,...
              lineType{idxLineID},...
              'Color',lineColor(idxLineID,:),...
              'LineWidth', lineWidth(idxLineID));
        hold on;    



        [val idx] = max(abs(idDataVector));
        peakTable(1,i) = val*sign(idDataVector(idx));
        disp(sprintf('%e: ID', peakTable(1,i)));
    end
    
    
    if(flag_ocpNoExoAvailable==1)
        lineMinActSq = ...
            plot( ocpData(:,dataIndices.x(i,2)), ...
              ocpData(:,dataIndices.y(i,2)).*(dataSigns(i)*dataScaling(i,2)),...
              lineType{idxLineMinActSq},...
              'Color',lineColor(idxLineMinActSq,:),...
              'LineWidth', lineWidth(idxLineMinActSq));
        hold on;    
    
        dataVec = max(ocpData(:,dataIndices.y(i,2)).*(dataSigns(i)*dataScaling(i,2)));
        
        [val idx] = max(abs( dataVec ));
        peakTable(2,i) = val*sign(dataVec(idx));
        disp(sprintf('%e: min act sq', peakTable(2,i)));
    end
    
    lineMinActSqExo = ...
        plot( ocpDataExo(:,dataIndices.x(i,3)), ...
          ocpDataExo(:,dataIndices.y(i,3)).*(dataSigns(i)*dataScaling(i,3)),...
          lineType{idxLineMinActSqExo},...
          'Color',lineColor(idxLineMinActSqExo,:),...
          'LineWidth', lineWidth(idxLineMinActSqExo));
    hold on;
        
    dataVec = max( ocpDataExo(:,dataIndices.y(i,3)).*(dataSigns(i)*dataScaling(i,3)) );
    [val idx] = max(abs( dataVec ));
    peakTable(3,i) = val*sign(dataVec(idx));
        
    disp(sprintf('%e: min act sq (exo)', peakTable(3,i)));
    
    if(addLegend(i) == 1)
            
        hL = [];
        hO = [];

        if(flag_IdDataAvailable==1 && flag_ocpNoExoAvailable==1)
            if(i ==1)
                [hL hO] = legend([lineID,lineMinActSq,lineMinActSqExo],...
                   'Exp. (IK/ID)', 'human-only OCP','with-exo OCP','Location','none');
            else
                [hL hO] = legend([lineID,lineMinActSq,lineMinActSqExo],...
                   'Exp . (IK/ID)', 'human-only OCP','with-exo OCP','Location','none');
            end
        end

        
        h = subPlotSquare(1,4);
        w = subPlotSquare(1,3);
        l = subPlotSquare(1,1);
        b = subPlotSquare(1,2);

        set(hL,'Position', [(l+w*0.325) (b+0.15*h) (w*0.2) (h*0.01)] );     

        for z=1:1:length(hO)                
            if(isempty(strfind(class(hO(z)),'Line'))==0)
              tmp = get(hO(z),'XData');
              if(length(tmp)==2)
               set(hO(z),'XData', [(tmp(2)-(tmp(2)-tmp(1))*0.45),tmp(2)]); 
              end
            end
        end

        legend boxoff;
    end
    xlabel(xLabelText{i});
    ylabel(yLabelText{i});
    title(titleText{i});
    
    ax = gca;
    ax.XTick = xTickTime;
    
    axis square;
    box off;
    
end

csvwrite([outputTableFolder,'peakHipKneeAnkleAngleAndTorque.csv'],peakTable);

%%
%Write the plots
%%


figNameList = {['fig_',costFcn,'_HipFlexionAngle.pdf'],... 
               ['fig_',costFcn,'_KneeFlexionAngle.pdf'],...
               ['fig_',costFcn,'_AnkleFlexionAngle.pdf'],...
               ['fig_',costFcn,'_HipExtensionTorque.pdf'],... 
               ['fig_',costFcn,'_KneeExtensionTorque.pdf'],...
               ['fig_',costFcn,'_AnkleExtensionTorque.pdf']}   ;     

figOutput = [1:1:length(figList)];
figOutput = [figOutput,1]; %The first export is usually sized oddly. Matlab bug?

for i=figOutput
    figure(figList(i).h);
    set(figList(i).h,'Units','centimeters',...
         'PaperUnits','centimeters',...
         'PaperSize',[pageWidth pageHeight],...
         'PaperPositionMode','manual',...
         'PaperPosition',[0 0 pageWidth pageHeight]);       
         %set(findall(figList(i).h,'-property','FontSize'),'FontSize',10);       
         set(figList(i).h,'renderer','painters');       
         print('-dpdf', [outputFigureFolder, sprintf('box%dkg',boxMass),'/' ,figNameList{i}]);  
end
