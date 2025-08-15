clc;
close all;
clear all;

flag_usingOctave = 0;
outputFigureFolder = 'figures/';
outputTableFolder = 'tables/';

boxMass      = 1; %Only 1 and 15 work!
costFcn      = 'minActSq';
ocpFolderExo = ['data/',sprintf('box%dkg',boxMass),'/',costFcn,'Exo_converged/'];

[ocpDataExo ocpColNamesExo] = ...
    getFileAndColumnNames(ocpFolderExo,...
                         'wholebodylifter2d_augmented.txt',...
                         'wholebodylifter2d_augmented_columnNames.txt');                     
                    
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
numberOfHorizontalPlotColumns = 4;
plotConfig;

timeCol             = getColumnIndex('time', ocpColNamesExo);
tauHipExoCol        = getColumnIndex('TauPosExoThighBarRotY', ocpColNamesExo);
tauLumbarExoCol     = getColumnIndex('TauPosExoTorsoBarRotY', ocpColNamesExo);
angleHipExoCol      = getColumnIndex('StateStateExoThighBarRotY', ocpColNamesExo);
angleLumbarExoCol   = getColumnIndex('StateStateExoTorsoBarRotY', ocpColNamesExo);
angularVelHipExoCol      = getColumnIndex('StateStateExoThighBarRotVelY', ocpColNamesExo);
angularVelLumbarExoCol   = getColumnIndex('StateStateExoTorsoBarRotVelY', ocpColNamesExo);

tauPassiveHipExt   = getColumnIndex('PassiveMuscleRightHipExtensionRotY',ocpColNamesExo);
tauActiveHipExt   = getColumnIndex('ActiveMuscleRightHipExtensionRotY',ocpColNamesExo);


lineType  =  {'-','-'};
lineColor =  [191 191 191;...
              0   0   0]./255;     
lineWidth = [1,1];

idxHip = 1;
idxLumbar =2;

xmin =min(ocpDataExo(:,timeCol));
xmax =max(ocpDataExo(:,timeCol));

peakData = zeros(2,4);


figure(fig1);
    subplot('Position',subPlotSquare);
        lineHipExo = ...
            plot(ocpDataExo(:,timeCol), ocpDataExo(:,tauHipExoCol),...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        [mxval idx] = max(abs(ocpDataExo(:,tauHipExoCol)));
        peakData(1,2) = mxval*sign(ocpDataExo(idx,tauHipExoCol));
        
        [val idx] = min(ocpDataExo(:,tauHipExoCol));
        
        plot(ocpDataExo(idx,timeCol),...
             ocpDataExo(idx,tauHipExoCol),'*',...
             'Color','k',...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        
        lineLumbarExo = ...
            plot(ocpDataExo(:,timeCol), ocpDataExo(:,tauLumbarExoCol).*-1,...
             lineType{idxLumbar},...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));
        hold on;
        
        peakData(2,2) = max(ocpDataExo(:,tauLumbarExoCol).*-1);

        [val idx] = max(abs(ocpDataExo(:,tauLumbarExoCol)));
        peakData(2,2) = val*sign(ocpDataExo(idx,tauLumbarExoCol));
                
        xlabel('Time (s)');
        ylabel('Torque (Nm)');
        title('Actuator Extension Torque');        
        axis tight;        
        axis square;
        box off;
        xlim([xmin,xmax]);
        
        ax = gca;
        ax.XTick = xTickTime;
        
        %legend([lineHipExo,lineLumbarExo],'Hip Actuator','Lumbar Actuator',...
        %        'Location','NorthWest');
        %legend boxoff;
    
figure(fig2);
    subplot('Position',subPlotSquare);
        lineHipExo = ...
            plot(ocpDataExo(:,angleHipExoCol).*(-1*180/pi),...
                 ocpDataExo(:,tauHipExoCol),...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
            plot(ocpDataExo(1,angleHipExoCol).*(-1*180/pi),...
                 ocpDataExo(1,tauHipExoCol),...
              'd',...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        xH = [ones(size(ocpDataExo(:,angleHipExoCol).*(-1*180/pi))),...
             ocpDataExo(:,angleHipExoCol).*(-1*180/pi)];
        yH = ocpDataExo(:,tauHipExoCol);
        bH = xH\yH;
        yHUpd = bH(1).*xH(:,1) + bH(2).*xH(:,2);
        
        disp(sprintf('X-Intercept %e \tSlope %e \tStiffness(Nm/rad) %e: Hip-Char.',...
                      bH(1),bH(2),bH(2).*(180/pi)));   
                      
        %plot(xH,yHUpd,'r');
        %hold on;
          
        lineLumbarExo = ...
            plot(ocpDataExo(:,angleLumbarExoCol).*(180/pi),...
                 ocpDataExo(:,tauLumbarExoCol).*-1,...
             lineType{idxLumbar},...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));          
        hold on;
            plot(ocpDataExo(1,angleLumbarExoCol).*(180/pi),...
                 ocpDataExo(1,tauLumbarExoCol).*-1,...
             'd',...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));
        hold on;
        
        xL = [ones(size(ocpDataExo(:,angleLumbarExoCol).*(1*180/pi))),...
             ocpDataExo(:,angleLumbarExoCol).*(1*180/pi)];
        yL = ocpDataExo(:,tauLumbarExoCol).*-1;
        bL = xL\yL;
        yLUpd = bL(1).*xL(:,1) + bL(2).*xL(:,2);
        
        disp(sprintf('X-Intercept %e \tSlope %e \tStiffness(Nm/rad) %e: Lumbar-Char',...
                      bL(1),bL(2),bL(2).*(180/pi)));   
                      
        %plot(xL,yLUpd,'g');
        %hold on;
        
              
        xlabel('Angle ($^\circ$)');
        ylabel('Torque (Nm)');
        title('Actuator Torque-Angle Curves');  
        axis tight;        
        axis square;
        box off;

        
        
        
        %legend([lineHipExo,lineLumbarExo],'Hip Actuator','Lumbar Actuator',...
        %        'Location','South');
        %legend boxoff;
        
%%
% Angle
%%
figure(fig3);
    subplot('Position',subPlotSquare);
        hipMotorAngle = ocpDataExo(:,angleHipExoCol).*(-180/pi);
        lineHipExo = ...
            plot(ocpDataExo(:,timeCol), hipMotorAngle,...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        peakData(1,1) = max(hipMotorAngle)-min(hipMotorAngle);
        
        lumbarMotorAngle = ocpDataExo(:,angleLumbarExoCol).*(180/pi);
        lineLumbarExo = ...
            plot(ocpDataExo(:,timeCol), lumbarMotorAngle,...
             lineType{idxLumbar},...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));
        hold on;
        
        peakData(2,1) = max(lumbarMotorAngle)-min(lumbarMotorAngle);
        
        xlabel('Time (s)');
        ylabel('Angle ($^\circ$)');
        title('Actuator Angles');        
        axis tight;        
        axis square;
        box off;
        xlim([xmin,xmax]);
        
        ax = gca;
        ax.XTick = xTickTime;
        
        %legendProperties = legend([lineHipExo,lineLumbarExo], ...
        %                   'Hip Actuator','Lumbar Actuator',...
        %                   'Location','none');
        h = subPlotSquare(1,4);
        w = subPlotSquare(1,3);
        l = subPlotSquare(1,1);
        b = subPlotSquare(1,2);

        %set(legendProperties,'Position', ...
        %[(l+w*0.4) (b+0.05*h) (w*0.3) (h*0.1) ]);     
            
        %legend boxoff;

        
%%
% Power
%%
figure(fig4);

hipMotorPower       =  ocpDataExo(:,angularVelHipExoCol)...
                     .*ocpDataExo(:,tauHipExoCol);
lumbarMotorPower    =  ocpDataExo(:,angularVelLumbarExoCol)...
                     .*ocpDataExo(:,tauLumbarExoCol);

    subplot('Position',subPlotSquare);
        lineHipExo = ...
            plot(ocpDataExo(:,timeCol), hipMotorPower,...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        [val idx] = max(abs(hipMotorPower));
        peakData(1,3) = val*sign(hipMotorPower(idx));
        
        lineLumbarExo = ...
            plot(ocpDataExo(:,timeCol), lumbarMotorPower,...
             lineType{idxLumbar},...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));
        hold on;
        
        [val idx] = max(abs(lumbarMotorPower));
        peakData(2,3) = val*sign(lumbarMotorPower(idx));
        
        xlabel('Time (s)');
        ylabel('Power ($W$)');  
        title('Actuator Power Output');
        axis tight;
        axis square;
        box off;
        xlim([xmin,xmax]);
        
        [hL hO] = legend([lineHipExo,lineLumbarExo],'Hip Motor','Lumbar Motor',...
                'Location','none');
            
            
        h = subPlotSquare(1,4);
        w = subPlotSquare(1,3);
        l = subPlotSquare(1,1);
        b = subPlotSquare(1,2);

        set(hL,'Position', ...
        [(l+0.2*w) (b+0.85*h) (w*0.2) (h*0.1) ]);     
              
        for z=[3,5]
        tmp = get(hO(z),'XData');
        set(hO(z),'XData', [(tmp(2)-(tmp(2)-tmp(1))*0.3),tmp(2)]);
        end
        
        ax = gca;
        ax.XTick = xTickTime;
        
        legend boxoff; 

%%
% Angular velocity
%%
figure(fig5);
    subplot('Position',subPlotSquare);
        hipMotorAngularVelocity    = ocpDataExo(:,angularVelHipExoCol).*(-180/pi);
        lumbarMotorAngularVelocity = ocpDataExo(:,angularVelLumbarExoCol).*(180/pi);
        
        lineHipExo = ...
            plot(ocpDataExo(:,timeCol), hipMotorAngularVelocity,...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        [val idx] = max(abs(hipMotorAngularVelocity));
        peakData(1,4) = val*sign(hipMotorAngularVelocity(idx));
        
        lineLumbarExo = ...
            plot(ocpDataExo(:,timeCol), lumbarMotorAngularVelocity,...
             lineType{idxLumbar},...
              'Color',lineColor(idxLumbar,:),...
              'LineWidth', lineWidth(idxLumbar));
        hold on;
        
        [val idx] = max(abs(lumbarMotorAngularVelocity));        
        peakData(2,4) = val*sign(lumbarMotorAngularVelocity(idx));
        
        xlabel('Time (s)');
        ylabel('Angular Velocity ($^\circ/s$)');
        title('Actuator Angular Velocity');        
        axis tight;        
        axis square;
        box off;
        xlim([xmin,xmax]);
        
        %legendProperties = legend([lineHipExo,lineLumbarExo], ...
        %                   'Hip Actuator','Lumbar Actuator',...
        %                   'Location','none');
        h = subPlotSquare(1,4);
        w = subPlotSquare(1,3);
        l = subPlotSquare(1,1);
        b = subPlotSquare(1,2);

        %set(legendProperties,'Position', ...
        %[(l+w*0.4) (b+0.05*h) (w*0.3) (h*0.1) ]);     
            
        ax = gca;
        ax.XTick = xTickTime;
        
        %legend boxoff;        

        
%%
% Passive hip extension torque
%%
figure(fig6);
    subplot('Position',subPlotSquare);
        passiveHipExtTau    = ocpDataExo(:,tauPassiveHipExt);
        activeHipExtTau    = ocpDataExo(:,tauActiveHipExt);
        
        linePassiveHipExt = ...
            plot(ocpDataExo(:,timeCol), passiveHipExtTau,...
             lineType{idxHip},...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        lineActiveHipExt = ...
            plot(ocpDataExo(:,timeCol), activeHipExtTau,...
             '--',...
              'Color',lineColor(idxHip,:),...
              'LineWidth', lineWidth(idxHip));
        hold on;
        
        xlabel('Time (s)');
        ylabel('Torque (Nm)');
        title('Passive Hip Extension');        
        axis tight;        
        axis square;
        box off;
        xlim([xmin,xmax]);
        
    
        
%%
%Write the plots
%%

csvwrite([outputTableFolder,'peakExoHipLumbarMotorAngleTorquePowerOmega.csv'],...
         peakData);

figNameList = {['fig_',costFcn,'_ExoTorqueOutput.pdf'],...
               ['fig_',costFcn,'_ExoTorqueAngle.pdf'],...
               ['fig_',costFcn,'_ExoAngle.pdf'],...
               ['fig_',costFcn,'_ExoPower.pdf'],...
               ['fig_',costFcn,'_ExoAngularVelocity.pdf'],...
               ['fig_',costFcn,'_PassiveHipExt.pdf']};     

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
         print('-dpdf', [outputFigureFolder,sprintf('box%dkg',boxMass),'/', figNameList{i}]);  
end        

