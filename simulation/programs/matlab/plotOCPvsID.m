clc;
close all;
clear all;

ocpFolderMinActSq = '../../OCP/RES/minActSq_ActDyn_kkt5p8eN5/';
ocpFolderMinTauSq = '../../OCP/RES/minTauSq_converged/';


idFolder = '../../OCP/PRE/ID/2D/';
idTrial = 'stoop2';

[ocpDataMinActSq ocpColNamesMinActSq] = ...
    getFileAndColumnNames(ocpFolderMinActSq,...
                         'wholebodylifter2d_augmented.txt',...
                         'wholebodylifter2d_augmented_columnNames.txt');
   
[ocpDataMinTauSq ocpColNamesMinTauSq] = ...
    getFileAndColumnNames(  ocpFolderMinTauSq,...
                            'wholebodylifter2d_augmented.txt',...
                            'wholebodylifter2d_augmented_columnNames.txt');


idData = struct('time',zeros(1,1),'q',zeros(1,1),'qDot',zeros(1,1), ...
                'qDotDot',zeros(1,1),'tau',zeros(1,1),'a',zeros(1,1));
            
idData.time    = csvread([idFolder,'time_',   idTrial,'.csv']);
idData.q       = csvread([idFolder,'q_',      idTrial,'.csv']);
idData.qDot    = csvread([idFolder,'qDot_',   idTrial,'.csv']);
idData.qDotDot = csvread([idFolder,'qDotDot_',idTrial,'.csv']);
idData.tau     = csvread([idFolder,'tau_',    idTrial,'.csv']);
idData.a       = csvread([idFolder,'act_',    idTrial,'.csv']);


assert( size(idData.time,1) == size(idData.q,1),      'ID data not sized correctly');
assert( size(idData.time,1) == size(idData.qDot,1),   'ID data not sized correctly');
assert( size(idData.time,1) == size(idData.qDotDot,1),'ID data not sized correctly');
assert( size(idData.time,1) == size(idData.tau,1),    'ID data not sized correctly');
assert( size(idData.time,1) == size(idData.a,1),      'ID data not sized correctly');

assert( size(idData.q,2)       == 21,'ID data not sized correctly'); 
assert( size(idData.qDot,2)    == 21,'ID data not sized correctly'); 
assert( size(idData.qDotDot,2) == 21,'ID data not sized correctly'); 
assert( size(idData.tau,2)     == 21,'ID data not sized correctly'); 
assert( size(idData.a,2)       == 30,'ID data not sized correctly'); 

idxIdMiddleTrunkRotY = 13;
idxIdUpperTrunkRotY = 14;


%%
% Setup plot parameters
%%
pageWidth  = 21.0;
pageHeight = 29.7;
set(groot, 'defaultAxesFontSize',8);
set(groot, 'defaultTextFontSize',8);
set(groot, 'defaultAxesLabelFontSizeMultiplier',1.0);
set(groot, 'defaultAxesTitleFontSizeMultiplier',1.0);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultAxesTitleFontWeight','bold');
set(groot, 'defaultFigurePaperUnits','centimeters');
set(groot, 'defaultFigurePaperSize',[pageWidth pageHeight]);
set(groot,'defaultFigurePaperType','A4');


plotWidth = 6.5/pageWidth;
plotHeight= 6.5/pageHeight;
plotHorizMargin = 3.0/pageWidth;
plotVertMargin  = 3.0/pageHeight;

topLeft = [0/pageWidth pageHeight/pageHeight];

subPlotSquare = zeros(1,4);
subPlotSquare(1,1) = topLeft(1)  + plotHorizMargin;
subPlotSquare(1,2) = topLeft(2)  - plotVertMargin - plotHeight;
subPlotSquare(1,3) = plotWidth;
subPlotSquare(1,4) = plotHeight;


%%
%
%%

lineColor = [159 182 205;...
             220 20 60;...
             61  89 171]./255;
         
lineWidth = [1;1;1];



[tauMax, idxTauMax] = max(-1.*idData.tau(:,idxIdMiddleTrunkRotY));
timeTauMax = idData.time(idxTauMax);

fig1 = figure;
subplot('Position',subPlotSquare);
    lineHuman = ...
        area( idData.time, ...
         (idData.q(:,idxIdMiddleTrunkRotY)...
         +idData.q(:,idxIdUpperTrunkRotY)).*(180/pi),...
         'FaceColor',lineColor(1,:),...
         'EdgeColor',lineColor(1,:));
    hold on;
    
    idxTime             = getColumnIndex('time', ocpColNamesMinActSq);
    idxMiddleTrunkRotY = getColumnIndex('StateMiddleTrunkRotY', ocpColNamesMinActSq);
    idxUpperTrunkRotY  = getColumnIndex('StateUpperTrunkRotY',  ocpColNamesMinActSq);
    
    lineOcpMinActSq = ...
        plot(   ocpDataMinActSq(:,idxTime), ...
              ( ocpDataMinActSq(:,idxMiddleTrunkRotY)...
                +ocpDataMinActSq(:,idxUpperTrunkRotY)).*(180/pi),...
                'Color',lineColor(2,:),...
                'LineWidth', lineWidth(2));
    hold on;
    
    idxTime             = getColumnIndex('time', ocpColNamesMinTauSq);
    idxMiddleTrunkRotY = getColumnIndex('StateMiddleTrunkRotY', ocpColNamesMinTauSq);
    idxUpperTrunkRotY  = getColumnIndex('StateUpperTrunkRotY',  ocpColNamesMinTauSq);
    
    lineOcpMinTauSq = ...
        plot( ocpDataMinTauSq(:,idxTime), ...
            ( ocpDataMinTauSq(:,idxMiddleTrunkRotY)...
             +ocpDataMinTauSq(:,idxUpperTrunkRotY )).*(180/pi),...
            'Color',lineColor(3,:),...
            'LineWidth', lineWidth(2));
    hold on;

    plot( [timeTauMax,timeTauMax],[(39-15),(39+15)],'k','LineWidth',lineWidth(2));
    hold on;
    plot( [timeTauMax],[(39)],'ok','MarkerFaceColor',[0,0,0],'MarkerSize',3);
    hold on;
    text(timeTauMax*1.5, (55)*0.9,'Kingma et al. 2004');
    text(timeTauMax*1.5, (55)*0.8,'  10.5 kg box at 0.5m stoop');
    text(timeTauMax*1.5, (55)*0.7,'  $39\pm12^\circ$');
    
    %legend([lineOcp, lineHuman],'OCP','Exp.','Location','SouthWest');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Lumbar Flexion Angle (L1-L5)');
    axis tight
axis square;  
box off;

fig2 = figure;
subplot('Position',subPlotSquare);
    lineHuman = ...
        area( idData.time, ...
         -1.*idData.tau(:,idxIdMiddleTrunkRotY),...
        'FaceColor',lineColor(1,:),...
        'EdgeColor',lineColor(1,:));
    hold on;
    
    idxTime           = getColumnIndex('time', ocpColNamesMinActSq);
    idxMiddleTrunkTau = getColumnIndex('TauMiddleTrunkRotY', ocpColNamesMinActSq);
    
    lineOcpMinActSq = ...
        plot( ocpDataMinActSq(:,idxTime), ...
        -1.*(ocpDataMinActSq(:,idxMiddleTrunkTau)),...
        'Color',lineColor(2,:),...
        'LineWidth', lineWidth(2));
    hold on;

    idxTime           = getColumnIndex('time', ocpColNamesMinTauSq);
    idxMiddleTrunkTau = getColumnIndex('TauMiddleTrunkRotY', ocpColNamesMinTauSq);
        
    lineOcpMinTauSq = ...
        plot( ocpDataMinTauSq(:,idxTime), ...
        -1.*( ocpDataMinTauSq(:,idxMiddleTrunkTau)),...
        'Color',lineColor(3,:),...
        'LineWidth', lineWidth(2));
    hold on;

    %legend([lineOcp, lineHuman],'OCP','Exp.','Location','SouthEast');
    plot( [timeTauMax,timeTauMax],[(199-12),(199+12)],'k','LineWidth',lineWidth(2));
    hold on;
    plot( [timeTauMax],[(199)],'ok','MarkerFaceColor',[0,0,0],'MarkerSize',3);
    hold on;
    text(timeTauMax*1.3, (199)*1.0,'Kingma et al. 2004');
    text(timeTauMax*1.3, (199)*0.925,'  10.5 kg box at 0.5m stoop');
    text(timeTauMax*1.3, (199)*0.85,'  199 $\pm$ 12 Nm');
    %plot( [timeTauMax],[199],'dr','MarkerFaceColor',[1,0,0]);
    %hold on;
    xlabel('Time (s)');
    ylabel('Torque (Nm, Ext+)');
    title('L5/S1 Net Moment');      
    axis tight;
    ylim([-25,250]);
axis square;
box off;

figList(2) = struct('h',[]);

figList(1).h  = fig1;
figList(2).h  = fig2;

figNameList = {'fig_lumbarFlexionAngle.pdf','fig_lumbarExtensionMoment.pdf'}   ;     

for i=1:1:length(figList)
    figure(figList(i).h);
    set(figList(i).h,'Units','centimeters',...
         'PaperUnits','centimeters',...
         'PaperSize',[pageWidth pageHeight],...
         'PaperPositionMode','manual',...
         'PaperPosition',[0 0 pageWidth pageHeight]);       
         %set(findall(figList(i).h,'-property','FontSize'),'FontSize',10);       
         set(figList(i).h,'renderer','painters');       
         print('-dpdf', figNameList{i});  
end
