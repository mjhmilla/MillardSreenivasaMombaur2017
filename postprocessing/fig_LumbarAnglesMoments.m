clc;
close all;
clear all;

flag_usingOctave = 0;

outputFigureFolder = 'figures/';
outputTableFolder = 'tables';


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


peakTable = zeros(4,2);


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
    ocpData = ocpData.*0;
end
[ocpDataExo ocpColNamesExo] = ...
    getFileAndColumnNames(ocpFolderExo,...
                         'wholebodylifter2d_augmented.txt',...
                         'wholebodylifter2d_augmented_columnNames.txt');                     
                    
loadInverseDynamicsData;


%%
%Plot config
%%
numberOfHorizontalPlotColumns = 2;
plotConfig;

%%
%Generate the plots.
%%

if(flag_IdDataAvailable == 1)
    [tauMax, idxTauMax] = max(-1.*idData.tau(:,idxIdMiddleTrunkRotY));
    timeTauMax = idData.time(idxTauMax);    
else    
    idxTimeExo           = getColumnIndex('time', ocpColNamesExo);
    idxMiddleTrunkTauExo = getColumnIndex('TauMiddleTrunkRotY', ocpColNamesExo);
    
    [minActSqExoMaxTq idxTauMax] = max(-1.*(ocpDataExo(:,idxMiddleTrunkTauExo)));
    timeTauMax = ocpDataExo(idxTauMax,idxTimeExo);

end

  
fig1 = figure;
subplot('Position',subPlotSquare);

    %%
    %ID
    %%
    if(flag_IdDataAvailable == 1)
        idMaxFlexionAngle = max((idData.q(:,idxIdMiddleTrunkRotY)...
                       +idData.q(:,idxIdUpperTrunkRotY)).*(180/pi));
        disp(sprintf('%e: ID max flexion angle',idMaxFlexionAngle));

        peakTable(1,1) = idMaxFlexionAngle;

        lineHuman = ...
            area( idData.time, ...
             (idData.q(:,idxIdMiddleTrunkRotY)...
             +idData.q(:,idxIdUpperTrunkRotY)).*(180/pi),...
             'FaceColor',lineColor(idxLineID,:),...
             'EdgeColor',lineColor(idxLineID,:));
        hold on;
    end
    
    %%
    %Min. Act Sq
    %%
    
    if(flag_ocpNoExoAvailable==1)
        idxTime             = getColumnIndex('time', ocpColNames);
        idxMiddleTrunkRotY = getColumnIndex('StateMiddleTrunkRotY', ocpColNames);
        idxUpperTrunkRotY  = getColumnIndex('StateUpperTrunkRotY',  ocpColNames);

        minActSqMaxFlexionAngle = max(( ocpData(:,idxMiddleTrunkRotY)...
                    +ocpData(:,idxUpperTrunkRotY)).*(180/pi));
        disp(sprintf('%e: min. act. sq max flexion angle',minActSqMaxFlexionAngle));

        lineOcpMinActSq = ...
            plot(   ocpData(:,idxTime), ...
                  ( ocpData(:,idxMiddleTrunkRotY)...
                    +ocpData(:,idxUpperTrunkRotY)).*(180/pi),...
                    lineType{idxLineMinActSq},...
                    'Color',lineColor(idxLineMinActSq,:),...
                    'LineWidth', lineWidth(idxLineMinActSq));
        hold on;

            peakTable(2,1) = minActSqMaxFlexionAngle;
    end
    
    %%
    %Min. Act Sq Exo
    %%
 
    idxTimeExo             = getColumnIndex('time', ocpColNamesExo);
    idxMiddleTrunkRotYExo  = getColumnIndex('StateMiddleTrunkRotY', ...
                                             ocpColNamesExo);
    idxUpperTrunkRotYExo   = getColumnIndex('StateUpperTrunkRotY',...
                                             ocpColNamesExo);
    
    minActSqExoMaxFlexionAngle = max(( ocpDataExo(:,idxMiddleTrunkRotYExo)...
                +ocpDataExo(:,idxUpperTrunkRotYExo)).*(180/pi));
    disp(sprintf('%e: min. act. sq (exo) max flexion angle',minActSqExoMaxFlexionAngle));
                                         
    lineOcpMinActSqExo = ...
        plot(   ocpDataExo(:,idxTimeExo), ...
              ( ocpDataExo(:,idxMiddleTrunkRotYExo)...
                +ocpDataExo(:,idxUpperTrunkRotYExo)).*(180/pi),...
                lineType{idxLineMinActSqExo},...
                'Color',lineColor(idxLineMinActSqExo,:),...
                'LineWidth', lineWidth(idxLineMinActSqExo));
    hold on;

    peakTable(3,1) = minActSqExoMaxFlexionAngle;
    
    %%
    % Literature
    %%
    if(flag_IdDataAvailable==1)
        tmax = max(idData.time);
        tmin = min(idData.time);
        barWidth = (tmax-tmin).*0.025;    
        plot( [timeTauMax-barWidth,timeTauMax+barWidth],...
              [(39-15),(39-15)],'k','LineWidth',0.5);
        hold on;
        plot( [timeTauMax-barWidth,timeTauMax+barWidth],...
              [(39+15),(39+15)],'k','LineWidth',0.5);
        hold on;

        peakTable(4,1) = 39;
        xlim([tmin, tmax]);

        
    else
       tmin = min(ocpDataExo(:,idxTimeExo));
       tmax = max(ocpDataExo(:,idxTimeExo));
       xlim([tmin,tmax]);
    end

    xLimits = xlim;
    yLimits = ylim;
    xWidth = xLimits(2)-xLimits(1);
    yHeight= yLimits(2)-yLimits(1);
   
    
    plot( [timeTauMax,timeTauMax],[(39-15),(39+15)],'k','LineWidth',0.5);
    hold on;
    plot( [timeTauMax],[(39)],'dk','MarkerFaceColor',[0,0,0],'MarkerSize',4);
    hold on;
    text(timeTauMax-(xWidth)*0.05, (39+14+yHeight*0.05),'Kingma et al. 2004',...
         'HorizontalAlignment','right');
    text(timeTauMax-(xWidth)*0.05, (39+14),'  10.5 kg stoop-lift',...
         'HorizontalAlignment','right');
    text(timeTauMax-(xWidth)*0.05, (39+14-yHeight*0.05),'  $39\pm14^\circ$',...
         'HorizontalAlignment','right');
    
    if(flag_IdDataAvailable == 1 && flag_ocpNoExoAvailable==1)
        legend([lineHuman,lineOcpMinActSq,lineOcpMinActSqExo],...
                'Exp. (IK/ID)','human-only OCP','with-exo OCP','Location','South');
        legend boxoff;
    end
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Lumbar Flexion Angle (L1-L5)');
    

    
    xlim(xLimits);
    ylim(yLimits);

    ax = gca;
    ax.XTick = xTickTime;
        
axis square;  
box off;

fig2 = figure;
subplot('Position',subPlotSquare);

    %%
    %ID
    %%    
    if(flag_IdDataAvailable==1)
        idMaxTq = max(-1.*idData.tau(:,idxIdMiddleTrunkRotY));
        disp(sprintf('%e: ID max tq',idMaxTq));

        lineHuman = ...
            area( idData.time, ...
             -1.*idData.tau(:,idxIdMiddleTrunkRotY),...
            'FaceColor',lineColor(idxLineID,:),...
            'EdgeColor',lineColor(idxLineID,:));
        hold on;

        peakTable(1,2) = idMaxTq;
    end
    %%
    %Min Act Sq.
    %%       
    if(flag_ocpNoExoAvailable==1)
        idxTime           = getColumnIndex('time', ocpColNames);
        idxMiddleTrunkTau = getColumnIndex('TauMiddleTrunkRotY', ocpColNames);


        minActSqMaxTq = max(-1.*(ocpData(:,idxMiddleTrunkTau)));
        disp(sprintf('%e: min act. sq max tq',minActSqMaxTq));

        peakTable(2,2) = minActSqMaxTq;    


        lineOcpMinActSq = ...
            plot( ocpData(:,idxTime), ...
            -1.*(ocpData(:,idxMiddleTrunkTau)),...
            lineType{idxLineMinActSq},...
            'Color',lineColor(idxLineMinActSq,:),...
            'LineWidth', lineWidth(idxLineMinActSq));
        hold on;
    end
    %%
    %Min Act Sq. Exo
    %%           
    idxTimeExo           = getColumnIndex('time', ocpColNamesExo);
    idxMiddleTrunkTauExo = getColumnIndex('TauMiddleTrunkRotY', ocpColNamesExo);
    
    minActSqExoMaxTq = max(-1.*(ocpDataExo(:,idxMiddleTrunkTauExo)));
    disp(sprintf('%e: min act. sq exo max tq',minActSqExoMaxTq));
    
    peakTable(3,2) = minActSqExoMaxTq;    

    
    lineOcpMinActSqExo = ...
        plot( ocpDataExo(:,idxTimeExo), ...
        -1.*(ocpDataExo(:,idxMiddleTrunkTauExo)),...
        lineType{idxLineMinActSqExo},...
        'Color',lineColor(idxLineMinActSqExo,:),...
        'LineWidth', lineWidth(idxLineMinActSqExo));
    hold on;

    if(flag_IdDataAvailable == 1)
        %legend([lineOcp, lineHuman],'OCP','Exp.','Location','SouthEast');
        tmax = max(idData.time);
        tmin = min(idData.time);
        xlim([tmin, tmax]);     
    else
       tmin = min(ocpDataExo(:,idxTimeExo));
       tmax = max(ocpDataExo(:,idxTimeExo));
       xlim([tmin,tmax]);
    end

    xLimits = xlim;
    yLimits = ylim;
    xWidth = xLimits(2)-xLimits(1);
    yHeight= yLimits(2)-yLimits(1);        
    barWidth = (tmax-tmin).*0.025;
    
    plot( [timeTauMax-barWidth,timeTauMax+barWidth],...
          [(199-12),(199-12)],'k','LineWidth',0.5);
    hold on;
    plot( [timeTauMax-barWidth,timeTauMax+barWidth],...
          [(199+12),(199+12)],'k','LineWidth',0.5);
    hold on;
    plot( [timeTauMax,timeTauMax],[(199-12),(199+12)],'k','LineWidth',0.5);    
    hold on;
    plot( [timeTauMax],[(199)],'dk','MarkerFaceColor',[0,0,0],'MarkerSize',3);
    hold on;
    
    text(timeTauMax-(xWidth)*0.05, (199+yHeight*0.05),'Kingma et al. 2004',...
         'HorizontalAlignment','right');
    text(timeTauMax-(xWidth)*0.05, (199),'  10.5 kg stoop-lift',...
         'HorizontalAlignment','right');
     
    text(timeTauMax-(xWidth)*0.05, (199-yHeight*0.05),'  199 $\pm$ 12 Nm',...
        'HorizontalAlignment','right');

    peakTable(4,2) = 199;    
    
    xlabel('Time (s)');
    ylabel('Extension Torque (Nm)');
    title('L5/S1 Net Moment');      
    
    xlim(xLimits);
    ylim(yLimits);
    
    ax = gca;
    ax.XTick = xTickTime;

axis square;
box off;

figList(2) = struct('h',[]);

figList(1).h  = fig1;
figList(2).h  = fig2;


csvwrite([outputTableFolder,'/peakLumbarAngleTorque.csv'],peakTable);

figNameList = {['fig_',costFcn,'_lumbarFlexionAngle.pdf'],...
               ['fig_',costFcn,'_lumbarExtensionMoment.pdf']}   ;     

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
         print('-dpdf', [outputFigureFolder, sprintf('box%dkg',boxMass),'/',figNameList{i}]);  
end
