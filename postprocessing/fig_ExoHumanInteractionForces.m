clc;
close all;
clear all;

flag_usingOctave = 0;

outputFigureFolder = 'figures/';
outputTableFolder  = 'tables/';

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

figList(1) = struct('h',[]);
figList(1).h  = fig1;
figList(2).h  = fig2;
figList(3).h  = fig3;

peakNormShearMoment = zeros(3,3);

%%
%Plot config
%%
numberOfHorizontalPlotColumns = 3;
plotConfig;

timeCol             = getColumnIndex('time', ocpColNamesExo);
exoPelvisFx         = getColumnIndex('Exo_Pelvis_Fx', ocpColNamesExo);
exoPelvisFz         = getColumnIndex('Exo_Pelvis_Fz', ocpColNamesExo);
exoPelvisTy         = getColumnIndex('Exo_Pelvis_Ty', ocpColNamesExo);

exoThighFx         = getColumnIndex('Exo_Thigh_Fx', ocpColNamesExo);
exoThighFz         = getColumnIndex('Exo_Thigh_Fz', ocpColNamesExo);
exoThighTy         = getColumnIndex('Exo_Thigh_Ty', ocpColNamesExo);

exoTorsoFx         = getColumnIndex('Exo_Torso_Fx', ocpColNamesExo);
exoTorsoFz         = getColumnIndex('Exo_Torso_Fz', ocpColNamesExo);
exoTorsoTy         = getColumnIndex('Exo_Torso_Ty', ocpColNamesExo);

dataIndices.x =ones(3,3).*timeCol;
dataIndices.y =[exoPelvisFx exoPelvisFz exoPelvisTy;...
                exoThighFx exoThighFz exoThighTy;...
                exoTorsoFx exoTorsoFz exoTorsoTy];




lineType  =  {'-','--','-'};
lineColor =  [0      0      0;...
              0      0      0;...
              0      0      0]./255;     
lineWidth = [1,1,1];

idxX =1;
idxZ =2;
idxY =3;


xmin =min(ocpDataExo(:,timeCol));
xmax =max(ocpDataExo(:,timeCol));

titleText ={'Exo-Pelvis Interaction Wrench',...
            'Exo-Thigh Interaction Wrench',...
            'Exo-Trunk Interaction Wrench'};

for(i=1:1:length(figList))
    
    figure(figList(i).h);

    subplot('Position',subPlotSplit(1,:));
        linePelvisTy = ...
        plot(ocpDataExo(:,dataIndices.x(i,3)),...
             -ocpDataExo(:,dataIndices.y(i,3)),...
             lineType{idxY},...
             'Color',lineColor(idxY,:),...
             'LineWidth', lineWidth(idxY));
        hold on;  
        
        dataVec = -ocpDataExo(:,dataIndices.y(i,3));
        [val idx] = max(abs(dataVec));
        peakNormShearMoment(i,3) = val*sign(dataVec(idx));
        
        xlim([xmin,xmax]);
        title(titleText{i});
        axis tight;
        xlabel('Time (s)');
        ylabel('Torque (Nm)');  
        
        ax = gca;
        ax.XTick = xTickTime;
        
        box off;
        if(i==1)
          legendProperties = legend('Moment-Y','Location','none');
            h = subPlotSplit(1,4);
            w = subPlotSplit(1,3);
            l = subPlotSplit(1,1);
            b = subPlotSplit(1,2);            
            set(legendProperties,'Position', ...
            [(l+w*0.2) (b+0.8*h) (w*0.3) (h*0.2) ]); 
          
          legend boxoff;
        end
    subplot('Position',subPlotSplit(2,:));
        linePelvisFx = ...
        plot(ocpDataExo(:,dataIndices.x(i,1)),...
             -ocpDataExo(:,dataIndices.y(i,1)),...
             lineType{idxX},...
             'Color',lineColor(idxX,:),...
             'LineWidth', lineWidth(idxX));
        hold on;
        
        dataVec = -ocpDataExo(:,dataIndices.y(i,1));
        [val idx] = max(abs(dataVec));
        peakNormShearMoment(i,1) = val*sign(dataVec(idx));

        
        linePelvisFz = ...
        plot(ocpDataExo(:,dataIndices.x(i,2)),...
             -ocpDataExo(:,dataIndices.y(i,2)),...
             lineType{idxZ},...
             'Color',lineColor(idxZ,:),...
             'LineWidth', lineWidth(idxZ));
        hold on; 
        
        dataVec = -ocpDataExo(:,dataIndices.y(i,2));
        [val idx] = max(abs(dataVec));
        peakNormShearMoment(i,2) = val*sign(dataVec(idx));

        
        xlim([xmin,xmax]);
        if(i == 2 || i == 3)
           ylim([-50,225]); 
        end
        axis tight;
        
        %axis square;
        box off;
        
        xlabel('Time (s)');
        ylabel('Force (N)');
        
        ax = gca;
        ax.XTick = xTickTime;
        
        if(i==1)
            legendProperties = ...
                legend([linePelvisFx, linePelvisFz],...
                   'Normal Force', 'Shear Force',...
                   'Location','none');
            linePos = get(legendProperties,'Position');
            
            h = subPlotSplit(2,4);
            w = subPlotSplit(2,3);
            l = subPlotSplit(2,1);
            b = subPlotSplit(2,2);
            
            set(legendProperties,'Position', ...
            [(l+w*0.2) (b+0.8*h) (w*0.3) (h*0.2) ]);   
               
            legend boxoff;
        end
end
        
%%
%Write the plots
%%
csvwrite([outputTableFolder,'peakExoNormShearMomentInteractionForces.csv'],...
          peakNormShearMoment);

figNameList = {['fig_',costFcn,'_ExoPelvisInteraction.pdf'],... 
               ['fig_',costFcn,'_ExoThighInteraction.pdf'],...
               ['fig_',costFcn,'_ExoTrunkInteraction.pdf']}   ;     

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
    