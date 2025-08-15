%%
% Setup plot parameters
%%
totalWidth = 177.13668/10; %Frontiers journal text width.

pageWidth  = totalWidth/numberOfHorizontalPlotColumns;
pageHeight = totalWidth/numberOfHorizontalPlotColumns;
if(flag_usingOctave == 0)
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
end
plotHorizMarginCm = totalWidth/(20*numberOfHorizontalPlotColumns/3.5);
plotVertMarginCm  = totalWidth/(10*numberOfHorizontalPlotColumns/3.5);

plotHorizMarginSplitCm = totalWidth/15;
plotVertMarginSplitCm  = totalWidth/15;


plotWidth = (pageWidth-plotHorizMarginCm)/pageWidth;
plotHeight= (pageHeight-plotVertMarginCm)/pageHeight;

plotHorizMargin = plotHorizMarginCm/pageWidth;
plotVertMargin  = plotVertMarginCm/pageHeight;

plotHorizMarginSplit = plotHorizMarginSplitCm/pageWidth;
plotVertMarginSplit  = plotVertMarginSplitCm/pageHeight;


topLeft = [0/pageWidth pageHeight/pageHeight];

subPlotSquare = zeros(1,4);
subPlotSquare(1,1) = topLeft(1)  + plotHorizMargin;
subPlotSquare(1,2) = topLeft(2)  - plotVertMargin/3 - plotHeight;
subPlotSquare(1,3) = plotWidth;
subPlotSquare(1,4) = plotHeight;

subPlotSplit = zeros(2,4);
subPlotSplit(1,1) = topLeft(1)  + plotHorizMarginSplit;
subPlotSplit(1,2) = topLeft(2)  - plotVertMarginSplit/3 - (1/3)*plotHeight;
subPlotSplit(1,3) = plotWidth.*0.8;
subPlotSplit(1,4) = (1/4)*plotHeight;

subPlotSplit(2,1) = topLeft(1)  + plotHorizMarginSplit;
subPlotSplit(2,2) = topLeft(2)  - 2*plotVertMarginSplit/3 - plotHeight;
subPlotSplit(2,3) = plotWidth.*0.8;
subPlotSplit(2,4) = (1/2)*plotHeight;


%%
%Line config
%%
idxLineID           = 1;
idxLineMinActSq     = 2;
idxLineMinActSqExo  = 3;


lineColor             = [191 191 191;...
                         0   0   0;...
                         255   0   0]./255;                     
lineWidth             = [2,1,1];
lineType              ={'-','-','-'};

xTickTime = [0,0.5,1,1.5,2,2.5,3,3.5];


