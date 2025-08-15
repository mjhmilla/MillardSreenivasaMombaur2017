clc;
close all;
clear all;

datFolder = '../../OCP/PRE/IK/2D';

folderContents = dir([datFolder, '/*.csv']);
numCsvFiles = length(folderContents(not([folderContents.isdir])));
nIter = 3;

for i=1:1:nIter
    qFileName        = sprintf('qIK_stoop%i_2D.csv',i);

    mkrErrorFileName = sprintf('markerError_stoop%i_2D.csv',i);

    data = csvread([datFolder,'/',qFileName]);

    time = data(:,1);
    qExp = data(:,2:1:size(data,2));

    %tf   = max(time);
    %freq = 100;
    %npts = floor(tf*100);
    npts = length(time);
    freq = floor(npts/(max(time)-min(time)));
    
    timeU = time;%[0:(1/(npts-1)):1]'.*tf;
    dtU   =  tf*(1/(npts-1));
    qU    = zeros(npts,size(qExp,2));

    [b,a] = butter(2, 10/(0.5*freq));

    for j=1:1:size(qExp,2)
       qU(:,j) = interp1(time,qExp(:,j),timeU); 

       qU(:,j) = filtfilt(b,a,qU(:,j));
    end

    qFilteredFileName= sprintf('qIKFiltered_stoop%i_2D.csv',i);
    %timeFileName     = sprintf('time_stoop%i_2D.csv',i);
    
    csvwrite([datFolder,'/',qFilteredFileName],[timeU,qU]);
    %csvwrite([datFolder,'timeExp.csv'],[timeU]);    
    
    
    mkrData   = csvread([datFolder,'/',mkrErrorFileName],1,0);
    
    fid = fopen([datFolder,'/',mkrErrorFileName]);
    header = textscan(fid, '%s', 40, 'delimiter',',');
    fclose(fid);
    
    meanMkrError = mean(mkrData(:,3:1:size(mkrData,2)));
    stdMkrError  = std(mkrData(:,3:1:size(mkrData,2)));
    maxMkrError  = max(mkrData(:,3:1:size(mkrData,2)));
    
    [maxErrVal, maxErrCol] = max(maxMkrError);
    
    [maxErrValVec, maxErrColVec] = sort(maxMkrError,2,'descend');
    
    disp(['  Marker Error (mm): ', mkrErrorFileName]);
    disp(sprintf('  Mean %0.1f +/- %0.1f, Max %0.1f in col %d %s', ...
         mean(meanMkrError).*1000,...
         mean(stdMkrError).*1000,...
         maxErrVal.*1000, maxErrCol+2, header{1}{maxErrCol+2}));
    
    for k=1:1:length(maxErrValVec)
       hIdx = maxErrColVec(k)+2;
       disp(sprintf('    %s: %0.1f', header{1}{hIdx},maxErrValVec(k)*1000)); 
    end
     
    here = 1; 
end


