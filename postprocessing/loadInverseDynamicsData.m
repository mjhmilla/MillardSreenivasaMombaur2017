
idFolder = ['data/',sprintf('box%dkg',boxMass),'/','ID/2D/'];
idTrial = 'stoop2'; 
idData = struct('time',zeros(1,1),'q',zeros(1,1),'qDot',zeros(1,1), ...
                'qDotDot',zeros(1,1),'tau',zeros(1,1),'a',zeros(1,1));

if(flag_IdDataAvailable==1)            
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
end

idx = 1;  
idxIdBoxPosX=idx;
idx=idx+1;
idxIdBoxPosZ=idx;
idx=idx+1;
idxIdBoxRotY=idx;
idx=idx+1;
idxIdPelvisPosX=idx;
idx=idx+1;
idxIdPelvisPosZ=idx;
idx=idx+1;
idxIdPelvisRotY=idx;
idx=idx+1;
idxIdRightHipRotY=idx;
idx=idx+1;
idxIdRightKneeRotY=idx;
idx=idx+1;
idxIdRightAnkleRotY=idx;
idx=idx+1;
idxIdLeftHipRotY=idx;
idx=idx+1;
idxIdLeftKneeRotY=idx;
idx=idx+1;
idxIdLeftAnkleRotY=idx;
idx=idx+1;
idxIdMiddleTrunkRotY=idx;
idx=idx+1;
idxIdUpperTrunkRotY=idx;
idx=idx+1;
idxIdHeadRotY=idx;
idx=idx+1;
idxIdRightUpperArmRotY=idx;
idx=idx+1;
idxIdRightLowerArmRotY=idx;
idx=idx+1;
idxIdRightHandRotY=idx;
idx=idx+1;
idxIdLeftUpperArmRotY=idx;
idx=idx+1;
idxIdLeftLowerArmRotY=idx;
idx=idx+1;
idxIdLeftHandRotY=idx;
numOfAniColIndices = idx;


