
%%
%
% Set up the index ranges for wholebodylifter2d_augmented ...
%
%%
timeIdx  = 1;

idxA = 0;
idxB = 1; 


idxA = getColumnIndex('StateBoxPosX', ocpColNames);
idxB = getColumnIndex('StateRightHandRotY',ocpColNames);
qIdx = [idxA:1:idxB];
assert(NoOfPositions == length(qIdx));


idxA = getColumnIndex('StateBoxVelX', ocpColNames);
idxB = getColumnIndex('StateRightHandRotVelY', ocpColNames);
dqIdx    = [idxA:1:idxB];
assert(length(dqIdx) == NoOfPositions);

actIdx = [];
if(NoOfActivationStates > 0)
    idxA = getColumnIndex('StateActRightHipExtensionRotY', ocpColNames);
    idxB = getColumnIndex('StateActRightHandFlexionRotY', ocpColNames);
    actIdx   = [idxA:1:idxB];
    assert(NoOfActivationStates == length(actIdx));
end


idxA = getColumnIndex('ControlRightHipExtensionRotY', ocpColNames);
idxB = getColumnIndex('ControlRightHandFlexionRotY', ocpColNames);
uIdx = [idxA:1:idxB];
assert(NoOfControls == length(uIdx));

idxA = getColumnIndex('TauBoxPosX', ocpColNames);
idxB = getColumnIndex('TauRightHandRotY', ocpColNames);
tauIdx = [idxA:1:idxB];
assert(NoOfGenForces == length(tauIdx));


idxA = getColumnIndex('ActiveMuscleRightHipExtensionRotY', ocpColNames);
idxB = getColumnIndex('ActiveMuscleRightHandFlexionRotY', ocpColNames);
mclActIdx = [idxA:1:idxB];
assert(NoOfMusclesActiveTq  == length(mclActIdx));


idxA = getColumnIndex('PassiveMuscleRightHipExtensionRotY', ocpColNames);
idxB = getColumnIndex('PassiveMuscleRightHandFlexionRotY', ocpColNames);
mclPasIdx = [idxA:1:idxB];
assert(NoOfMusclesPassiveTq  == length(mclPasIdx));

idxA = getColumnIndex('Heel_Fx', ocpColNames);
idxB = getColumnIndex('Toe_Fz', ocpColNames);
footIdx = [idxA:1:idxB];
assert(NoOfFootEntries == length(footIdx));

idxA = getColumnIndex('Hand_x', ocpColNames);
idxB = getColumnIndex('Hand_z', ocpColNames);
handIdx = [idxA:1:idxB];
assert(NoOfHandEntries == length(handIdx));