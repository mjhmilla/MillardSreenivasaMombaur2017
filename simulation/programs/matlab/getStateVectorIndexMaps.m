qIdxExo  = [];
qIdxExoFreeFlyer = [];
if(nPosExo > 0)
    qIdxExo            = [1,2,3,4,5,6,7,8,9];
    qIdxExoFreeFlyer   = qIdxExo(1:3);
    assert(nPosExo == length(qIdxExo),'qIdxExo is not consistent with nPosExo');
end

qIdxBox            = [1:1:nPosBox] + nPosExo;
qIdxBoxFreeFlyer   = qIdxBox(1:3);

qIdxHuman          = [1:1:nPosHuman] + nPosExo + nPosBox;
qIdxHumanFreeFlyer = qIdxHuman(1:3); 

assert(nPosBox == 3, 'Experimental data configured for a box with 3 states');
qIdIdxBox            = [1:1:nPosBox];
qIdIdxBoxFreeFlyer   = qIdIdxBox(1:3);

qIdIdxHuman          = [1,2,3, 4,5,6, 10,11,12, 13,14,15] + length(qIdIdxBox);
assert(length(qIdIdxHuman) == nPosHuman,'qIdIdxHuman length inconsisent with nPosHuman');
qIdIdxHumanFreeFlyer = qIdIdxHuman(1:3); 

nStatesHuman = length(qIdxHuman)*2;
nStatesBox   = length(qIdxBox)*2;
nStatesExo   = nPosExo*2;

actStates = [];
if(flag_addActivationDynamics == 1)
   actStates = ones(size(qExp,1), nStatesActivation).*0.11;
else
   nStatesActivation = 0; 
end

nStates      = nStatesBox + nStatesExo + nStatesHuman + nStatesActivation + nStatesTauDotExo;

nControlsHuman = (nPosHuman-3)*2;
assert(nControlsHuman == 18,'nControlsHuman not consistent with one-armed one-legged 2dof model');
nControls = nControlsHuman + nControlsExo;

colMapExp2Model = [qIdIdxBox,qIdIdxHuman];
colMapModel2Exp = [qIdxBox,qIdxHuman];