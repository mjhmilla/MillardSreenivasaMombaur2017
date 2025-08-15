function [statesAll, statesShoot, timeStamp] = ...
    fnc_formatStateVectors (time,q,qdot,auxStates, events, nPhases, nShoot, nStates)

states_startValues = [];

if(isempty(auxStates)==0)
  states_startValues = [q qdot auxStates];
else
  states_startValues = [q qdot];
end



count = 1;
for i = 1:nPhases    
    if nShoot(i) > 1
        stateEvents_res(count:count+nShoot(i)-1) = ...
            [events(i):(events(i+1)-events(i))/(nShoot(i)-1):events(i+1)];
    else
        stateEvents_res(count) = events(i);
    end
    count = length(stateEvents_res)+1;
end
stateEvents_res(count) = events(end);
stateEvents_res = ceil(stateEvents_res);

statesShoot = zeros(length(stateEvents_res),nStates);

k=1;
for i=1:1:nPhases
   stateStart = states_startValues(events(i),:);
   stateEnd = states_startValues(events(i+1),:);
   for j=1:1:nShoot(i)
       s = (j-1)/(nShoot(i)-1);
       statesShoot(k,:) = (1-s).*stateStart + s.*stateEnd;        
       k=k+1;
   end    
end
statesShoot(k,:) = statesShoot(k-1,:);

%statesShoot(1:length(stateEvents_res),1:nStates) = ...
%       states_startValues(stateEvents_res,:);

timeStamp = time(events(1):events(end),1)-time(events(1),1);
statesAll = states_startValues(events(1):events(end),:);
statesAll(end+1,:) = statesAll(end,:);
timeStamp(end+1) = timeStamp(end);