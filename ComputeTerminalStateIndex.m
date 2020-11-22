function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix
global DROP_OFF


for m = 1 : size(map, 1)       
    for n = 1 : size(map, 2)   
        if map(m, n) == DROP_OFF
            drop_row = m;
            drop_col = n;
            break
        end
    end                  
end 
global K

for k = 1 : K
    if stateSpace(k,1) == drop_row && stateSpace(k,2) == drop_col && stateSpace(k,3) == 1
        stateIndex = k;
        break
    end
    
end

end