function G1 = ComputeStageCosts_Alby(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX


P = ComputeTransitionProbabilities_def(stateSpace,map);    
    
Stage_costs_matrix = zeros(K,5);

sum_matrix = zeros(K,5);

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;
            
        end
        
end


for i = 1 : K
    
    for u = 1 : 5

        for j = 1 : K
        
            sum_matrix(i,u) = sum_matrix(i,u) + P(i,j,u);
            
        end
            
    end
    
end


for i = 1 : K
    
    for u = 1 : 5
        
        if(sum_matrix(i,u) == 0)
            
            Stage_costs_matrix(i,u) = Inf;
            
        elseif(i == TERMINAL_STATE_INDEX)
            
            %do nothing, leave zero
            
        else
            
            Stage_costs_matrix(i,u) = Nc * P(i,base,u) + 1 - P(i,base,u);
            
        end
        
    end
    
end


G1 = Stage_costs_matrix;
    
end

