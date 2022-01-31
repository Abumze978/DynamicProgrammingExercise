function G = ComputeStageCosts(stateSpace, map)
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


P = ComputeTransitionProbabilities(stateSpace,map);    
    
Stage_costs_matrix = zeros(K,5);

sum_matrix = zeros(K,5);

%shooter + base 

shooters = [];

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;
            m_base = stateSpace(i,1);
            n_base = stateSpace(i,2);
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER && stateSpace(i,3) == 0)
            
            shooters = [shooters, i];
            
        end
        
end

%creating crashing probabilities matrix

Crashing_probabilities = zeros(size(map,1) + 2,size(map,2) + 2);

for m = 2 : size(map,1) + 1
    
    for n = 2 : size(map,2) + 1
        
        if(map(m - 1,n - 1) == TREE)
            
            Crashing_probabilities(m,n) = 1;
            
        else
            
            for s = 1 : size(shooters,2)
                
                distance = abs(m - (stateSpace(shooters(s),1) + 1)) + abs(n - (stateSpace(shooters(s),2) + 1)); %manhattan distance
                
                if(distance <= R)
                    
                    Crashing_probabilities(m,n) = Crashing_probabilities(m,n) + GAMMA./(distance + 1);
                    
                end
                
            end
            
        end
        
    end
    
end

Crashing_probabilities(1,:) = 1;
Crashing_probabilities(size(map,1) + 2,:) = 1;
Crashing_probabilities(:,1) = 1;
Crashing_probabilities(:,size(map,2) + 2) = 1;


%check needed for non admissible inputs

for i = 1 : K
    
    for u = 1 : 5

        for j = 1 : K
        
            sum_matrix(i,u) = sum_matrix(i,u) + P(i,j,u);
            
        end
            
    end
    
end

%computing stage costs for every entry except for the neighbours of the
%base

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

for m = m_base - 2 : m_base + 2
    
    for n = n_base - 2 : n_base + 2
        
        %selecting the 13 neigbouring cells
        
        if(abs(m - m_base) + abs(n - n_base) <= 2) %maximum movement is 2 cells --> manhattan distance
        
            %checking if they are in the stateSpace
            
            index = 0;
        
            for k = 1 : K

                if(stateSpace(k,1) == m && stateSpace(k,2) == n && stateSpace(k,3) == 0)

                    index = k;

                end

            end
            
            if(index ~= 0)
                
                for u = 1 : 5
                    
                    m_temp = m;
                    n_temp = n;
                    go = 0;
                    ok = 0;
                    
                    if((n - n_base > 0 && u == SOUTH) || (n - n_base < 0 && u == NORTH) || (m - m_base > 0 && u == WEST) || (m - m_base < 0 && u == EAST) || (u == HOVER && ((abs(m - m_base) == 1 && abs(n - n_base) == 0) || (abs(m - m_base) == 0 && abs( n - n_base) == 1)) || (m == m_base && n == n_base)))
                        
                        if(u == SOUTH)
                            
                            n_temp = n - 1;
                            go = 1;
                            
                        elseif(u == NORTH)
                            
                            n_temp = n + 1;
                            go = 1;
                            
                        elseif(u == EAST)
                            
                            m_temp = m + 1;
                            go = 1;
                            
                        elseif(u == WEST)
                            
                            m_temp = m - 1;
                            go = 1;
                            
                        elseif(u == HOVER)
                            
                            go = 1;
                            
                        end
                        
                        for k = 1 : K

                            if(stateSpace(k,1) == m_temp && stateSpace(k,2) == n_temp && stateSpace(k,3) == 0)

                                ok = 1;

                            end

                        end
                        
                        %correction factor for crashing
                        %probabilities
                        m_temp = m_temp + 1;
                        n_temp = n_temp + 1;
                        
                        if(go == 1 && ok == 1)
                        
                             P_crash =(1 - P_WIND) * Crashing_probabilities(m_temp,n_temp) + 0.25 * P_WIND * ((Crashing_probabilities(m_temp,n_temp - 1) + Crashing_probabilities(m_temp,n_temp + 1) + Crashing_probabilities(m_temp + 1,n_temp) + Crashing_probabilities(m_temp - 1,n_temp)));
                             
                             Stage_costs_matrix(index,u) = Nc * P_crash + 1 - P_crash;
                              
                        end
                        
                    end
                
                end
                
            end
            
        end
        
    end
    
end
        

G = Stage_costs_matrix;
    
end

