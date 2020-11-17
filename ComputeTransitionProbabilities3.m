function P = ComputeTransitionProbabilities3(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%Initiliazing transition probabilities matrix

Transition_probabilities_matrix = zeros(K,K,5);

%finding the index of the base and the indexes of the shooters

shooters = [];   %creating an array that contains all the stateSpace indexes of the shooters

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER && stateSpace(i,3) == 0) %sennò mi dà due stati per ogni shooter
            
            shooters = [shooters, i];
            
        end
        
end

%MANCA PICK UP

for i = 1 : K  %extracting two indexes
    
    m_i = stateSpace(i,1);
    n_i = stateSpace(i,2);
    
    for j = 1 : K
        
        m_j = stateSpace(j,1);
        n_j = stateSpace(j,2);
        
        if(abs(m_i-m_j)<=1 && abs(n_i-n_j)<=1)
            
            admissible_movements = stateSpace(i,3) == stateSpace(j,3);
            
            if(admissible_movements) %checking if going from i to j is feasible
                
                %note that since j is derived from stateSpace matrix it is for sure
                %NOT a tree and it is inside the world bounds
                
                %now that I'm on i,j I go through the different inputs
                
                for u = 1 : 5
                    
                    if (u == NORTH)   
                        
                        if (n_j == n_i + 1 && m_j == m_i)  % regular movement
                            
                            p_missed = 1;
                            
                            for s = 1 : size(shooters,2)  %going through the shooters array
                                
                                distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                
                                if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                    
                                    %calculating the probability of
                                    %getting shot
                                    
                                    p_missed = p_missed * (1-GAMMA./(distance + 1));
                                    
                                end
                                
                            end
                            
                            Transition_probabilities_matrix (i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND)*(1-p_missed);
                            
                            Transition_probabilities_matrix(i,j,u) = (1-P_WIND)*p_missed;
                            
                        end
                        
                    elseif (u == SOUTH)
                        
                        if (n_j == n_i - 1 && m_j == m_i)  % regular movement
                            
                            p_missed = 1;
                            
                            for s = 1 : size(shooters,2)  %going through the shooters array
                                
                                distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                
                                if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                    
                                    %calculating the probability of
                                    %getting shot
                                    
                                    p_missed = p_missed * (1-GAMMA./(distance + 1));
                                    
                                end
                                
                            end
                            
                            Transition_probabilities_matrix (i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND)*(1-p_missed);
                            
                            Transition_probabilities_matrix(i,j,u) = (1-P_WIND)*p_missed;
                            
                        end
                        
                    elseif (u == EAST)
                        
                        if (m_j == m_i + 1 && n_i == n_j)  % regular movement
                            
                            p_missed = 1;
                            
                            for s = 1 : size(shooters,2)  %going through the shooters array
                                
                                distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                
                                if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                    
                                    %calculating the probability of
                                    %getting shot
                                    
                                    p_missed = p_missed * (1-GAMMA./(distance + 1));
                                    
                                end
                                
                            end
                            
                            Transition_probabilities_matrix (i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND)*(1-p_missed);
                            
                            Transition_probabilities_matrix(i,j,u) = (1-P_WIND)*p_missed;
                            
                        end
                        
                    elseif (u == WEST)
                        
                        if (m_j == m_i - 1 && n_i == n_j) % regular movement
                            
                            p_missed = 1;
                            
                            for s = 1 : size(shooters,2)  %going through the shooters array
                                
                                distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                
                                if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                    
                                    %calculating the probability of
                                    %getting shot
                                    
                                    p_missed = p_missed * (1-GAMMA./(distance + 1));
                                    
                                end
                                
                            end
                            
                            Transition_probabilities_matrix (i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND)*(1-p_missed);
                            
                            Transition_probabilities_matrix(i,j,u) = (1-P_WIND)*p_missed;
                            
                        end
                        
                    else  %HOVER
                        
                        if (m_j == m_i && n_i == n_j)    % regular movement
                            
                            p_missed = 1;
                            
                            for s = 1 : size(shooters,2)  %going through the shooters array
                                
                                distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                
                                if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                    
                                    %calculating the probability of
                                    %getting shot
                                    
                                    p_missed = p_missed * (1-GAMMA./(distance + 1));
                                    
                                end
                                
                            end
                            
                            Transition_probabilities_matrix (i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND)*(1-p_missed);
                            
                            Transition_probabilities_matrix(i,j,u) = (1-P_WIND)*p_missed;
                            
                        end
                        
                    end
                        
                end
                
            end
            
        end
        
    end
    
end

%%VENTOOOO

for i = 1 : K  %extracting one index
    
    m_i = stateSpace(i,1);
    n_i = stateSpace(i,2);
    
    for u = 1 : 5
        
        n_temp = n_i;
        m_temp = m_i;
        
        if(u == NORTH)
            
            n_temp = n_temp + 1;
            
        elseif(u == EAST)
            
            m_temp = m_temp + 1;
            
        elseif(u == WEST)
            
            m_temp = m_temp - 1;
            
        elseif(u == SOUTH)
            
            n_temp = n_temp - 1;
            
        else
            
            %do nothing
            
        end
        
        trees_bounds = 0;
        counter = 0;
        p_crash_shooters_cumulata = 0;
        
        for m = m_temp - 1 : m_temp + 1
            
            for n = n_temp - 1 : n_temp + 1
                
                
                
                is_neighbour = (abs(m_temp-m) == 0 && abs(n_temp-n) == 1) || (abs(m_temp-m) == 1 && abs(n_temp-n) == 0);
                
                if(is_neighbour)
                    
                    index = 0;
                    
                    for k = 1 : K    %from m,n to stateSpace index.
                        
                        if stateSpace(k,1) == m && stateSpace(k,2) == n && stateSpace(i,3) == stateSpace(k,3)
                            
                            index = k;
                            break
                            
                        end
                        
                    end
                    
                    if(index ~=0)
                        
                        p_missed = 1;
                        
                        for s = 1 : size(shooters,2)  %going through the shooters array
                            
                            distance = abs(stateSpace(index,1) - stateSpace(shooters(s),1)) + abs(stateSpace(index,2) - stateSpace(shooters(s),2));
                            
                            if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                
                                p_missed = p_missed * (1-GAMMA./(distance + 1));
                                
                            end
                            
                        end
                        
                        p_crash_shooters_cumulata = p_crash_shooters_cumulata + 1 - p_missed;
                        
                        Transition_probabilities_matrix(i,index,u) = 0.25*P_WIND*p_missed;
                        
                        %Probabilità di andare alla base secondo me è
                        %sbagliata
                        
%                         Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + (1 - p_missed)*P_WIND;
                        
                    end
                    
                else
                    
                    trees_bounds = trees_bounds + 1;
                    
                end
                
                counter = counter + 1;
                
                if(counter == 9) %abbiamo finito di controllare tutte le 9 caselle. Aggiorniamo probablità di essere crashati per colpa del vento
                    
                    Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + P_WIND * 0.25 * trees_bounds + P_WIND * (1- trees_bounds * 0.25) * p_crash_shooters_cumulata;
         
                end
                
            end
            
        end
        
    end
    
end

P = Transition_probabilities_matrix;

a = 1;
b = 5;

disp('from')
disp(stateSpace(a,1));
disp(stateSpace(a,2));
disp(stateSpace(a,3))
disp('to')
disp(stateSpace(b,1));
disp(stateSpace(b,2));
disp(stateSpace(b,3))
disp('with')
disp(P(a,b,NORTH));
disp(P(a,b,SOUTH));
disp(P(a,b,EAST));
disp(P(a,b,WEST));
disp(P(a,b,HOVER));

c = 193;
d = 191;

disp('from')
disp(stateSpace(d,1));
disp(stateSpace(d,2));
disp(stateSpace(d,3));
disp('to')
disp(stateSpace(c,1));
disp(stateSpace(c,2));
disp(stateSpace(c,3));
disp('with');
disp(P(d,c,NORTH));
disp(P(d,c,SOUTH));
disp(P(d,c,EAST));
disp(P(d,c,WEST));
disp(P(d,c,HOVER));

disp('from')
disp(stateSpace(d,1));
disp(stateSpace(d,2));
disp(stateSpace(d,3));
disp('to')
disp(stateSpace(base,1));
disp(stateSpace(base,2));
disp(stateSpace(base,3));
disp('with')
disp(P(d,base,NORTH));
disp(P(d,base,SOUTH));
disp(P(d,base,EAST));
disp(P(d,base,WEST));
disp(P(d,base,HOVER));


disp('from')
disp(stateSpace(50,1));
disp(stateSpace(50,2));
disp(stateSpace(50,3));
disp('to')
disp(stateSpace(52,1));
disp(stateSpace(52,2));
disp(stateSpace(52,3));
disp('with');
disp(P(50,52,NORTH));
disp(P(50,52,SOUTH));
disp(P(50,52,EAST));
disp(P(50,52,WEST));
disp(P(50,52,HOVER));

i = 1;
u = HOVER;
sum = 0;

for j = 1 : K
    
    sum = sum + P(i,j,u);
    
end

disp('from');
disp(stateSpace(i,1));
disp(stateSpace(i,2));
disp(stateSpace(i,3));
disp('with input');
disp(u)
disp('probability of going to all of other states (should be 1)');
disp(sum);

end
                
            
   

            
            
            




    

 
            


