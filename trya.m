function P = trya(stateSpace, map)
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
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER)
            
            shooters = [shooters, i];
            
        end
        
end

%PROBLEMA: accedo più volte allo stesso elemento della matrice

for i = 1 : K  %extracting two indexes
    
    row_i = stateSpace(i,1);  %m_i
    col_i = stateSpace(i,2);  %n_i
    
    for j = 1 : K
        
        row_j = stateSpace(j,1);  %m_j
        col_j = stateSpace(j,2);  %n_j+
        
        if((abs(row_i - row_j) <= 1 )&& (abs(col_i - col_j) <= 1))
        
        %note that since j is derived from stateSpace matrix it is for sure
        %NOT a tree and it is inside the world bounds
        
        %boolean variable that tells me if j is pick up station
        
        %j_is_pickup = map(stateSpace(j,1),stateSpace(j,2)) == PICK_UP;
        
        %now that I'm on i,j I go through the different inputs
        
        for u = 1 : 5
            
            if(stateSpace(i,3) == stateSpace(j,3)) %|| j_is_pickup ) %only transitions where the 'package state' is consistent except for the pickup station
                
                if(u == EAST)
                    
                    if(col_j == col_i + 1 && row_j == row_i)  %regular movement
                        
                        %Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) + 1-P_WIND;
                        Transition_probabilities_matrix(i,j,u) =1-P_WIND;
                        
                    end
                    
                elseif(u == WEST)
                    
                    if(col_j == col_i - 1 && row_j == row_i)  %regular movement
                        
                        %Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) + 1-P_WIND;  %regular movement
                        Transition_probabilities_matrix(i,j,u) =1-P_WIND;
                        
                    end
                    
                elseif(u == NORTH)
                    
                    if(row_j == row_i + 1 && col_i == col_j)  %regular movement
                        
                        %Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) + 1-P_WIND;
                        Transition_probabilities_matrix(i,j,u) =1-P_WIND;
                        
                  for m = row_j-1 : row_j+1
                
                for n = col_j-1 : col_j+1
                    
                    is_neighbour = (abs(row_j-m) == 0 && abs(col_j-n) == 1) || (abs(row_j-m) == 1 && abs(col_j-n) == 0);
                    
                    is_j = (abs(row_j-m) == 0 && abs(col_j-n) == 0);
                    
                    if(is_neighbour || is_j)
                        
                        Index = 0;
                        
                        for k = 1 : K    %from m,n to stateSpace index.
                            
                            if stateSpace(k,1) == m && stateSpace(k,2) == n
                                
                                Index = k;
                                break
                                
                            end
                            
                        end
                        
                        if(Index ~=0)   %NOT a crash
                            
                            %boolean variable that tells me if k is pick up station
                            
                            %k_is_pickup = map(stateSpace(k,1),stateSpace(k,2)) == PICK_UP;
                            
                            %boolean variable that tells me if i and k have
                            %same 'package state'
                            
                            same_state = stateSpace(i,3) == stateSpace(Index,3);
                            
                            if(same_state) %|| k_is_pickup)
                                
                                %Transition_probabilities_matrix(i,k,u) = Transition_probabilities_matrix(i,j,u) + 0.25*P_WIND;
                                Transition_probabilities_matrix(i,Index,u) = Transition_probabilities_matrix(i,Index,u) * 0.25*P_WIND;
                                
                                p_missed = 1;
                                
                                for s = 1 : size(shooters,2)  %going through the shooters array
                                    
                                    distance = abs(stateSpace(Index,1) - stateSpace(shooters(s),1)) + abs(stateSpace(Index,2) - stateSpace(shooters(s),2));
                                    
                                    if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                        
                                        %calculating the probability of
                                        %getting shot
                                        
                                        p_missed = p_missed * (1-GAMMA./(distance + 1));
                                        
                                        Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + GAMMA./(distance + 1);
                             
                                    end
                                    
                                end
                                
                                Transition_probabilities_matrix(i,Index,u) = Transition_probabilities_matrix(i,Index,u) * p_missed;
                                
                            end
                            
                            if(map(stateSpace(Index,1),stateSpace(Index,2)) == PICK_UP && stateSpace(Index,3) == 0)
                                
                                Transition_probabilities_matrix(k,Index+1,u) = 1; 
                                
                            end
                            
                        else %crash caused by the wind
                            
                            Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + 0.25*P_WIND;
                            
                        end
                        
                    end
                    
                end
                        end
                    end
                          
                  
                elseif(u == SOUTH)
                    
                    if(row_j == row_i - 1 && col_i == col_j) %regular movement
                        
                        %Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) + 1-P_WIND;
                        Transition_probabilities_matrix(i,j,u) =1-P_WIND;
                        
                    end
                    
                else  %HOVER
                    
                    if(row_j == row_i && col_i == col_j)    %regular movement
                        
                        %Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) + 1-P_WIND;
                        Transition_probabilities_matrix(i,j,u) =1-P_WIND;
                        
                    end
                    
                end
                
            end
            
            %Here I'm accounting for wind movement which can bring
            %the drone outside the world bounds or into a tree! So I
            %have to check where I'm ending up
            
            %I just check the 9 cells nearby j, included j (there is
            %no point on looking over the whole map).
            %I select the 4 ones in which the wind can bring the drone
            %If the wind didn't make our drone crash I check for the
            %shooters
    
           
                        
                    %elseif(abs(row_j-m) == 0 && abs(col_j-n) == 0)  %NON ci siamo mossi col vento
                        
                        %for s = 1 : size(shooters,2)  %going through the shooters array
                            
                            %distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                            
                            %p_missed = 1;
                            
                            %if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                
                                %calculating the probability of
                                %getting shot
                                
                                %p_missed = p_missed * (1-GAMMA./(distance + 1));
                                
                                %Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + GAMMA./(distance + 1);

                            %end
                            
                        %end
                        
                        %Transition_probabilities_matrix(i,k,u) = Transition_probabilities_matrix(i,k,u) * p_missed;
                        
                    end
                    
                end
                
            end
            
        end
     
    
P = Transition_probabilities_matrix;

disp(stateSpace(10,1));
disp(stateSpace(10,2));
disp(stateSpace(30,1));
disp(stateSpace(30,2));
disp(P(10,30,1));

disp(stateSpace(9,1));
disp(stateSpace(9,2));
disp(stateSpace(11,1));
disp(stateSpace(11,2));
disp(P(9,11,1));
disp(P(9,11,2));
disp(P(9,11,3));
disp(P(9,11,4));
disp(P(9,11,5));
end
                
            
            
            
            
            




    

 
            


