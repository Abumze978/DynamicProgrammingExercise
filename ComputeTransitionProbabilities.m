function P = ComputeTransitionProbabilities(stateSpace, map)
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

counter_north = 0;
counter_east = 0;
counter_south = 0;
counter_west = 0;
counter_hover = 0;



for i = 1 : K  %extracting two indexes
    
    row_i = stateSpace(i,1);  %m_i
    col_i = stateSpace(i,2);  %n_i
    
    for j = i-1 : i
        
        row_j = stateSpace(j,1);  %m_j
        col_j = stateSpace(j,2);  %n_j
        
        admissible_movements = (abs(row_i - row_j) <= 1 ) && (abs(col_i - col_j) <= 1) && stateSpace(i,3) == stateSpace(j,3);
        
        if(admissible_movements) %checking if going from i to j is feasible
            
            %note that since j is derived from stateSpace matrix it is for sure
            %NOT a tree and it is inside the world bounds
            
            %now that I'm on i,j I go through the different inputs
            
            for u = 1 : 5
                
                switch u
                    
                    case u == EAST
                        
                        if(col_j == col_i + 1 && row_j == row_i)
                            
                            Transition_probabilities_matrix(i,j,u) = 1-P_WIND;
                            counter_east = counter_east +1;
                            
                        end    
                    
                    case u == WEST 
                        
                        if(col_j == col_i - 1 && row_j == row_i)
                            
                            counter_west = counter_west +1;
                            Transition_probabilities_matrix(i,j,u) = 1-P_WIND;
                            
                        end
                        
                    case u == NORTH
                        
                        if(row_j == row_i + 1 && col_i == col_j)
                            
                            counter_north= counter_north+1;
                            Transition_probabilities_matrix(i,j,u) = 1-P_WIND;
                            
                        end
                        
                    case u == SOUTH (row_j == row_i - 1 && col_i == col_j)
                        
                        if(row_j == row_i - 1 && col_i == col_j)
                            
                            counter_south = counter_south +1;
                            Transition_probabilities_matrix(i,j,u) = 1-P_WIND;
                            
                        end
                        
                    case u == HOVER 
                        
                        if(row_j == row_i && col_i == col_j)
                        
                            counter_hover = counter_hover + 1;
                            Transition_probabilities_matrix(i,j,u) = 1-P_WIND;
                            
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
                            
                            if(Index ~= 0)   %NOT a crash
                                
                                if(k ~= j) %aggiorno per il vento solo se k non è j!
                                    
                                    Transition_probabilities_matrix(i,k,u) = Transition_probabilities_matrix(i,j,u) * 0.25*P_WIND;
                                    
                                end
                                
                                p_missed = 1;
                                
                                for s = 1 : size(shooters,2)  %going through the shooters array
                                    
                                    distance = abs(stateSpace(k,1) - stateSpace(shooters(s),1)) + abs(stateSpace(k,2) - stateSpace(shooters(s),2));
                                    
                                    if(distance <= R)  %our drone ended up in a cell which is in the range of the shooters
                                        
                                        %calculating the probability of
                                        %getting shot
                                        
                                        p_missed = p_missed * (1-GAMMA./(distance + 1));
                                        
                                        Transition_probabilities_matrix(i,base,u) = GAMMA./(distance + 1);
                                        
                                    end
                                    
                                end
                                
                                Transition_probabilities_matrix(i,k,u) = Transition_probabilities_matrix(i,k,u) * p_missed;
                                
                                if(map(stateSpace(Index,1),stateSpace(Index,2)) == PICK_UP && stateSpace(Index,3) == 0)
                                    
                                    Transition_probabilities_matrix(Index,Index+1,u) = 1;
                                    
                                end
                                
                            else %crash caused by the wind
                                
                                Transition_probabilities_matrix(i,base,u) = 0.25*P_WIND;
                                
                            end
                            
                        end
                        
                    end
                    
                end
                
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

disp('from')
disp(stateSpace(1,1));
disp(stateSpace(1,2));
disp(stateSpace(1,3));
disp('to')
disp(stateSpace(3,1));
disp(stateSpace(3,2));
disp(stateSpace(3,3));
disp('with');
disp(P(1,3,NORTH));
disp(P(1,3,SOUTH));
disp(P(1,3,EAST));
disp(P(1,3,WEST));
disp(P(1,3,HOVER));

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

disp(counter_north);
disp(counter_east) ;
disp(counter_south) ;
disp(counter_west);
disp(counter_hover);
end
                
            
   

            
            
            




    

 
            


