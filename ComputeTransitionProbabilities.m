function P = ComputeTransitionProbabilities_def(stateSpace, map)
%COMPUTE_TRANSITION_PROBABILITIES Compute transition probabilities.
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

% initializing transition probabilities matrix

Transition_probabilities_matrix = zeros(K,K,5);
w = size(map,1); %width
h = size(map,2); %height

shooters = [];   % creating an array that contains all the stateSpace indexes of the shooters

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;   %StateSpace index of the base without package
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER && stateSpace(i,3) == 0)
            
            shooters = [shooters, i]; %StateSpace indexes of shooters (without package)
            
        elseif((map(stateSpace(i,1),stateSpace(i,2)) == PICK_UP) && stateSpace(i,3) == 0)
            
            pick_up = i; %StateSpace index of pick_up station without package
            m_pick_up = stateSpace(i,1);
            n_pick_up = stateSpace(i,2);
            
        elseif((map(stateSpace(i,1),stateSpace(i,2)) == DROP_OFF) && stateSpace(i,3) == 1)
            
            drop_off = i; %StateSpace index of drop_off station with package
            m_drop_off = stateSpace(i,1);
            n_drop_off = stateSpace(i,2);
            
        end
        
end

%Matrix of the probability of crash due to a TREE or a SHOOTER  
Crashing_probabilities = zeros(size(map,1),size(map,2));

for m = 1 : size(map,1)
    
    for n = 1 : size(map,2)
        
        if(map(m,n) == TREE)
            
            Crashing_probabilities(m,n) = 1;
            
        else
            
            for s = 1 : size(shooters,2)
                
                %Manhattan distance
                distance = abs(m - stateSpace(shooters(s),1)) + abs(n - stateSpace(shooters(s),2)); 
                
                if(distance <= R) 
                    
                    Crashing_probabilities(m,n) = Crashing_probabilities(m,n) + GAMMA./(distance + 1);
                    
                end
                
            end
            
        end
        
    end
    
end

%Filling the Transition Probabilities Matrix
for i = 1 : K  
    
    m_i = stateSpace(i,1);  
    n_i = stateSpace(i,2);  
    pack_i = stateSpace(i,3);
    
    for j = 1 : K   
        
        m_j = stateSpace(j,1);  % m_j
        n_j = stateSpace(j,2);  % n_j
        pack_j = stateSpace(j,3);
        
        %When appling one of the possible inputs, the drone can stay in the
        %same cell or move by one cell depending on the input chosen. 
        %The following if condition is entered only if the j picked belongs to 
        %the 5 aformentioned ones
        
        if(((m_j == m_i + 1 && n_j == n_i) || (m_j == m_i - 1 && n_j == n_i) || (m_j == m_i && n_j == n_i + 1) || (m_j == m_i && n_j == n_i - 1) || (m_j == m_i && n_j == n_i)))
            
            for u = 1 : 5
                
                %if the drone starts from the state corresponding to the
                %drop_off variable, the probability that it remains there 
                %for any chosen input is 1                
                if (i ~= drop_off)  
                    
                    %Edge cases where 'package state' changes between
                    %transitions (while over the PICK_UP station and while crashing holding the package),
                    %are not covered in the next if condition.
                    if (pack_i == pack_j)
                        
                        %The probability matrix is updated if the chosen
                        %input and the movement towards a cell are possible
                        if ((u == NORTH && n_j == n_i + 1 && m_j == m_i) || (u == SOUTH && n_j == n_i - 1 && m_j == m_i) || (u == EAST && m_j == m_i + 1 && n_i == n_j) || (u == WEST && m_j == m_i - 1 && n_i == n_j) || (u == HOVER && m_j == m_i && n_i == n_j))
                            
                            %If the drone, starting from a state i, will
                            %arrive at the PICK_UP station without a
                            %package, it will pick the package 
                            if (j == pick_up) 
                                
                                %state i 'package state' = 0. If drone after
                                %movement ends in pick_up station, it
                                %will pick up the package -> 'package state' = 1,
                                %which corresponds not to state j, but to state j + 1
                                Transition_probabilities_matrix(i,j+1,u) = (1 - P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                                
                                %Probability to go from a state without a
                                %package to the pick_up station and not
                                %picking up the package
                                Transition_probabilities_matrix(i,j,u) = 0;
                                
                            %drone started from state i with 'package state' = 1
                            elseif (j == drop_off) 
                                
                                %The probability of starting with a package
                                %and arriving at the drop_off station
                                %without the package (index j-1) is zero
                                Transition_probabilities_matrix(i,j-1,u) = 0;
                                
                                %Probability of starting with a package and
                                %arriving at the drop off station with a
                                %package
                                Transition_probabilities_matrix(i,j,u) = (1 - P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                               
                            %in all other cases   
                            else
                                
                                Transition_probabilities_matrix(i,j,u) = (1-P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                                
                            end
                      
                            %Transition probabilities matrix updated in case of 2 cells motion due to wind.
                            %All 9 neighbouring cells of j are considered 
                            %(with j being the final drone destination in absence of wind)
                            for m = m_j-1 : m_j+1
                                
                                for n = n_j-1 : n_j+1
                                    
                                    is_neighbour = 0;
                                    
                                    %Only wind-possible motions are considered in the following if condition
                                    %(diagonal motions not considered)
                                    if ((abs(m_j-m) == 0 && abs(n_j-n) == 1) || (abs(m_j-m) == 1 && abs(n_j-n) == 0))     % cioe' se (m,n) e' uno stato a N,S,E,W di j
                                        
                                        is_neighbour = 1;
                                        
                                    end
                                    
                                    
                                    if (is_neighbour == 1)  
                                     
                                        %flag that indicates if the cell where wind moves 
                                        %the drone exists in the StateSpace
                                        final_state  = 0;
                                        
                                        %flags for pick_up or drop_off stations
                                        %after wind action
                                        pick = 0;
                                        drop = 0;
                                        
                                        %Counters for number of borders and trees
                                        %surrounding a cell
                                        count_borders = 0;
                                        tree = 0;

                                        %check if the wind moved the drone
                                        %outside the StateSpace
                                        for k = 1 : K    
                                            
                                            %if condition entered only if the cell (m,n) exists in
                                            %the StateSpace index
                                            if (stateSpace(k,1) == m && stateSpace(k,2) == n) 
                                                
                                                %If the final state where the wind 
                                                %moved the drone corresponds to the pick_up
                                                %station and the drone started from state i
                                                %without the package, the final state will be the
                                                %the pick_up station, but with the package
                                                if (map(stateSpace(k,1),stateSpace(k,2)) == PICK_UP && stateSpace(k,3) == 1 && pack_i == 0)
                                                   
                                                    final_state = k; 
                                                    pick = 1; 
                                                
                                                %Else if the wind moves the
                                                %drone to the drop_off station and the starting
                                                %state i was with package, the drone will end up in
                                                %the state corresponding to the drop_off station
                                                % without the package
                                                elseif (map(stateSpace(k,1),stateSpace(k,2)) == DROP_OFF &&stateSpace(k,3) == 0 && pack_i == 1)
                                                   
                                                    final_state = k;
                                                    drop = 1;
                                                
                                                %All other cases where the
                                                %package state i and k (k = drone state after wind action)
                                                %corresponds
                                                elseif (stateSpace(k,3) == pack_i)  
                                                    
                                                    final_state = k;
                                                    
                                                end
                                                
                                            end
                                            
                                        end
                                        
                                        %At this point the variable final state has value 0 if the wind made 
                                        %the drone crash, or value different from 0 otherwise
                                        
                                        %If the wind made the drone crash because moved it outside the map,
                                        %I count the number of borders surrounding the cell in which the
                                        %drone was before crashing
                                        
                                        %Case corner
                                        if (final_state == 0 && ((m==0 && n == 1) || (m == 0 && n == h) || (m == w+1 && n == 1) || (m == w+1 && n == h)...
                                                || (m == 1 && n == 0) || (m == w && n == 0) || (m == w && n == h+1) || (m == 1 && n == h+1)))
                                            
                                            count_borders = count_borders + 2;
                                            
                                        %Case lateral border    
                                        elseif (final_state == 0 && ((m == 0 && n < h && n > 0) || (m == w+1 && n < h && n > 0) || (n == 0 && m < w && m > 0) || (n == h+1 && m < h && m > 0)))
                                           
                                            count_borders = count_borders + 1;
                                            
                                        %I count the number of TREEs
                                        %surrouning the cell in which the
                                        %drone was before the wind made it crash
                                        elseif (final_state == 0 && map(m,n) == TREE)
                                            
                                            tree = tree + 1;
                                            
                                        end
                                        
                                        %If the wind does not make the drone crashes
                                        %and does not move it neither to pick_up or drop_off station 
                                        if (final_state ~= 0 && pick == 0 && drop == 0 )
                                              
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2)));
                                        
                                        %If the variable pick is 1 it means that the wind moved the drone 
                                        %to the pick_up station and the drone started from state i
                                        %without a package.
                                        %If pick == 1, the final_state will
                                        %be necessarily admissible (final_state != 0)
                                        elseif (pick == 1)
                                           
                                                
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2)));
                                                Transition_probabilities_matrix(i,final_state-1,u) = 0;
                                        
                                        %If the variable drop is 1 it means that the wind moved the drone 
                                        %to the drop_off station and the drone started from state i
                                        %with a package.
                                        %If drop == 1, the final state will
                                        %be necessarily admissible (final_state != 0)
                                        elseif (drop == 1)
                                            
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state-1,1),stateSpace(final_state-1,2)));
                                                Transition_probabilities_matrix(i,final_state+1,u) = 0;
                                                
                                        %If the wind made the drone crashes
                                        %out of the map
                                        elseif (final_state == 0 && count_borders > 0) 
                                            
                                            Transition_probabilities_matrix(i,base,u) = (count_borders * 0.25 * P_WIND);
                                        %If the wind made the drone crashes
                                        %against a TREE
                                        elseif (final_state == 0 && tree > 0)
                                            
                                            Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + tree * 0.25 * P_WIND;
                                            
                                        end
                                        
                                    end
                                end
                            end
                        end
                    end
                    
                %If the initial state i is from the drop_off station with a
                %package
                else 
                    
                    Transition_probabilities_matrix(i,i,u) = 1;
                    
                end
            end
        end
        
    end
end

for i = 1 : K

    for u = 1 : 5
        
        sum = zeros(K,5); 
        
        for j = 1 : K 
            
            if(j ~= base) 
                
                %Probabilities of going from a certain state i to all the state j, except for j = base 
                sum(i,u) = sum(i,u) + Transition_probabilities_matrix(i,j,u);
                
            end
            
        end
        
        
        
        
  
        %If sum is equal 0, it means that the chosen input for that state
        %was not admissible
        if (sum(i,u) ~= 0)
            
            Transition_probabilities_matrix(i,base,u) = (1 - sum(i,u)); 
            
        end
    end
end

P = Transition_probabilities_matrix;

end