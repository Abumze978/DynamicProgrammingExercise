function P = ComputeTransitionProbabilities_Leo(stateSpace, map)
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

% finding the indexes of the stateSpace corresponding to the base and the shooters

shooters = [];   % creating an array that contains all the stateSpace indexes of the shooters

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;   % index of the state 'BASE without package'
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER && stateSpace(i,3) == 0)
            
            shooters = [shooters, i];
            
        end
        
end

Crashing_probabilities = zeros(size(map,1),size(map,2));

for m = 1 : size(map,1)
    
    for n = 1 : size(map,2)
        
        if(map(m,n) == TREE)
            
            Crashing_probabilities(m,n) = 1;
            
        else
            
            for s = 1 : size(shooters,2)
                
                distance = abs(m - stateSpace(shooters(s),1)) + abs(n - stateSpace(shooters(s),2)); %manhattan distance
                
                if(distance <= R) %somma perchè unione. Basta infatti che uno solo mi becchi e sono morto
                    
                    Crashing_probabilities(m,n) = Crashing_probabilities(m,n) + GAMMA./(distance + 1);
                    
                end
                
            end
            
        end
        
    end
    
end

counter = 0;

for i = 1 : K  % prendo il primo stato
    
    m_i = stateSpace(i,1);  % m_i
    n_i = stateSpace(i,2);  % n_i
    pack_i = stateSpace(i,3);
    
    for j = 1 : K   % prendo il secondo stato
        
        m_j = stateSpace(j,1);  % m_j
        n_j = stateSpace(j,2);  % n_j
        pack_j = stateSpace(j,3);
        
        if(abs(m_i - m_j) <= 1 && abs(n_i - n_j) <= 1)
            
            for u = 1 : 5   % fissati i due stati, scorro tutti i control inputs
                
                if (pack_i == pack_j)   % only transitions where the 'package state' is consistent except for the pickup station
                    
                    if ((u == NORTH && n_j == n_i + 1 && m_j == m_i) || (u == SOUTH && n_j == n_i - 1 && m_j == m_i) || (u == EAST && m_j == m_i + 1 && n_i == n_j) || (u == WEST && m_j == m_i - 1 && n_i == n_j) || (u == HOVER && m_j == m_i && n_i == n_j))
                        
                        Transition_probabilities_matrix(i,j,u) = (1-P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                        
                        Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND) * Crashing_probabilities(m_j,n_j);
                         
                    end
                    
%                     disp(Transition_probabilities_matrix(i,j,u));
%                     
%                     disp(Transition_probabilities_matrix(i,base,u));
                    
                    % ADESSO COMINCIO A CONSIDERARE IL VENTO
                    
                    % "sono" nello stato intermedio j
                    % sto ancora considerando il caso con i come stato iniziale
                    % da j, il vento puo' spostarmi nelle 4 direzioni N,S,E,O
                    % oppure posso stare fermo (no vento)
                    
                    % cerco le 4 caselle dove posso finire, controllando se sono
                    % FREE oppure no
                    
                    %                 trees = 0;
                    %                 iterations = 0;
                    
                    for m = m_j-1 : m_j+1
                        
                        for n = n_j-1 : n_j+1
                            
                            %                         iterations = iterations + 1;
                            
                            is_neighbour = 0;
                            
                            if (abs(m_j-m) == 0 && abs(n_j-n) == 1) || (abs(m_j-m) == 1 && abs(n_j-n) == 0)     % cioe' se (m,n) e' uno stato a N,S,E,W di j
                                
                                is_neighbour = 1;
                                
                            end
                            
                            if (is_neighbour == 1)  % caso in cui c'e' vento, quindi mi sposto
                                
                                counter = counter + 1;
                                % controllo se lo stato in cui vengo spostato
                                % e' FREE oppure no
                                % devo anche controllare che lo stato in cui
                                % finisco abbia o non abbia il pacco a seconda
                                % di se lo avevo o no
                                
                                final_state  = 0;
                                
                                for k = 1 : K    % from m,n to stateSpace index. Cerco nella stateSpace l'indice corrispondente a (m,n). Se non lo trovo vuol dire che e' un TREE o un BORDER
                                    
                                    if stateSpace(k,1) == m && stateSpace(k,2) == n && stateSpace(k,3) == pack_i
                                        
                                        final_state = k;
                                        
                                    end
                                    
                                end
                                
                                if (final_state ~=0)   % NOT a crash, cioe' lo stato (m,n) = final_state e' FREE
                                    
                                    Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2)));
                                    
                                    Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + 0.25 * P_WIND * Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2));
                                    
                                else %albero o bordo
                                    
                                    Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + 0.25 * P_WIND;
                                    
                                end
                                %
                                %                                     if (map(stateSpace(final_state,1),stateSpace(final_state,2)) == PICK_UP && stateSpace(final_state,3) == 0)
                                %
                                %                                         %Transition_probabilities_matrix(k,k+1,u) = 1;
                                %                                         Transition_probabilities_matrix(i,final_state + 1,u) = 1;
                                %
                                %                                     end
                                
                                %                                 if (map(stateSpace(final_state,1),stateSpace(final_state,2)) == DROP_OFF && stateSpace(final_state,3) == 1)
                                %
                                %                                 Transition_probabilities_matrix(i,final_state - 1,u) = 1;
                                %
                                %                                 end
                                
                                %                                 else % non ho trovato lo stato nella stateSpace --> crash dovuto al vento
                                %
                                %                                     %                                 trees = trees +1;
                                %                                     Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + 0.25 * P_WIND;  % secondo me dovrebbe tener conto del numero di alberi che ci circondano
                                %                                     %
                                %                                 end
                                %
                                %                         if(iterations == 9)
                                %
                                %                             Transition_probabilities_matrix(i,base,u) = trees./4 * P_WIND;
                                %
                                %                         end
                                %
                                %                             elseif (abs(m_j-m) == 0 && abs(n_j-n) == 0)  % NON c'e' vento, quindi non posso crashare E rimango in j
                                %
                                %                                 Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) * (1 - P_WIND);
                                %
                                %                                 % CASO SHOOTERS SENZA VENTO
                                %
                                %                                 p_missed = 1;
                                %
                                %                                 for s = 1 : size(shooters,2)  % going through the shooters array
                                %
                                %                                     distance = abs(stateSpace(j,1) - stateSpace(shooters(s),1)) + abs(stateSpace(j,2) - stateSpace(shooters(s),2));
                                %
                                %                                     if(distance <= R)  % our drone ended up in a cell which is in the range of the shooters
                                %
                                %                                         % calculating the probability of not
                                %                                         % getting shot
                                %
                                %                                         p_missed = p_missed * (1-GAMMA./(distance + 1));
                                %
                                %                                         % Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + GAMMA./(distance + 1);
                                %
                                %                                     end
                                %
                                %                                 end
                                %
                                %                                 % se mi mancano tutti, aggiorno la probabilita' di
                                %                                 % stare fermo
                                %                                 Transition_probabilities_matrix(i,j,u) = Transition_probabilities_matrix(i,j,u) * p_missed;
                                %
                                %                                 % se almeno uno mi colpisce, aggiorno la
                                %                                 % probabilita' di finire alla base
                                %                                 Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + (1 - p_missed);
                                %
                                %                                 if (map(stateSpace(j,1),stateSpace(j,2)) == PICK_UP && stateSpace(j,3) == 0)
                                %
                                %                                     Transition_probabilities_matrix(i,j+1,u) = 1;
                                %
                                %                                 end
                                %
                                %                                 %                             if (map(stateSpace(j,1),stateSpace(j,2)) == DROP_OFF && stateSpace(j,3) == 1)
                                %                                 %
                                %                                 %                                 Transition_probabilities_matrix(i,j-1,u) = 1;
                                %                                 %
                                %                                 %                             end
                                %                                 %
                                %                             end
                                %                         end
                                %                     end
                                %                 end
                                
                            end
                        end
                    end
                end
            end
        end
    end
end

disp(counter)
            
            
P = Transition_probabilities_matrix;

% disp('0.81')
disp(P(77,75,SOUTH))
% disp('7/300')
disp(P(77,73,SOUTH))
% disp('0,02')
disp(P(77,39,SOUTH))
% disp('7/300')
disp(P(77,107,SOUTH))
% disp('37/300')
disp(P(77,77,SOUTH))
disp(P(77,base,SOUTH))

% for i = 1 : K
%     
%     for u = 1 : 5
%         
%         sum = 0;
%         
%         for j = 1 : K
%             
%             sum = sum + P(i,j,u);
%             
%         end
%         
%         disp(sum);
%         
%     end
%     
% end


end