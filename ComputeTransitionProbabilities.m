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

% finding the indexes of the stateSpace corresponding to the base and the shooters

shooters = [];   % creating an array that contains all the stateSpace indexes of the shooters

for i = 1 : K
        
        if((map(stateSpace(i,1),stateSpace(i,2)) == BASE) && stateSpace(i,3) == 0)
            
            base = i;   % index of the state 'BASE without package'
            
        elseif(map(stateSpace(i,1),stateSpace(i,2)) == SHOOTER && stateSpace(i,3) == 0)
            
            shooters = [shooters, i];
            
        elseif((map(stateSpace(i,1),stateSpace(i,2)) == PICK_UP) && stateSpace(i,3) == 0)
            
            pick_up = i;
            m_pick_up = stateSpace(i,1);
            n_pick_up = stateSpace(i,2);
            
        elseif((map(stateSpace(i,1),stateSpace(i,2)) == DROP_OFF) && stateSpace(i,3) == 1)
            
            drop_off = i;
            m_drop_off = stateSpace(i,1);
            n_drop_off = stateSpace(i,2);
            
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

for i = 1 : K  % prendo il primo stato
    
    m_i = stateSpace(i,1);  % m_i
    n_i = stateSpace(i,2);  % n_i
    pack_i = stateSpace(i,3);
    
    for j = 1 : K   % prendo il secondo stato
        
        m_j = stateSpace(j,1);  % m_j
        n_j = stateSpace(j,2);  % n_j
        pack_j = stateSpace(j,3);
        
        %POSSO AVERE UN CAMBIO DI PACCHETTO SE HO UN PACCO E CRASHO
        
        if(((m_j == m_i + 1 && n_j == n_i) || (m_j == m_i - 1 && n_j == n_i) || (m_j == m_i && n_j == n_i + 1) || (m_j == m_i && n_j == n_i - 1) || (m_j == m_i && n_j == n_i)))
            %entro dentro questo if se j è uno dei quattro vicini di i (o i stesso)
            
            for u = 1 : 5   % fissati i due stati, scorro tutti i control inputs
                
                if (i ~= drop_off)
                    
                    if (pack_i == pack_j)
                        % only transitions where the 'package state' is consistent except for the pickup station
                        
                        if ((u == NORTH && n_j == n_i + 1 && m_j == m_i) || (u == SOUTH && n_j == n_i - 1 && m_j == m_i) || (u == EAST && m_j == m_i + 1 && n_i == n_j) || (u == WEST && m_j == m_i - 1 && n_i == n_j) || (u == HOVER && m_j == m_i && n_i == n_j))
                            
                            if (j == pick_up) %se arrivo alla pick up station senza pacco e parto senza pacco
                                
                                Transition_probabilities_matrix(i,j+1,u) = (1 - P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                                %vado da i a j+1 perchè prendo il pacco che prima non avevo
                                Transition_probabilities_matrix(i,j,u) = 0;
                                
                            elseif (j == drop_off) %se arrivo alla drop off CON il pacco partendo con pacco
                                
                                Transition_probabilities_matrix(i,j-1,u) = 0;
                                %non posso partire con il pacco e arrivare alla drop off con il pacco
                                Transition_probabilities_matrix(i,j,u) = (1 - P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                                %prob di partire con pacco e arrivare a drop off senza pacco senza vento
                                
                            else
                                
                                Transition_probabilities_matrix(i,j,u) = (1-P_WIND) * (1-Crashing_probabilities(m_j,n_j));
                                
                            end
                            
                            % Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + (1-P_WIND) * Crashing_probabilities(m_j,n_j);
                            
                            
                            
                            for m = m_j-1 : m_j+1
                                
                                for n = n_j-1 : n_j+1
                                    
                                    is_neighbour = 0;
                                    
                                    if ((abs(m_j-m) == 0 && abs(n_j-n) == 1) || (abs(m_j-m) == 1 && abs(n_j-n) == 0))     % cioe' se (m,n) e' uno stato a N,S,E,W di j
                                        
                                        is_neighbour = 1;
                                        
                                    end
                                    
                                    if (is_neighbour == 1)  % caso in cui c'e' vento, quindi mi sposto
                                        
                                        % controllo se lo stato in cui vengo spostato
                                        % e' FREE oppure no
                                        % devo anche controllare che lo stato in cui
                                        % finisco abbia o non abbia il pacco a seconda
                                        % di se lo avevo o no
                                        
                                        final_state  = 0;
                                        pick = 0;
                                        drop = 0;
                                        count_borders = 0;
                                        tree = 0;
                                        
                                        for k = 1 : K    % from m,n to stateSpace index. Cerco nella stateSpace l'indice corrispondente a (m,n). Se non lo trovo vuol dire che e' un TREE o un BORDER
                                            
                                            if (stateSpace(k,1) == m && stateSpace(k,2) == n) %&& stateSpace(k,3) == pack_i)
                                                %entro qui se la CELLA è ammissibile, con o senza pacco
                                                
                                                if (map(stateSpace(k,1),stateSpace(k,2)) == PICK_UP && stateSpace(k,3) == 1 && pack_i == 0)
                                                    %se final state è pick up ma parto senza pacco (caso quindi di pick up
                                                    %vero e proprio)
                                                    
                                                    final_state = k; %con pacco
                                                    pick = 1; %devo prendere il pacco
                                                    
                                                elseif (map(stateSpace(k,1),stateSpace(k,2)) == DROP_OFF &&stateSpace(k,3) == 0 && pack_i == 1)
                                                    %se parto da i con pacco e mi trovo sopra la cella drop off
                                                    
                                                    final_state = k;
                                                    drop = 1;
                                                    
                                                elseif (stateSpace(k,3) == pack_i)  % && (map(stateSpace(k,1),stateSpace(k,2)) ~= PICK_UP))
                                                    %final state diverso da pick up e hanno lo stesso pacco, quindi tutto come prima
                                                    
                                                    final_state = k;
                                                    
                                                end
                                                
                                            end
                                            
                                        end
                                        
                                        if (final_state == 0 && ((m==0 && n == 1) || (m == 0 && n == h) || (m == w+1 && n == 1) || (m == w+1 && n == h)...
                                                || (m == 1 && n == 0) || (m == w && n == 0) || (m == w && n == h+1) || (m == 1 && n == h+1)))
                                            %caso spigolo mappa
                                            
                                            count_borders = count_borders + 2;
                                            
                                        elseif (final_state == 0 && ((m == 0 && n < h && n > 0) || (m == w+1 && n < h && n > 0) || (n == 0 && m < w && m > 0) || (n == h+1 && m < h && m > 0)))
                                            %caso bordo laterale mappa
                                            
                                            count_borders = count_borders + 1;
                                            
                                        elseif (final_state == 0 && map(m,n) == TREE)
                                            %se mi manda contro un albero
                                            
                                            tree = tree + 1;
                                            
                                        end
                                        
                                        if (final_state ~= 0 && pick == 0 && drop == 0 )
                                            % arrivo in una cella ammissibile ma non sono nel caso pick up nè drop off
                                            
%                                             if(Transition_probabilities_matrix(i,final_state,u) == 0)
%                                                 %dovremmo capire perchè ci sovrascrive, senza scrivere questo if paraculo...
                                                
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2)));
                                                
                                                
%                                             end
                                            
                                        elseif (pick == 1)
                                            %mi trovo nel caso di pick up
                                            %non metto il controllo su final state ammissibile perchè se pick è 1,
                                            %significa che final state è per forza ammissibile
                                            
%                                             if(Transition_probabilities_matrix(i,final_state,u) == 0)
                                                %questo if qui potemmo anche risparmiarcelo perchè lui accede solo a caselle
                                                %con lo stesso pacco (vedi riga 101)
                                                
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state,1),stateSpace(final_state,2)));
                                                Transition_probabilities_matrix(i,final_state-1,u) = 0;
                                                
%                                             end
                                            
                                        elseif (drop == 1)
                                            
%                                             if(Transition_probabilities_matrix(i,final_state,u) == 0)
                                                %come righe sopra
                                                
                                                Transition_probabilities_matrix(i,final_state,u) = 0.25 * P_WIND * (1 - Crashing_probabilities(stateSpace(final_state-1,1),stateSpace(final_state-1,2)));
                                                Transition_probabilities_matrix(i,final_state+1,u) = 0;
                                                %la prob di andare da i con pacco a drop off con pacco è zero
                                                
                                                
%                                             end
                                            
                                        elseif (final_state == 0 && count_borders > 0) %mi sono schiantato su un bordo
                                            
                                            Transition_probabilities_matrix(i,base,u) = (count_borders * 0.25 * P_WIND);
                                            
                                        elseif (final_state == 0 && tree > 0)
                                            
                                            Transition_probabilities_matrix(i,base,u) = Transition_probabilities_matrix(i,base,u) + tree * 0.25 * P_WIND;
                                            
                                        end
                                        
                                    end
                                end
                            end
                        end
                    end
                    
                else %se parto da drop off con pacco
                    
                    Transition_probabilities_matrix(i,i,u) = 1;
                    
                end
            end
        end
        
    end
end





for i = 1 : K

    for u = 1 : 5
        
        sum = zeros(K,5); %QUESTO DOVREBBE ANDARE FUORI DAI FOR NO? FUNZIONA LO STESSO MA COSI INIZIALIZZI A ZERO LA MATRICE OGNI VOLTA INUTILMENTE
        
        for j = 1 : K 
            
            if(j ~= base) %&& Transition_probabilities_matrix(i,base,u) == 0
%                 se su base c'è già un valore, significa che sono nel caso
%                 in cui mi schianto su un bordo e quindi l'ho già calcolato
                
                sum(i,u) = sum(i,u) + Transition_probabilities_matrix(i,j,u);
                
            end
            
        end
        
        
        
        
   %end

        if (sum(i,u) ~= 0)
            Transition_probabilities_matrix(i,base,u) = (1 - sum(i,u)); %Transition_probabilities_matrix(i,base,u) + ;
            %Ho aggiunto a dx dell'uguale Prob base perchè se sono in una
            %casella dove VOLONTARIAMENTE decido di andare in base, avrò come
            %Prob base la somma tra il valore di crash e il valore dello
            %spostamento volontario. Allora succede che mentre faccio la somma,
            %mi conta anche il valore dello spostamento volontario e quano poi
            %faccio 1-somma da mettere in Prob base, andando poi a fare la
            %somma di tutta la riga, non mi viene più 1, perchè mi sto perdendo
            %la prob di andare in base volontariamente. Invece aggiungendo
            %Trans_prob a dx, se non ci vado volontariamente avrò 0 e quindi
            %amen, ma se ci posso andare volontariamente, la riga torna a
            %valere 1. CREDO
        end
    end
end

P = Transition_probabilities_matrix;



% P(307,:,HOVER)
% 
% %P(169,:,NORTH)
% 
% 
% % disp(P(77,75,SOUTH))
% % disp(P(77,73,SOUTH))
% % disp(P(77,39,SOUTH))
% % disp(P(77,107,SOUTH))
% % disp(P(77,77,SOUTH))
% % disp(P(77,base,SOUTH))
% % 
% % 
% % disp(P(169,171,NORTH))
% % disp(P(169,195,NORTH))
% % disp(P(169,147,NORTH))
% % disp(P(169,173,NORTH))
% % disp(P(169,169,NORTH))
% % disp(P(169,base,NORTH))





end