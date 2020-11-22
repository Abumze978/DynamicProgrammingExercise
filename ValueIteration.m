function [ J_opt1, u_opt_ind1 ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

terminal_state = TERMINAL_STATE_INDEX;



J_opt1 = zeros(K,1);
u_opt_ind1 = zeros(K,1);

%riempio i valori della riga i corrispondente al terminal state
J_opt1(terminal_state) = 0;
u_opt_ind1(terminal_state) = HOVER;

%matrice dei costi Vl, sarà una matrice K x 2, tengo conto solo delle
%ultime due iterazioni per fare il controllo tra l'attuale e la precedente,
%delle altre non mi importa più
V = zeros(K,2);

%inizializzo V0 con valori random
V(:,1) = randn;

finish = 0;

%Se i valori sono ancora diversi tra loro faccio un'altra iterazione
while (finish < K-1)
    
    for i = 1 : K
        
        if (i ~= terminal_state)
        
        %per tenere traccia dei 5 valori che otterrò durante la minimizzazione
        V_u = zeros(5,1);
        
        for u = 1 : 5
            
            Prob = 0;
            
            for j = 1 : K
                
                Prob = Prob + P(i,j,u)*V(j,1);
                
            end
            
            V_u(u) = G(i,u) + Prob;
            
        end
        
        %variabile che mi tiene traccia del minimo valore dei 5 input e a
        %quale input corrisponde
        Vmin = Inf;
        
        for u = 1 : 5
            
            if (V_u(u) < Vmin)
                
                Vmin = V_u(u);
                u_opt_ind1(i) = u;
                
            end
            
        end
        
        V(i,2) = Vmin;
        
        end
    end
    
    %controllo che i costi siano diversi tra di loro abbastanza
    for i = 1 : K
        
        if (i ~= terminal_state)
        
        if (V(i,1) - V(i,2) < 0.00001)
            
            %mi fermo se tutti i valori sono uguali, quindi finish deve essere
            %K-1 prima di fermarmi
            finish = finish + 1;
            
        end
        
        end
        
    end
    
    %se non mi devo fermare, sposto tutti i valori della seconda colonna
    %nella prima e azzero la seconda
    if (finish < K-1)
       
        for i = 1 : K
            
            if (i ~=terminal_state)
           
            V(i,1) = V(i,2);
            V(i,2) = 0;
            
            end
            
        end
        
    end
   
end

if  (finish == K-1)
    
    for i = 1 : K
        
        if (i ~= terminal_state)
    
        J_opt1(i,1) = V(i,2);
        
        end
    
    end
    
end

J_opt = J_opt1;
u_opt_ind = u_opt_ind1;


end