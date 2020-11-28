function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

% Definisco vettore f della funzione obiettivo senza terminal state. In
% pratica il mio vettore V non avrà mai il terminal state(ERA QUELLO
% L'ERRORE...)

f = [];

for i = 1 : K

     if(i ~= TERMINAL_STATE_INDEX)

            f = [f;-1];
             
     end
         
end

%NUOVA SOLUZIONE CHE ELIMINA DA MATRICE A E DA VETTORE Q GLI INGRESSI NON
%AMMISSIBILI

% Definisco matrice A costruita su cinque strati, uno per ogni ingresso
% senza input non ammissibili e senza terminal state
% Ad ogni iterazione aggiungo una riga alla matrice A

A = [];

for u = 1 : 5
    
    for i = 1 : K
        
        if(i ~= TERMINAL_STATE_INDEX) 

            if(G(i,u) == Inf)  %controllo ammissibilità input

                %do nothing

            else %se l'input è ammissibile allora aggiungo ad A un vettore riga con le probabilità P(i,j,u)

                P_vector = [];

                for k = 1 : K  %riempio una riga di A
                    
                    if(k ~= TERMINAL_STATE_INDEX) 
                        
                        if(k == i)  %diagonale

                            P_vector = [P_vector,1 - P(i,k,u)];
                            
                        else
                            
                            P_vector = [P_vector, - P(i,k,u)];
                            
                        end
                    
                    end

                end

                A = [A;
                    P_vector];

            end
        
        end
        
    end
    
end

% Definisco vettore Q che è lo "srotolamento" di G senza i costi infiniti e
% senza terminal state 

Q = [];

for u = 1 : 5
    
    for i = 1 : K
        
        %non aggiungo i costi infiniti(corrispondenti a input non ammissibili)
        %e non considero il terminal state
        if(G(i,u) == Inf || i == TERMINAL_STATE_INDEX)  
            
            %do nothing
            
        else
            
            Q = [Q;
                G(i,u)];
        
        end
            
    end
        
end  

J = linprog(f,A,Q);
J_opt = zeros(K,1);

k=1;

for i = 1 : K
    
    if( i ~= TERMINAL_STATE_INDEX)
    
        J_opt(i,1) = J(k,1);
        
        k = k + 1;
        
    end
    
end 

u_opt_ind = zeros(K,1);

for i = 1 : K
    
    if(i == TERMINAL_STATE_INDEX)
        
        u_opt_ind(i) = HOVER;
        
    else
    
        min = Inf;
        index_min = 0;
    
        for u = 1 : 5

            sum = 0;

            for j = 1 : K

                sum = sum + P(i,j,u) * J_opt(j);

            end

            if((sum + G(i,u)) < min)

                min = sum + G(i,u);

                index_min = u;

            end

        end

        u_opt_ind(i) = index_min;

    end

end