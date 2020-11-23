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

% Definisco vettore f della funzione obiettivo

f = zeros(K,1);

for i = 1 : K

     if(i ~= TERMINAL_STATE_INDEX)

            f(i,1) = -1;
             
     end
         
end




% Definisco matrice A costruita su cinque strati, uno per ogni ingresso
A = zeros(K*5 ,K);

for u = 1 : 5
    
    for i = 1 : K

        i_temp = i + K * (u-1);

        for j = 1 : K 
            
%             j_temp = j * u;
            
            if(i ~= TERMINAL_STATE_INDEX)

                A(i_temp,j) = - P(i,j,u);

                if(i == j) %diagonale
                    
                    A(i_temp,j) = A(i_temp,j) + 1;  

                end
                
            end
            
        end

    end
    
end


% Definisco vettore Q che è lo "srotolamento" di G

Q = zeros(K*5,1);

for u = 1 : 5
    
    for i = 1 : K
        
        i_temp = i + K*(u-1);
        
        if(G(i,u) ~= Inf) %inf mi dava problemi. Non dovrebbe essere sbagliato eliminarlo 
%                           del tutto visto che moltiplicando
%                           per P la riga dovrebbe fare zero e quindi
%                           soddisfare il vincolo
            
            Q(i_temp,1) = G(i,u);
            
        end
        
    end
    
end

J_opt = linprog(f,A,Q);

u_opt_ind = zeros(K,1);

J = zeros(K*5,1);

for u = 1 : 5

    for i = 1 : K

            i_temp = i + K * (u-1);
            
            prob = 0;
            
            for j = 1 : K
                
                prob = prob + P(i,j,u)* J_opt(j);
                
            end
            
            J(i_temp,1) =  G(i,u) + prob;

    end
    
end

for u = 1 : 5
    
    for i = 1 : K
        
        i_temp = i + K *(u-1);
        
        if(abs(J_opt(i,1) - J(i_temp,1)) < 0.00001)
            
            u_opt_ind(i,1) = u;
            
        end
        
    end
    
end

end



