function [ J_opt, u_opt_ind ] = PolicyIteration_Leo2(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

terminal_state = TERMINAL_STATE_INDEX;

% gestisco il caso terminal state
J_opt(terminal_state) = 0;
u_opt_ind(terminal_state) = HOVER;

%% INITIALIZATION
% initialize with a proper policy
mu_h = zeros(K,1);

for i = 1:K
    mu_h(i) = HOVER;
end

% calcolo J associato a mu_0
J_compare = zeros(K,2);

% for i = 1:K
%     if i ~= terminal_state
%         Prob = 0;
%         for j = 1:K
%             Prob = Prob + P(i,j,HOVER)*J_compare(j,1);
%         end
%         J_compare(i,1) = G(i,HOVER) + Prob;
%     end
% end

stop = 0;

while (stop < K-1)

    % POLICY EVALUATION
    
% NON USAVAMO MAI COST_VECTOR, per ridurre la run l'ho messo tra commenti
%     cost_vector = zeros(K,1);
%     for i = 1:K
%         if i ~= terminal_state
%             cost_vector(i) = G(i,mu_h(i));
%         end
%     end
    
    
    %NON POSSIAMO FARE QUESTO RACCOGLIMENTO PERCHE LE Pij(u) MOLTIPLICANO
    %OGNUNA UNA J DIVERSA
%     prob_matrix = zeros(K,K);
%     for r = 1:K
%         for c = 1:K
%             
%             if r ~= c
%                 prob_matrix(r,c) = -P(r,c,mu_h(r));
%             else
%                 prob_matrix(r,c) = prob_matrix(r,c) + 1;
%             end
%         end
%     end
%     
%     J_compare(:,2) = linsolve(prob_matrix, cost_vector);

%     % controllo se i costi sono uguali
%     stop = 0;
%     for i = 1:K
%         if i ~= terminal_state
%             if (J_compare(i,1) - J_compare(i,2) < 0.00001)
%                 stop = stop + 1;
%             end
%         end
%     end

PRIMO MODO
    J_sym = sym('j_%d',[1 K]); %perchè j_%d e non solo j?
    
    eq_vector = [];
    
    for i = 1:K
        if i ~= terminal_state
            
            Prob1 = 0;
            for j = 1:K

               Prob1 = Prob1 + P(i,j,mu_h(i))*J_sym(j); 

            end

            J_sym(i) = G(i,mu_h(i)) + Prob1;
            
            eq_vector = [eq_vector; J_sym(i)];
            
        else
            
            J_sym(i) = 0;
            
        end
    end
    
    % alla fine di questo for ho un linear system di K-1 equazioni lineari
    
    J_pe = solve(eq_vector, J_sym);
    
    for i = 1:K
        if i ~= terminal_state
            J_compare(i,2) = J_pe.J_sym(i);
        else
            J_compare(i,2) = 0;
        end
    end
    
% SECONDO MODO
%     for i = 1:K
%         if i ~= terminal_state
% 
%             Prob1 = 0;
%             for j = 1:K
%                 
%                 if (j ~= i)
% 
%                     Prob1 = Prob1 + P(i,j,mu_h(i))*J_compare(j,2);
%                     
%                 end
% 
%             end
% 
%             J_compare(i,2) = (G(i,mu_h(i)) + Prob1)/(1 - P(i,i,mu_h(i)));
% 
%         end
%     end

    % POLICY IMPROVEMENT
    
    J_to_min = zeros(5,1);
    
    for i = 1:K
        if i ~= terminal_state

            for u = 1:5
                
                %Ho aggiunto la minimizzazione solo sulle u ammissibili =
                %U(i)
                if (G(i,u) < Inf)

                Prob2 = 0;
                for j = 1:K

                    Prob2 = Prob2 + P(i,j,u)*J_compare(j,2); 

                end

                J_to_min(u) = G(i,u) + Prob2;
                
                end
            end

            J_min = Inf;

            for u = 1:5

                if(G(i,u) < Inf)
                
                if (J_to_min(u) < J_min)

                    J_min = J_to_min(u);
                    mu_h(i) = u;
                end
                end
            end

        end
    end

    % alla fine di questo ciclo ho una nuova policy

    % controllo se i costi sono uguali
    stop = 0;
    for i = 1:K
        if i ~= terminal_state
            %ho messo il valore assoluto... 
            if (abs(J_compare(i,1) - J_compare(i,2)) < 0.00001)
                stop = stop + 1;
            end
        end
    end
    
    J_compare(:,1) = J_compare(:,2);
    J_compare(:,2) = 0;
    
end

J_opt = J_compare(:,1);
u_opt_ind = mu_h;

J_opt(terminal_state) = 0;
u_opt_ind(terminal_state) = HOVER;

end