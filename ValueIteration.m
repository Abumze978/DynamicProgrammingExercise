function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
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

% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)


%HO TOLTO UN % RIGA 32 PERCHè VENIVA TUTTO GIALLO E MI SCOCCIAVA :)

J_opt = zeros(K,1);
u_opt_ind = zeros(K,1);

%vettore dei costi Vl, sarà una matrice K x l, dove l indica il numero di
%iterazioni che faremo 
V = [];

%inizializzo V0 (credo vada bene metterli tutti a zero...)
V0 = zeros(5,1);

for i = 1 : K
    
    %per tenere traccia dei 5 valori che otterrò durante la minimizzazione
    Vmin = zeros(5,1);
   
    for u = 1 : 5
        
        Prob = 0;
       
        for j = 1 : K 
           
            Prob = Prob + P(i,j,u)*V(j);
            
        end
        
    end
    
end



end