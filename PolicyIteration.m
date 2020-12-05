function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
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
% Do you need to do something with the terminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

terminal_state = TERMINAL_STATE_INDEX;

% Initialization of optimal cost and policy
J_opt = zeros(K,1);
u_opt_ind = zeros(K,1);

% Since I will iterate the procedure above all i except for the terminal
% state, I set these values now: 
% the cost to go from starting at the terminal state is zero
% the optimal input is HOVER
J_opt(terminal_state) = 0;
u_opt_ind(terminal_state) = HOVER;

%% INITIALIZATION
% initialization with a proper policy
mu_h = zeros(K,1);

for i = 1:K
    if i ~= terminal_state
        mu_h(i) = HOVER;
    end
end

% This matrix will compare the costs of the current and the previous
% iteration
J_compare = zeros(K,2);

stop = 0;

% start of the Policy Iteration procedure
while (stop < K-1)

    % POLICY EVALUATION
    
    % Defining matrix A. Non admissible inputs and terminal state are not
    % considered.
    % A is built iteratively line by line

    A = [];

    for i = 1 : K

        if(i ~= TERMINAL_STATE_INDEX) 

            if(G(i,mu_h(i)) == Inf)  % checking admissibility of input

                %do nothing

            else  % if input is admissible then I add a line to A

                P_vector = [];

                for k = 1 : K  % filling a line of A

                    if(k ~= TERMINAL_STATE_INDEX) 

                        if(k == i)  % diagonal

                            P_vector = [P_vector,1 - P(i,k,mu_h(i))];

                        else

                            P_vector = [P_vector, - P(i,k,mu_h(i))];

                        end

                    end

                end

                A = [A;
                    P_vector];

            end

        end

    end
    
    % Defining vector Q which is the vectorization of stage costs matrix without
    % non admissible inputs and terminal state

    Q = [];

    for i = 1 : K

        if(i == TERMINAL_STATE_INDEX || G(i,mu_h(i)) == Inf) 

            %do nothing

        else

            Q = [Q;
                G(i,mu_h(i))];

        end

    end 

    J_sol = linsolve(A,Q);


    k=1;

    for i = 1 : K

        if( i ~= TERMINAL_STATE_INDEX)

            J_compare(i,2) = J_sol(k,1);

            k = k + 1;

        end

    end
   
    % POLICY IMPROVEMENT
    
    for i = 1:K
        if i ~= terminal_state
            
            J_to_min = zeros(5,1);

            for u = 1:5

                Prob2 = 0;
                
                for j = 1:K

                    Prob2 = Prob2 + P(i,j,u)*J_compare(j,2); 

                end

                J_to_min(u) = G(i,u) + Prob2;
            end

            J_min = Inf;

            for u = 1:5

                if (J_to_min(u) < J_min)

                    J_min = J_to_min(u);
                    mu_h(i) = u;
                end
            end

        end
    end

    % at the end of this for loop I get a new policy

    % I need to verify that for each state i the optimal costs to go are the
    % same between he current iteration and the previous one. I do the 
    % comparison for all states i except for the terminal state
    stop = 0;
    for i = 1:K
        if i ~= terminal_state
            if (abs(J_compare(i,1) - J_compare(i,2)) < 0.00001)
                stop = stop + 1;
            end
        end
    end
    
    % I now move the second column to the first, deleting the values of the previous iteration.
    % I set the second column to zero, and this will be filled in the next iteration of the while condition 
    for i = 1:K
        if i ~= terminal_state
            J_compare(i,1) = J_compare(i,2);
            J_compare(i,2) = 0;
        end
    end
    
end

% At the end of the while condition the matrix J_compare will have on the second column 
% the optimal cost-to-go 
for i = 1:K
    if i ~= terminal_state
        J_opt(i) = J_compare(i,1);
    end
end

% The vector u_opt_ind has been updated in the last iteration on the while
% condition
u_opt_ind = mu_h;

end