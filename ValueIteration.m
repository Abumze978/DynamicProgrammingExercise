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

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

terminal_state = TERMINAL_STATE_INDEX;

%Initialization of optimal cost and policy
J_opt1 = zeros(K,1);
u_opt_ind1 = zeros(K,1);

%Cost to go from terminal state = 0
%Optimal input at terminal state = HOVER
J_opt1(terminal_state) = 0;
u_opt_ind1(terminal_state) = HOVER;

%This matrix compares the values of the current and the previous
%iteration, all previous iterations are discarded
V = zeros(K,2);

%Arbitrary initialization of V
V(:,1) = 0;

%This variable will have value K-1 when the Value Iteration process will be
%finished
finish = 0;

%Value Iteration process
while (finish < K-1)
    
    for i = 1 : K
        
        %VI is applied to all states except for the terminal state
        if (i ~= terminal_state)
        
            %This array memorizes all costs at a certain iteration relative
            %to a specific input
            V_u = zeros(5,1);

            %Cost resulting from using a certain input
            for u = 1 : 5

                Prob = 0;

                for j = 1 : K

                    Prob = Prob + P(i,j,u)*V(j,1);

                end

                V_u(u) = G(i,u) + Prob;

            end

            Vmin = Inf;

            %Minimum value of V_u array
            for u = 1 : 5
                
                %Only admissible inputs has to be considered. Correspond to
                %the ones with an expected cost less than infinity 
                if G(i,u) < Inf

                    %Calculate the minimum
                    if (V_u(u) < Vmin)

                        Vmin = V_u(u);
                        
                        %Assignment of relative optimal input
                        u_opt_ind1(i) = u;

                    end
                end
            end

            %Optimal cost of the current iteration
            V(i,2) = Vmin;

        end
    end
    
    %Check needed to verify that for each state i the optimal costs to go is the
    %same between current and previous iteration.
    %Comparison is done for all states i except for the terminal state
    finish = 0;
    for i = 1 : K
        
        if (i ~= terminal_state)
        
            if (abs(V(i,1) - V(i,2)) < 0.00001)

                finish = finish + 1;

            end
        
        end
        
    end
    
    %If values are different between the two most recent iteration, s
    %second column is moved to the first, descarding values of the
    %previous iteration. The current second column is made up of zeros
    %and is filled during the next iteration
    if (finish < K-1)
       
        for i = 1 : K
            
            if (i ~=terminal_state)
           
                V(i,1) = V(i,2);
                V(i,2) = 0;
            
            end
            
        end
        
    end
   
end


%At the end of the while condition the matrix V will have on the second coloumn 
%the optimal cost to go 
for i = 1 : K

    if (i ~= terminal_state)

        J_opt1(i) = V(i,2);

    end

end


J_opt = J_opt1;

% Vector u_opt_ind1 has been updated in the last iteration of the while
% condition, hence corresponds to the optimal inputs
u_opt_ind = u_opt_ind1;


end