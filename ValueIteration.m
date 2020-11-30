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

%Since I will iterate the procedure above all i except for the terminal
%state, I set these values now: 
%the cost to go from starting at the terminal state is zero
%the optimal input is HOVER
J_opt1(terminal_state) = 0;
u_opt_ind1(terminal_state) = HOVER;

%This matrix will compare the values of the current and the previous
%iteration, all previous iterations will not be recorded, since are useless
V = zeros(K,2);

%Arbitrary initialization of V at iteration 0
V(:,1) = 0;

%This variable will have value K-1 when the Value Iteration process will be
%finished
finish = 0;

%Until the Value Iteration process is not termined 
while (finish < K-1)
    
    for i = 1 : K
        
        %I perform the VI process for all the states except of the terminal state
        if (i ~= terminal_state)
        
            %This array will memorize all costs at a certain iteration
            V_u = zeros(5,1);

            %I calculate the cost connected to using a certain input
            for u = 1 : 5

                Prob = 0;

                for j = 1 : K

                    Prob = Prob + P(i,j,u)*V(j,1);

                end

                V_u(u) = G(i,u) + Prob;

            end

            Vmin = Inf;

            %I look for the minimum value of V_u array
            for u = 1 : 5
                
                %But I need to consider only the possible inputs, that are
                %the ones with an expected cost less than infinity 
                if G(i,u) < Inf

                    %I compute the minimum
                    if (V_u(u) < Vmin)

                        Vmin = V_u(u);
                        
                        %Assignment of the optimal insput, consistentely
                        %with the optimal cost 
                        u_opt_ind1(i) = u;

                    end
                end
            end

            %The optimal cost of the current iteration
            V(i,2) = Vmin;

        end
    end
    
    %I need to verify that for each state i the optimal costs to go are the
    %same between he current iteration (second coloum on V) and the
    %previous one. I do the comparison for all states i except for the
    %terminal state
    finish = 0;
    for i = 1 : K
        
        if (i ~= terminal_state)
        
            if (abs(V(i,1) - V(i,2)) < 0.00001)

                finish = finish + 1;

            end
        
        end
        
    end
    
    %If values are different between the two most recent iteration I will
    %move the secondo coloumn to the first, deleting the values of the
    %previous iteration, that will be useless from now on. The current
    %second coloumn is made up of zeros, will be filled in the next
    %iteration of the while condition 
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

%The vector u_opt_ind has been updated in the last iteration on the while
%condition
u_opt_ind = u_opt_ind1;


end