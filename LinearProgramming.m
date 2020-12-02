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


%Defining vector f without terminal state
f = [];

for i = 1 : K

     if(i ~= TERMINAL_STATE_INDEX)

            f = [f;-1];
             
     end
         
end



%Defining matrix A built on 5 layers, one for each input. Non admissible
%inputs and terminal state are not considered
%A is built iteratively line by line

A = [];

for u = 1 : 5
    
    for i = 1 : K
        
        if(i ~= TERMINAL_STATE_INDEX) 

            if(G(i,u) == Inf)  %check if it's an admissible input

                %do nothing

            else %if input is admissible then I add a line to A

                P_vector = [];

                for k = 1 : K  %filling a line of A
                    
                    if(k ~= TERMINAL_STATE_INDEX) 
                        
                        if(k == i)  %diagonal

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

%Defining vector Q which is the vectorization of stage costs matrix without
%non admissible inputs and terminal state

Q = [];

for u = 1 : 5
    
    for i = 1 : K
        
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