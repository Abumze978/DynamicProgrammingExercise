% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2020
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
%
% --

%% Clear workspace and command window
clear all;
close all;
clc;

%% Options
% [M, N]
mapSize = [15, 20];
% Set to true to generate a random map of size mapSize, else set to false 
% to load the pre-exsisting example map
generateRandomWorld = false;

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
% IMPORTANT: Do not add or remove any global parameter in main.m
global GAMMA R Nc P_WIND
GAMMA  = 0.2; % Shooter gamma factor
R = 2; % Shooter range
Nc = 10; % Time steps required to bring drone to base when it crashes
P_WIND = 0.1; % Gust of wind probability

% IDs of elements in the map matrix
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE 
FREE = 0;
TREE = 1;
SHOOTER = 2;
PICK_UP = 3;
DROP_OFF = 4;
BASE = 5;

% Index of each action in the P and G matrices. Use this ordering
global NORTH SOUTH EAST WEST HOVER
NORTH  = 1;
SOUTH = 2;
EAST = 3;
WEST = 4;
HOVER = 5;

%% Generate map
% map(m,n) represents the cell at indices (m,n) according to the axes
% specified in the PDF.
disp('Generate map');
if generateRandomWorld
	[map] = GenerateWorld(mapSize(1), mapSize(2));
else
    % We can load a pre-generated map.
    load('exampleWorld.mat');
end
MakePlots(map);

%% Generate state space
disp('Generate state space');
% Generate a (K x 3)-matrix 'stateSpace', where each accessible cell is
% represented by two rows (with and without carrying a package).
stateSpace = [];
for m = 1 : size(map, 1)       %numero di righe(che in verità son le colonne: girare pc per capire!)
    for n = 1 : size(map, 2)   %numero di colonne
        if map(m, n) ~= TREE    %se NON è un albero
            stateSpace = [stateSpace;   %se è un albero stateSpace rimane invariata
                          m, n, 0;
                          m, n, 1];
        end
    end
end
% State space size
global K
K=size(stateSpace,1); %numero di righe della stateSpace

%% Set the following to true as you progress with the files
transitionProbabilitiesImplemented = true;
stageCostsImplemented = false;
valueIterationImplemented = false; 
policyIterationImplemented = false;
linearProgrammingImplemented = false;

%% Compute the terminal state index
global TERMINAL_STATE_INDEX
if transitionProbabilitiesImplemented
    % TODO: Question a)
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end 
%% Compute transition probabilities
if transitionProbabilitiesImplemented
    disp('Compute transition probabilities');
    % Compute the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.
    
    % TODO: Question b)
    P1 = ComputeTransitionProbabilities_def(stateSpace, map);
    
 load('example_P.mat');
 

%questo pezzo di codice mi stampa prima la probabilità di andare in base
%calcolata da un generico i con un generico u dell'esempio e poi quella
%calcolata dal nostro algoritmo. Ad ogni iterazione ci accorgiamo che
%quando ci sono calcoli da fare il nostro algoritmo ci azzecca ma a volte
%mette degli 1 quando ci sarebbero degli zero

% contatore = 0;
%     for i = 1 : K
%             
%             for u = 1 : 5
%                 
%                 disp(P(i,137,u));
%                 disp(P1(i,137,u));
%                 contatore = contatore + 1;
%         
%             end
%             
%     end
%         
%     disp(contatore);
end

 counter = 0;
 errors = [];
    
     for u = 1 : 5
        
        for i = 1 : K
            
            for j = 1 : K
                
                if(P1(i,j,u) ~= P(i,j,u)) % && mod(i,2) ~= 0 && i ~= size(map,1) && j ~= size(map,2) )
                    
                    counter = counter + 1;
                    errors = [errors ;
                               i,j,u];
                    
%                     disp('noi');
%                     P1(i,j,u) 
%                     disp('loro');
%                     P(i,j,u)
                    
                end
            end
        end
     end
    
%     disp('errors = ');
%     disp(errors);
    disp('num errori ancora presenti = ');
    disp(counter);

     
disp('loro');
P(3,137,SOUTH)
disp('noi');
P1(3,137,SOUTH)



%% Compute stage costs
if stageCostsImplemented 
    disp('Compute stage costs');
    % Compute the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
    
    % TODO: Question c)
    G = ComputeStageCosts(stateSpace, map);
end

%% Solve stochastic shortest path problem
% Solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming
if valueIterationImplemented
    disp('Solve stochastic shortest path problem with Value Iteration');
    
    % TODO: Question d)
    [ J_opt_vi, u_opt_ind_vi ] = ValueIteration(P, G);
    
    if size(J_opt_vi,1)~=K || size(u_opt_ind_vi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if policyIterationImplemented
    disp('Solve stochastic shortest path problem with Policy Iteration');
    
    % TODO: Question d)
    [ J_opt_pi, u_opt_ind_pi ] = PolicyIteration(P, G);
    
    if size(J_opt_pi,1)~=K || size(u_opt_ind_pi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if linearProgrammingImplemented
    disp('Solve stochastic shortest path problem with Linear Programming');
    
    % TODO: Question d)
    [ J_opt_lp, u_opt_ind_lp ] = LinearProgramming(P, G);
    
    if size(J_opt_lp,1)~=K || size(u_opt_ind_lp,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end

%% Plot results
disp('Plot results');
if valueIterationImplemented
    MakePlots(map, stateSpace, J_opt_vi, u_opt_ind_vi, 'Value iteration');
end
if policyIterationImplemented
    MakePlots(map, stateSpace, J_opt_pi, u_opt_ind_pi, 'Policy iteration');
end
if linearProgrammingImplemented
    MakePlots(map, stateSpace, J_opt_lp, u_opt_ind_lp, 'Linear programming');
end

%% Terminated
disp('Terminated');
