%% Simulator Skeleton File - Project 1 

% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

close all

%% Get Robotarium object used to communicate with the robots/simulator
N = 1;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
data = [];

% Select the number of iterations for the experiment.
iterations = 3000; % Do not change

% Create a boundary barrier
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary();

% Other important variables
target1 = [-0.5; 0; 0];
target2 = [0.5; 0; 0];
k = 1;

%%%%%%%%%%%%%%%%%%%%%%%% Place Static Variables Here %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%    
% var = 0;
  
    
    
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    p = r.get_poses();

    %%%%%%%%%%%%%%%%%%%%%%%% Place Algorithm Here %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
    % u = ???;
    
    % You can try with u = [0.1; 0] and [0; 1] first. Observe what happens to
    % get a sense of how it works.
    
    % You should think about implementing a finite-state machine. How many
    % states are there? What are the transitions? What signals the transition
    % from one state to another?
    
    
    
    
    
    
    
    
    %%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Send velocities to agents
    
    % Apply the barrier to the valocities
    u = uni_barrier_certificate(u, p);
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, u); % u is the input, a 2x1 vector for 1 robot 
    
    % Send the previously set velocities to the agents.  This function must be called!
    data = [data [p; u]];
    r.step(); 
end
    
% Call r.debug() after our experiment is over!
save('data.mat', 'data');
r.debug(); % Do not modify


%%%%%%%%%%%%%%%%%%%%%%%% Place Helper Functions Here %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%

% function a = foo(b)

  
    
    
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%