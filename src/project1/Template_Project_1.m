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
tolerance = 0.01;
reasonable_speed = 0.08;
state = 1;

%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Iterate for the previously specified number of iterations
for t = 1:iterations

    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    p = r.get_poses();
    % if t == 1
    %     init_p = p;
    % end
    %%%%%%%%%%%%%%%%%%%%%%%% Place Algorithm Here %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
    % u = ???;

    % You can try with u = [0.1; 0] and [0; 1] first. Observe what happens to
    % get a sense of how it works.

    % You should think about implementing a finite-state machine. How many
    % states are there? What are the transitions? What signals the transition
    % from one state to another?
    % consider three states
    % 1 -> 2: going to target 1 open loop
    % 2 -> 3: going to target 2 rotate
    % 3 -> 4: going to target 2 closed loop
    % 4 -> 5: going to target 1 rotate
    % 5 -> 6: going to target 1 open loop
    % 6 -> 7: going to target 2 rotate
    % 7 -> 8: going to target 2 open loop


    if state == 1
        [theta_dif, distance] = sniff(target1, p, state);
        if distance > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [reasonable_speed; turn_angle];
        else
            state = 2
            u = [0; 0];
        end
    elseif state == 2
        [theta_dif, distance] = sniff(target2, p, state);
        if abs(theta_dif) > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [0; turn_angle];
        else
            state = 3
            end_t = t + 379;
            u = [0; 0];
        end
    elseif state == 3
        if t < end_t
            u = [0.08; 0];
        else
            state = 4
            u = [0; 0];
        end
    elseif state == 4
        [theta_dif, distance] = sniff(target1, p, state);
        if abs(theta_dif) > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [0; turn_angle];
        else
            state = 5
            u = [0; 0];
        end
    elseif state == 5
        [theta_dif, distance] = sniff(target1, p, state);
        if distance > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [reasonable_speed; turn_angle];
        else
            state = 6
            u = [0; 0];
        end
    elseif state == 6
        [theta_dif, distance] = sniff(target2, p, state);
        if abs(theta_dif) > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [0; turn_angle];
        else
            state = 7
            u = [0; 0];
        end
    elseif state == 7
        [theta_dif, distance] = sniff(target2, p, state);
        if distance > tolerance
            turn_angle = get_turn_angle(theta_dif, k, state);
            u = [reasonable_speed; turn_angle];
        else
            break;
        end
    end

    % if t == 3000
    %     fail_records = [fail_records [p]]
    % end
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
function direction = get_direction(x_diff, y_diff);
    tan = y_diff/x_diff;
    direction = atan(tan);
    quadrant = get_quadrant(x_diff, y_diff);
    if quadrant == 3
        direction = direction - pi;
    elseif quadrant == 2
        direction = direction + pi;
    end
    while direction < -pi
        direction = direction + 2*pi;
    end
    while direction > pi
        direction = direction + 2*pi;
    end

end

function [theta_dif, distance] = sniff(target, p, state)
    x_diff = target(1) - p(1);
    y_diff = target(2) - p(2);
    direction = get_direction(x_diff, y_diff);
    distance = norm(p(1:2)-target(1:2));
    if state == 2
      theta_dif = get_theta_dif(0, p);
    elseif state == 4
      theta_dif = get_theta_dif(pi, p);
    else
      theta_dif = get_theta_dif(direction, p);
    end
    while theta_dif < -pi
        theta_dif = theta_dif + 2*pi;
    end
    while theta_dif > pi
        theta_dif = theta_dif - 2*pi;
    end

end

function quadrant = get_quadrant(x_diff, y_diff)
    if x_diff > 0
        if y_diff > 0
            quadrant = 1;
        else
            quadrant = 4;
        end
    else
        if y_diff > 0
            quadrant = 2;
        else
            quadrant = 3;
        end
    end
end

function theta_dif = get_theta_dif(target, p)
    theta_dif = target - p(3);
end

function turn_angle = get_turn_angle(theta_dif, k, state)
    turn_angle = k*theta_dif;
    if ismember(state, [2,5,7])
        direction = sign(theta_dif);
        turn_angle = direction*min(1, abs(k*theta_dif));
    end
end

%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
