function ballodebounceandhalf
%BALLODEBOUNCEANDHALF  Run a demo of a bouncing ball.
%   This is an example of repeated event location, where the initial
%   conditions are changed after each terminal event.  This demo computes one 
%   bounce and terminates at the apex of the second bounce. The speed of the 
%   ball is attenuated by 0.5 after the bounce. The trajectory is plotted 
%   using the output function ODEPLOT. Modified from BALLODE
%
%   See also ODE23, ODE45, ODESET, ODEPLOT, FUNCTION_HANDLE.

%   Mark W. Reichelt and Lawrence F. Shampine, 1/3/95
%   Copyright 1984-2014 The MathWorks, Inc.

%   Changes from BALLODE made by Elizabeth
%   Additional comments added by Art Kuo

tstart = 0;
tfinal = 30;
y0 = [0; 20];  % initial condition for the ball's state, where the
               % current state is stored in vector y = [position; velocity]
refine = 4;

% The options for solving the ordinary differential equation specify 
% to turn on event detection, and to make a plot of the output
options1 = odeset('Events',@height_events,'OutputFcn',@odeplot,'OutputSel',1,...
   'Refine',refine);  
options2 = odeset('Events',@velocity_events,'OutputFcn',@odeplot,'OutputSel',1,...
   'Refine',refine);
% options1 = stop integration when height passes through 0
% options2 = stop integration when velocity passes through 0

% Following are to set up graphics figure for the plot
fig = figure;
ax = axes;
ax.XLim = [0 30];
ax.YLim = [0 25];
box on
hold on;

tout = tstart;  % tout and yout store the output time and state vector
yout = y0.';    % for each segment of simulations. A segment consists of
teout = [];     % a ball flight, terminated by a detected ground contact
yeout = [];     % event.
ieout = [];

% Simulation constants
g = -9.81;  % gravitational acceleration
m = 1;      % mass of ball
b = 0.01;      % drag coefficient
e = 0.5;    % coefficient of restitution for bouncing

% Solve the ordinary differential equation until the first terminal event.
% The state-derivative function is f; in Matlab, @f signifies a
% "function handle," which informs ode23 that f can be called as a
% function.
% The time range for the simulation is indicated by [tstart tfinal].
% Simulation's initial condition is specified as y0.
   
% first bounce
[t,y,te,ye,ie] = ode23(@f,[tstart tfinal],y0,options1);
if ~ishold
   hold on
end
% Accumulate output.  This could be passed out as output arguments.
nt = length(t);
tout = [tout; t(2:nt)];       % With each segment of simulation, 
yout = [yout; y(2:nt,:)];     % the output time and state are appended
                                 % to ever-growing tout and yout vectors.
teout = [teout; te];          % Events at tstart are never reported.
yeout = [yeout; ye];
ieout = [ieout; ie];
   
% Set the new initial conditions, with speed attenuation.
y0(1) = 0;              % After each bounce, initial position (height)
                        % is zero for the next flight phase.
y0(2) = -e*y(nt,2);     % After each bounce, take the ending velocity
                        % of the previous segment, and reverse it and
                        % multiply by the coefficient of restitution.
    
tstart = t(nt);

   
% half bounce
   
[t,y,te,ye,ie] = ode23(@f,[tstart tfinal],y0,options2);
if ~ishold
   hold on
end
% Accumulate output.  This could be passed out as output arguments.
nt = length(t);
tout = [tout; t(2:nt)];       % With each segment of simulation, 
yout = [yout; y(2:nt,:)];     % the output time and state are appended
                              % to ever-growing tout and yout vectors.
teout = [teout; te];          % Events at tstart are never reported.
yeout = [yeout; ye];
ieout = [ieout; ie];

plot(teout,yeout(:,1),'ro')
xlabel('time');
ylabel('height');
title('Ball trajectory and the events');
hold off
odeplot([],[],'done');

% --------------------------------------------------------------------------
function dydt = f(t,y)
% FUNCTION f(t,y)     State-derivative function for ball flight
%   dydt = f(t,y) returns the state-derivative dydt, given a current
%                 time t and state y. State is a two-element vector
%                 containing [verticalposition; verticalvelocity].
dydt = [y(2); (m*g-b*y(2))/m];    % dydt = [verticalvelocity; verticalacceleration]
                     % where verticalacceleration is g (downward).
end % subfunction
% --------------------------------------------------------------------------

function [value,isterminal,direction] = height_events(t,y)
% FUNCTION events(t,y)    Event function for ball flight.
%   value = events(t,y) is a zero-crossing function that triggers an event
%                       that may be used to halt simulation.
%                       value is a vector of functions, the zero-crossing
%                       of which is detected by the ODE solver.
%
%   [value, isterminal, direction] = events(t,y) can return isterminal = 1
%                       to halt simulation, or 0 to restart.
%                       direction = +1, 0, -1 specifies whether zero-crossings
%                       should be detected in positive, any, or negative
%                       directions.

% Here the event detected is ground contact.
% Locate the time when height passes through zero in a decreasing direction
% and stop integration.
value = y(1);     % detect height = 0
isterminal = 1;   % stop the integration
direction = -1;   % negative direction
end % subfunction

%------------------------------------------------------------------------

function [value,isterminal,direction] = velocity_events(t,y)
% FUNCTION events(t,y)    Event function for ball flight.
%   value = events(t,y) is a zero-crossing function that triggers an event
%                       that may be used to halt simulation.
%                       value is a vector of functions, the zero-crossing
%                       of which is detected by the ODE solver.
%
%   [value, isterminal, direction] = events(t,y) can return isterminal = 1
%                       to halt simulation, or 0 to restart.
%                       direction = +1, 0, -1 specifies whether zero-crossings
%                       should be detected in positive, any, or negative
%                       directions.

% Here the event detected is ground contact.
% Locate the time when velocity passes through zero in a decreasing direction
% and stop integration.
value = y(2);     % detect velocity = 0
isterminal = 1;   % stop the integration
direction = -1;   % negative direction
end % subfunction

end % main function