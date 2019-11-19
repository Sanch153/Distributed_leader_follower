%% Main Leader Controller (MODIFY ONLY THE INTERIOR OF THIS FUNCTION!)
function [ui,auxFlags] = nodeLevelLeaderController(xi,xj,xT,di,Delta,delta,auxFlags)
%%%%%% Modify this function to avoid collisions and remain connected %%%%%%
% Inputs:
%   -xi         : 2-by-1 position of current agent
%   -xj         : 2-by-|Ni| matrix of neighbor positions
%   -xT         : 2-by-1 vector of current target position
%   -di         : 1-by-|Ni| vector of neighbor desired separation
%   -Delta      : Maximum interaction distance
%   -delta      : Minimum safety separation
%   -auxFlags   : 1-by-|Ni| vector of auxiliary flags
% Outputs:
%   -ui         : 2-by-1 velocity reference for current agent
%   -auxFlags   : 1-by-|Ni| vector of auxiliary flags
% 
% Note: The auxiliary flags argument is initially all zeros and gets
% overwritten after each call, so you may use it as a piece of persistent
% memory should you need it.
ui = [0;0];
for jj = 1:size(xj,2)
    norma = norm(xj(:,jj) - xi);
    delta = delta + 0.05;
    dij = di(jj);
    if norma >=  Delta - 0.1
        h = 0;
    else
        h = 1;
    end
    if dij == 0
        %wij = 0.01*h*(norma - delta)/(norma*((Delta - norma)^3));
        wij = (norma-delta)/norma^2;
    else
        %wij = 0.01*h*(norma - delta)/(norma*((Delta - norma)^3));
        wij = (norma-delta)/norma^2;
    end
    %ui = ui + (xi - xT)
    ui = ui + 0.01*wij*(xj(:,jj) - xi);
end

if norm(xT-xi) <= 0.4
    ui = ui + 1*(xT-xi);
else
ui = ui + 0.1*(xT-xi);
end
%%%%%% Modify this function to avoid collisions and remain connected %%%%%%
end