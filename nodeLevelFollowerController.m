%% Main Follower Controller (MODIFY ONLY THE INTERIOR OF THIS FUNCTION!)
function [ui,auxFlags] = nodeLevelFollowerController(xi,xj,di,Delta,delta,auxFlags)
%%%%%% Modify this function to avoid collisions and remain connected %%%%%%
% Inputs:
%   -xi         : 2-by-1 position of current agent
%   -xj         : 2-by-|Ni| matrix of neighbor positions
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
    if norma > Delta - 0.1
            h = 0;
        else
            h = 1;
    end
    dij = di(jj);
    if dij == 0
        wij = h*(norma - delta)/(norma*((Delta - norma)^3));
    else
        numera = h*(norma - dij)*(norma*((Delta - dij) - (dij - delta)) + dij*(Delta + delta) - 2*Delta*delta);
        denomin = norma*((Delta - norma)^2)*(norma - delta)^2;
        wij = (numera/denomin);
    end
    ui = ui + 0.03*wij*(xj(:,jj) - xi);
end
%%%%%% Modify this function to avoid collisions and remain connected %%%%%%
end