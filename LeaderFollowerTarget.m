%% Target Tracking Over Leader-Follower Networks
% Executes decentralized controllers to track a target pose given
% formation topologies while avoiding collisions and loss of connectivity.
% The leader node is assumed to be the last agent, the only agent aware of
% the target.
%% Network Parameters
networkChoice = 2; % Choices are 1, 2, or 3.
Delta = 1;  % Maximum interaction distances
desiredSep = 0.6;  % desired formation distances
delta = 0.1; % Minimum safety distance
%% Load Network
[numAgents,robotInit,A_formation] = loadNetwork(networkChoice,Delta,desiredSep);
% Initial Delta-disk graph
[A,edgeIndices,pairwiseDist] = deltaDisk(robotInit,numAgents,Delta);
%% Taget positions
targetDist = 2*Delta;
targets = targetDist*[1 1;1 -1;-1 -1; -1 1];
%% Visualization Elements
hFigure = figure('Color','white','NumberTitle','off');
colorMat = 0.8*hsv(2);
colorMat = [repmat(colorMat(1,:),numAgents-1,1);colorMat(2,:)];
hold on
hDomain = rectangle('Position',targetDist*[-1 -1 2 2],'LineWidth',2);
hTarget = plot(targets(1,1),targets(1,2),'ko','MarkerSize',20,'LineWidth',2,'MarkerFaceColor',[0 0.5 0]);
hEdge = plot(...
    reshape([robotInit(1,edgeIndices(:,1));robotInit(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))],[],1),...
    reshape([robotInit(2,edgeIndices(:,1));robotInit(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))],[],1),...
    'k','LineWidth',2);
hRobots = scatter(robotInit(1,:),robotInit(2,:),100,colorMat,'filled');
axis equal off
% xlim(1.1*targetDist*[-1 1])
% ylim(1.1*targetDist*[-1 1])
%% Main Loop
dt = 0.01;
maxIter = 1e8;
robotPosition = robotInit;
auxFlags = zeros(numAgents);
currentTargetId = 1;
currentTarget = targets(currentTargetId,:).';
for kk = 1:maxIter
    % Compute the control at the node-level
    u = zeros(2,numAgents);
    % Follower Dynamics
    for ii = 1:numAgents-1
        % Inputs: agent state, neighbor states, neighbor formation spec,
        % sensing range, collision safety dist, and neighbor auxiliary flag
        [u(:,ii),auxFlags(ii,A(ii,:))] = nodeLevelFollowerController(robotPosition(:,ii),robotPosition(:,A(ii,:)),A_formation(ii,A(ii,:)),Delta,delta,auxFlags(ii,A(ii,:)));
    end
    % Leader Dynamics. Inputs: agent state, neighbor states, neighbor
    % formation spec, current target position, sensing range, collision
    % safety dist, and neighbor auxiliary flag
    [u(:,numAgents),auxFlags(numAgents,A(numAgents,:))] = nodeLevelLeaderController(robotPosition(:,numAgents),robotPosition(:,A(numAgents,:)),currentTarget,A_formation(numAgents,A(numAgents,:)),Delta,delta,auxFlags(numAgents,A(numAgents,:)));
    % Update the states
    robotPosition = robotPosition + dt*u;
    % Update graph topology
    [A,edgeIndices,pairwiseDist] = deltaDisk(robotPosition,numAgents,Delta);
    % Check if target reached
    if norm(currentTarget-robotPosition(:,numAgents))<delta
        if currentTargetId<=size(targets,1)
            currentTargetId = currentTargetId+1;
            if currentTargetId <= size(targets,1)
                currentTarget = targets(currentTargetId,:).';
                set(hTarget,'XData',currentTarget(1),'YData',currentTarget(2))
            end
        end
    end
    % Update the graphics
    set(hRobots,'XData',robotPosition(1,:),'YData',robotPosition(2,:))
    set(hEdge,'XData',reshape([robotPosition(1,edgeIndices(:,1));robotPosition(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))],[],1),...
        'YData',reshape([robotPosition(2,edgeIndices(:,1));robotPosition(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))],[],1))
    drawnow limitrate
    pause(dt)
    % Check for collisions
    if any(pairwiseDist<delta)
        disp('Collision Detected! Terminating...')
        return
    end
    % Check for stationarity
    if all(vecnorm(u)<5e-3*Delta)
        % Check for connectivity
        L = diag(sum(A,2)) - A;
        lambda = eigs(L,2,'smallestabs');
        if lambda(2)<1e-6
            disp('Agents have converged: Connectivity lost!')
            return
        end
        % Graph connected, check if all targets reached
        if currentTargetId>4
            disp('Agents have converged: All targets visited!')
        else
            disp('Agents have converged: Not all targets visited!')
        end
        return
    end
end
disp('Max number of iterations reached. Terminating...')
%% Auxiliary Functions
function [numAgents,robotInit,A_formation] = loadNetwork(networkChoice,Delta,desiredSep)
% Choose between hardcoded scenarios
switch networkChoice
    case 1
        numAgents=5;
        Delta=(2/3*desiredSep+1/3*Delta)/2;
        robotInit = zeros(2,numAgents);
        robotInit(:,1)=[-Delta;-Delta];
        robotInit(:,2)=[Delta;-Delta];
        robotInit(:,4)=[-Delta;Delta];
        robotInit(:,5)=[Delta;Delta];
        centerNode = 3;
        A_formation = zeros(numAgents);
        A_formation(1,[2 4 5]) = 1;
        A_formation(2,[3 5]) = 1;
        A_formation(3,[4 5]) = 1;
        A_formation(4,5) = 1;
        A_formation = A_formation + A_formation.';
        % Desired Sep Weights
        A_formation(setdiff(1:numAgents,centerNode),setdiff(1:numAgents,3)) = desiredSep*A_formation(setdiff(1:numAgents,centerNode),setdiff(1:numAgents,centerNode));
        A_formation(centerNode,setdiff(1:numAgents,centerNode)) = sqrt(2)/2*desiredSep*A_formation(centerNode,setdiff(1:numAgents,centerNode));
        A_formation(setdiff(1:numAgents,centerNode),centerNode) = sqrt(2)/2*desiredSep*A_formation(setdiff(1:numAgents,centerNode),centerNode);
    case 2
        numAgents=6;
        Delta=0.33*Delta+0.67*desiredSep;
        thetaParam = (0:2*pi/numAgents:2*pi)*eye(numAgents+1,numAgents);
        robotInit = Delta*[cos(thetaParam);sin(thetaParam)];
        A_formation = diag(ones(numAgents-1,1),1);
        A_formation(1,numAgents) = 1;
        A_formation = A_formation + A_formation.';
        A_formation = desiredSep*A_formation;
    case 3
        numAgents=7;
        Delta=0.33*Delta+0.67*desiredSep;
        thetaParamX = (3*pi:-3*pi/(numAgents-1):0);
        thetaParamY = (7*pi/4:-3/2*pi/(numAgents-1):pi/4);
        robotInit = Delta*[cos(thetaParamX);sin(thetaParamY)];
        A_formation = diag(ones(numAgents-1,1),1);
        A_formation = A_formation + A_formation.';
        A_formation = desiredSep*A_formation;
    otherwise
        error('Network Choices are ''1'', ''2'', or ''3''')
end
end
% Obtain delta-disk graph adjacency information
function [A,edgeIndices,pairwiseDist] = deltaDisk(robotStates,numAgents,Delta)
pairwiseDist = pdist(robotStates.');
A = squareform(pairwiseDist)<Delta;
A = (A - diag(diag(A)))~=0;
numEdges = sum(A,'all')/2;
edgeIndices = zeros(numEdges,2);
edgeCounter = 1;
for ii = 1:numAgents-1
    for jj = ii+1:numAgents
        if A(ii,jj) == 1
            edgeIndices(edgeCounter,1) = ii;
            edgeIndices(edgeCounter,2) = jj;
            edgeCounter = edgeCounter + 1;
        end
    end
end
end