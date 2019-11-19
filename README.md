# Distributed_leader_follower
Simulation of a distributed control algorithm to drive a multi-agent system with a single leader (has knowledge about target locations) to the target positions with collision avoidance.  
-The nodeLevelFollowerController.m gives the control law for the followers    
-The nodeLevelLeaderController.m gives the control law for the leader    
-The LeaderFollowerTarget.m file needs to be run to see the simulation, currently it is set with the parameters of network choice-2, this can be changed in the code of this file. To make the same controller run for other network topologies, the user must play and experiment with the constants in the control law.    
