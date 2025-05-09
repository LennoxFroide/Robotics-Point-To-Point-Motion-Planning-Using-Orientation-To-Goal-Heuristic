clear;clc;
% Setting up the connection between client(MATLAB script) and
% Server(CoppeliaSim scenario)
[clientID,currentScenario] = setUpClientServerConnection();
% Instantiating the robot and getting the start and goal coordinates
robot = robotNamespace.RobotClass(clientID,currentScenario);
% Retrieving the robot handle
pioneerRobot = robot.robotHandle;
% Setting the Xstart and Xgoal for the mobile robot
% Instantiating the obstacles and getting their user defined coordinates
obstacles = robotNamespace.ObstacleClass(clientID,currentScenario);
% Getting the obstacle coordinates
% obstacles.getObstacleCoordinates();
obstacles.cuboidList = obstacles.getObstacleCoordinates('Cuboid');
obstacles.cylinderList = obstacles.getObstacleCoordinates('Cylinder');
obstacles.sphereList = obstacles.getObstacleCoordinates('Sphere');
% Getting obstacle handles
cuboidHandle = obstacles.getObstacleHandle('Cuboid');
cylinderHandle = obstacles.getObstacleHandle('Cylinder');
sphereHandle = obstacles.getObstacleHandle('Sphere');
% Setting the position of the obstacles according to the user's inputs
firstobstaclePositionsSet = obstacles.setObstacleAtUserPositions('Cuboid');
secondobstaclePositionsSet = obstacles.setObstacleAtUserPositions('Cylinder');
thirdobstaclePositionsSet = obstacles.setObstacleAtUserPositions('Sphere');
% Sanity check
if firstobstaclePositionsSet && secondobstaclePositionsSet && thirdobstaclePositionsSet
    pause(0.005)
else
    return
end
% TEST
[robotPosition,robo1] = robot.scenarioName.simxGetObjectPosition(robot.clientId,pioneerRobot,-1,robot.scenarioName.simx_opmode_blocking);
% Setting the position of the obstacles according to the user's inputs
% Getting the left and right motor handles to control the motion of the
% mobile robot
% This is achieved by instantiating the Navigation class
navigation = robotNamespace.NavigationClass(clientID,currentScenario);
% Getting the handles of the proximity sensors to be used in collision
% avoidance
[navigation.firstSensor,navigation.secondSensor,navigation.thirdSensor,navigation.fourthSensor] = navigation.getProximitySensorHandles('Pioneer_p3dx_ultrasonicSensor2','Pioneer_p3dx_ultrasonicSensor4','Pioneer_p3dx_ultrasonicSensor6','Pioneer_p3dx_ultrasonicSensor8');
% [firstSensor,secondSensor,thirdSensor,fourthSensor] = getProximitySensorHandles('Pioneer_p3dx_ultrasonicSensor2','Pioneer_p3dx_ultrasonicSensor4','Pioneer_p3dx_ultrasonicSensor6','Pioneer_p3dx_ultrasonicSensor8',clientID,currentScenario);
% Getting the current robot position
[returnCode,robotPositionCurrent] = robot.scenarioName.simxGetObjectPosition(robot.clientId,pioneerRobot,-1,robot.scenarioName.simx_opmode_blocking);
% [returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
% Orienting the mobile robot in the direction of the goal
[xDirection,yDirection] = navigation.goalDirection(robotPositionCurrent,robot.goal);
% [xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
% Getting direction robot is facing
headDirection = navigation.getDirectionRobotIsFacing(pioneerRobot);
% headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
% Running the obstacle avoidance simulation
% Re-orient robot position based on goal and its current position
returnCode = navigation.orientRobotTowardsGoal([xDirection,yDirection],headDirection,pioneerRobot);
% returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,pioneerRobot);
% Enabling proximity sensors/ Setting them to streaming
[returnCode, firstdetectionState,detectedPoint] = navigation.enableProximitySensors('first');
[returnCode, seconddetectionState,detectedPoint] = navigation.enableProximitySensors('second');
[returnCode, thirddetectionState,detectedPoint] = navigation.enableProximitySensors('third');
[returnCode, fourthdetectionState,detectedPoint] = navigation.enableProximitySensors('fourth');
% Starting the journey towards the goal
while(1)
    % Adding velocity to both motors
    [jointReturn] = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.leftMotor,0.5,navigation.scenarioName.simx_opmode_blocking);
    [jointReturn]  = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.rightMotor,0.5,navigation.scenarioName.simx_opmode_blocking);
    %{
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.5,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.5,currentScenario.simx_opmode_blocking);
    %}
    pause(1.5);
    % Breaking
    [jointReturn] = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.leftMotor,0.0,navigation.scenarioName.simx_opmode_blocking);
    [jointReturn]  = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.rightMotor,0.0,navigation.scenarioName.simx_opmode_blocking);
    %{
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.0,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.0,currentScenario.simx_opmode_blocking);
    %}
    reachedGoal = robot.checkIfAtGoalPosition(robot.goal);
    % reachedGoal = checkIfAtGoalPosition(goalPosition,currentScenario,clientID,pioneerRobot);
    if reachedGoal
       break;
    end
    % Checking if we've detected obstacle
    [jointReturn] = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.leftMotor,0.5,navigation.scenarioName.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(navigation.clientId,navigation.rightMotor,0.5,navigation.scenarioName.simx_opmode_blocking);
    [obstacleDetected,obstacleLocation] = robot.checkForObstacles(navigation.firstSensor,navigation.secondSensor, navigation.thirdSensor,navigation.fourthSensor);
    %{
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.5,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.5,currentScenario.simx_opmode_blocking);
    [obstacleDetected,obstacleLocation] = checkForObstacles(firstSensor,secondSensor,thirdSensor,fourthSensor,clientID,currentScenario);
    %}
    if obstacleDetected
        obstacleAvoided = robot.avoidObstacles(navigation.leftMotor,navigation.rightMotor,obstacleLocation);
        % obstacleAvoided = avoidObstacles(leftMotor,rightMotor,currentScenario,clientID,obstacleLocation,pioneerRobot);
        if obstacleAvoided
            reachedGoal = robot.checkIfAtGoalPosition(robot.goal);
            % reachedGoal = checkIfAtGoalPosition(goalPosition,currentScenario,clientID,pioneerRobot);
            if reachedGoal
                break;
            else
                % Getting the current robot position
                [returnCode,robotPositionCurrent] = robot.scenarioName.simxGetObjectPosition(robot.clientId,pioneerRobot,-1,robot.scenarioName.simx_opmode_blocking);
                % [returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
                % Orienting the mobile robot in the direction of the goal
                [xDirection,yDirection] = navigation.goalDirection(robotPositionCurrent,robot.goal);
                % [xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
                % Getting direction robot is facing
                headDirection = navigation.getDirectionRobotIsFacing(pioneerRobot);
                % headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
                % Re-orient robot position based on goal and its current position
                returnCode = navigation.orientRobotTowardsGoal([xDirection,yDirection],headDirection,pioneerRobot);
                % returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,clientID,pioneerRobot,leftMotor,rightMotor,currentScenario);
                continue;
            end
        end
    end
    % After moving for a while we have to reorient ourselves towards the
    % goal direction
    [jointReturn] = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.leftMotor,0.0,navigation.scenarioName.simx_opmode_blocking);
    [jointReturn]  = navigation.scenarioName.simxSetJointTargetVelocity(navigation.clientId,navigation.rightMotor,0.0,navigation.scenarioName.simx_opmode_blocking);
    %{
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.0,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.0,currentScenario.simx_opmode_blocking);
    %}
    % Orienting the mobile robot in the direction of the goal
    % Getting the current robot position
    [returnCode,robotPositionCurrent] = robot.scenarioName.simxGetObjectPosition(robot.clientId,pioneerRobot,-1,robot.scenarioName.simx_opmode_blocking);
    [xDirection,yDirection] = navigation.goalDirection(robotPositionCurrent,robot.goal);
    %{
    [returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
    [xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
    %}
    % Getting direction robot is facing
    headDirection = navigation.getDirectionRobotIsFacing(pioneerRobot);
    % headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
    % Re-orient robot position based on goal and its current position
    returnCode = navigation.orientRobotTowardsGoal([xDirection,yDirection],headDirection,pioneerRobot);
    % returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,clientID,pioneerRobot,leftMotor,rightMotor,currentScenario);
    continue;
end
% Setting the proximity sensorts to streaming
% runCoppeliaSimulation();
disp("We're at the goal location!!!")
%--------------Helper function to set up client-server connection-------%
function [clientId,scenario] = setUpClientServerConnection()
    % Coppelia Sim remote server
    scenario = remApi('remoteApi');
    % Closing any pre-existing connections
    scenario.simxFinish(-1);
    % Grabbing the client ID returned by remote server->CoppeliaSim
    clientId = scenario.simxStart('127.0.0.1',19999,true,true,5000,5);
    % We have a valid client Id
    if clientId>-1
        disp("Client-server connection established!");
    end
end