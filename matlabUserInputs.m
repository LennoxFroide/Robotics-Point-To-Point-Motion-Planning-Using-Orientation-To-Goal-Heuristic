clear;clc;
% Setting up the connection between client(MATLAB script) and
% Server(CoppeliaSim scenario)
[clientID,currentScenario] = setUpClientServerConnection();
% Getting all of the user defined coordinates
startPosition = getStartCoordinates();
goalPosition = getGoalCoordinates();
[cuboidPosition,spherePosition,cylinderPosition] = getObstacleCoordinates();
% Getting the robot handle
pioneerRobot = getRobotHandle(clientID,currentScenario);
% Setting the Xstart and Xgoal for the mobile robot
robotStartAndGoalPositionSet = setRobotStartAndGoalPositions(startPosition,goalPosition,pioneerRobot,currentScenario,clientID);
% TEST
[robotPosition,robo1] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
% Setting the position of the obstacles according to the user's inputs
obstaclePositionsSet = setObstacleAtUserPositions(cuboidPosition,cylinderPosition,spherePosition,clientID,currentScenario);
% Getting the left and right motor handles to control the motion of the
% mobile robot
[leftMotor,rightMotor] = getMotorHandles('Pioneer_p3dx_leftMotor','Pioneer_p3dx_rightMotor',clientID,currentScenario);
% Testing how to move robot in the reverse direction
%{
[jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,-0.5,currentScenario.simx_opmode_blocking);
[jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,-0.5,currentScenario.simx_opmode_blocking);
%}
% Getting the handles of the proximity sensors to be used in collision
% avoidance
[firstSensor,secondSensor,thirdSensor,fourthSensor] = getProximitySensorHandles('Pioneer_p3dx_ultrasonicSensor2','Pioneer_p3dx_ultrasonicSensor4','Pioneer_p3dx_ultrasonicSensor6','Pioneer_p3dx_ultrasonicSensor8',clientID,currentScenario);
% Getting the current robot position
[returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
% Orienting the mobile robot in the direction of the goal
[xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
% Getting direction robot is facing
headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
%{
[returnCode2,floorHandle] = currentScenario.simxGetObjectHandle(clientID,'Floor',currentScenario.simx_opmode_blocking);
[angles,angles2] = currentScenario.simxGetObjectOrientation(clientID,pioneerRobot,floorHandle,currentScenario.simx_opmode_buffer);
%}
% Running the obstacle avoidance simulation
% Re-orient robot position based on goal and its current position
returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,clientID,pioneerRobot,leftMotor,rightMotor,currentScenario);
% Enabling proximity sensors/ Setting them to streaming
[returnCode, firstdetectionState,detectedPoint,~,~] = currentScenario.simxReadProximitySensor(clientID,firstSensor,currentScenario.simx_opmode_streaming);
[returnCode, seconddetectionState,detectedPoint,~,~] = currentScenario.simxReadProximitySensor(clientID,secondSensor,currentScenario.simx_opmode_streaming);
[returnCode, thirddetectionState,detectedPoint,~,~] = currentScenario.simxReadProximitySensor(clientID,thirdSensor,currentScenario.simx_opmode_streaming);
[returnCode, fourthdetectionState,detectedPoint,~,~] = currentScenario.simxReadProximitySensor(clientID,fourthSensor,currentScenario.simx_opmode_streaming);
% Starting the journey towards the goal
while(1)
    % Adding velocity to both motors
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.5,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.5,currentScenario.simx_opmode_blocking);
    pause(1.5);
    % Adding velocity to both motors
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.0,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.0,currentScenario.simx_opmode_blocking);
    reachedGoal = checkIfAtGoalPosition(goalPosition,currentScenario,clientID,pioneerRobot);
    if reachedGoal
       break;
    end
    % Checking if we've detected obstacle
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.5,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.5,currentScenario.simx_opmode_blocking);
    [obstacleDetected,obstacleLocation] = checkForObstacles(firstSensor,secondSensor,thirdSensor,fourthSensor,clientID,currentScenario);
    if obstacleDetected
        obstacleAvoided = avoidObstacles(leftMotor,rightMotor,currentScenario,clientID,obstacleLocation,pioneerRobot);
        if obstacleAvoided
            reachedGoal = checkIfAtGoalPosition(goalPosition,currentScenario,clientID,pioneerRobot);
            if reachedGoal
                break;
            else
                % Getting the current robot position
                [returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
                % Orienting the mobile robot in the direction of the goal
                [xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
                % Getting direction robot is facing
                headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
                % Re-orient robot position based on goal and its current position
                returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,clientID,pioneerRobot,leftMotor,rightMotor,currentScenario);
                continue;
            end
        end
    end
    % After moving for a while we have to reorient ourselves towards the
    % goal direction
    [jointReturn] = currentScenario.simxSetJointTargetVelocity(clientID,leftMotor,0.0,currentScenario.simx_opmode_blocking);
    [jointReturn]  = currentScenario.simxSetJointTargetVelocity(clientID,rightMotor,0.0,currentScenario.simx_opmode_blocking);
    % Orienting the mobile robot in the direction of the goal
    % Getting the current robot position
    [returnCode,robotPositionCurrent] = currentScenario.simxGetObjectPosition(clientID,pioneerRobot,-1,currentScenario.simx_opmode_blocking);
    [xDirection,yDirection] = goalDirection(robotPositionCurrent,goalPosition);
    % Getting direction robot is facing
    headDirection = getDirectionRobotIsFacing(clientID,currentScenario,leftMotor,rightMotor,pioneerRobot);
    % Re-orient robot position based on goal and its current position
    returnCode = orientRobotTowardsGoal([xDirection,yDirection],headDirection,clientID,pioneerRobot,leftMotor,rightMotor,currentScenario);
    continue;
end
% Setting the proximity sensorts to streaming
% runCoppeliaSimulation();
disp("We're at the goal location!!!")
%------------Helper function to check whether we've arrive at the goal position-------%
function reachedGoal = checkIfAtGoalPosition(goalPosition,simulation,clientId,pioneerRobot)
    % Getting the current robot position
    [returnCode,robotPositionCurrent] = simulation.simxGetObjectPosition(clientId,pioneerRobot,-1,simulation.simx_opmode_blocking);
    % Checking if we're at the goal position
    goalX = goalPosition(1,1);goalY = goalPosition(1,2);
    currentX = robotPositionCurrent(1,1);currentY = robotPositionCurrent(1,2);
    if (((goalX - .5<= currentX) && (currentX <= goalX + .5)) && ((goalY - .5 <= currentY)  && (currentY <= goalY + .5)))
        reachedGoal = true;
    else
        reachedGoal = false;
    end
end
%------------Helper function to avoid any obstacles-------%
function obstacleAvoided = avoidObstacles(leftMotor,rightMotor,simulation,clientId,obstacleLocation,pioneerRobot)
    % Getting obstacles x and y coordinates
    obstacleX = obstacleLocation(1,1);
    obstacleY = obstacleLocation(1,2);
    % Getting the robot's position
    [robotPosition,robo1] = simulation.simxGetObjectPosition(clientId,pioneerRobot,-1,simulation.simx_opmode_blocking);
    robotX = robo1(1,1);
    robotY = robo1(1,2);
    % Setting the joint velocities to 0
    % Breaking
    [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
    [jointReturn]  = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
    % Turning by 90 degrees
    % Rotating robot by dynamically picking a motor for rotation
    if robotX > obstacleX
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        % Breaking
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
    elseif robotX < obstacleX
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        % Breaking
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
    else
       [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        % Breaking
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking); 
    end
     % The obstacle avoidance maneuver is complete
    obstacleAvoided = true;
end
%------------Helper function to detect any obstacles-------%
function [obstacleDetected,position] = checkForObstacles(firstSensor,secondSensor,thirdSensor,fourthSensor,clientId,simulation)
    position = nan;
    % Changing the operation mode to buffer
    [returnCode, firstdetectionState,detectedPointOne,~,~] = simulation.simxReadProximitySensor(clientId,firstSensor,simulation.simx_opmode_buffer);
    [returnCode, seconddetectionState,detectedPointTwo,~,~] = simulation.simxReadProximitySensor(clientId,secondSensor,simulation.simx_opmode_buffer);
    [returnCode, thirddetectionState,detectedPointThree,~,~] = simulation.simxReadProximitySensor(clientId,thirdSensor,simulation.simx_opmode_buffer);
    [returnCode, fourthdetectionState,detectedPointFour,~,~] = simulation.simxReadProximitySensor(clientId,fourthSensor,simulation.simx_opmode_buffer);  
    % Checking the detection states
    %if firstdetectionState||seconddetectionState||thirddetectionState||fourthdetectionState
    if firstdetectionState
        obstacleDetected = true;
        position = detectedPointOne;
        % An immediate return to avoid obstacle
        return;
    elseif seconddetectionState
        obstacleDetected = true;
        position = detectedPointTwo;
        return
    elseif thirddetectionState
        obstacleDetected = true;
        position = detectedPointThree;
        return
    elseif fourthdetectionState
        obstacleDetected = true;
        position = detectedPointFour;
        return
    else
        obstacleDetected = false;
    end
end
%------------Helper function to face the robot's head towards goal-------%
function returnCode = orientRobotTowardsGoal(goalDirection,headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation)
    xDirectionToGoal = goalDirection(1,1); yDirectionToGoal = goalDirection(1,2);
    if (xDirectionToGoal == "L" && yDirectionToGoal == "D")
        returnMessage = orientToLDdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation);
    elseif (xDirectionToGoal == "L" && yDirectionToGoal == "U")
        returnMessage = orientToLUdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation);
    elseif (xDirectionToGoal == "R" && yDirectionToGoal == "U")
        returnMessage = orientToRUdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation);
    elseif (xDirectionToGoal == "R" && yDirectionToGoal == "D")
        returnMessage = orientToRDdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation);
    else
    end
    if returnMessage == "DONE"
        returnCode = 1;
        return
    else
        disp("Error in picking function to re-orient robot with!")
    end
end
%------------Helper function to face the robot's head towards RightUp direction-------%
function message = orientToRUdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation)
    % Getting head direction
    headXDirection = headDirection(1,1);
    headYDirection = headDirection(1,2);
    if (headXDirection == "R" && headYDirection == "U")
        message = "DONE";
        return
    elseif (headXDirection == "R" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
        pause(2);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    end
end
%------------Helper function to face the robot's head towards RightDown direction-------%
function message = orientToRDdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation)
    % Getting head direction
    headXDirection = headDirection(1,1);
    headYDirection = headDirection(1,2);
    if (headXDirection == "R" && headYDirection == "D")
        message = "DONE";
        return
    elseif (headXDirection == "R" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    end
end
%------------Helper function to face the robot's head towards LeftUp direction-------%
function message = orientToLUdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation)
    % Getting head direction
    headXDirection = headDirection(1,1);
    headYDirection = headDirection(1,2);
    if (headXDirection == "L" && headYDirection == "U")
        message = "DONE";
        return
    elseif (headXDirection == "R" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "R" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    end
end
%------------Helper function to face the robot's head towards LeftDown direction-------%
function message = orientToLDdirection(headDirection,clientId,robotHandle,leftMotor,rightMotor,simulation)
    % Getting head direction
    headXDirection = headDirection(1,1);
    headYDirection = headDirection(1,2);
    if (headXDirection == "L" && headYDirection == "D")
        message = "DONE";
        return
    elseif (headXDirection == "R" && headYDirection == "D")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(2.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "R" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
        pause(4);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    elseif (headXDirection == "L" && headYDirection == "U")
        % Rotating robor using just left wheel 
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
        pause(1.5);
        [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
        message = "DONE";
        return;
    end
end
%------------Helper function to get the direction the robot head is facing-------%
function headDirection = getDirectionRobotIsFacing(clientId,simulation,leftMotor,rightMotor,pioneerRobot)
    % Getting the current robot position
    [returnCode,robotPositionInitial] = simulation.simxGetObjectPosition(clientId,pioneerRobot,-1,simulation.simx_opmode_blocking);
    % Moving the robot forward for a bit
    [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.5,simulation.simx_opmode_blocking);
    [jointReturn]  = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.5,simulation.simx_opmode_blocking);
    pause(1);
    % Breaking
    [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
    [jointReturn]  = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
    % Getting the current robot position
    [returnCode,robotPositionNew] = simulation.simxGetObjectPosition(clientId,pioneerRobot,-1,simulation.simx_opmode_blocking);
    headDirection = determineHeadFromPositionChange(robotPositionInitial,robotPositionNew);
    % Reversing the robot to the initial position
    [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,-0.5,simulation.simx_opmode_blocking);
    [jointReturn]  = simulation.simxSetJointTargetVelocity(clientId,rightMotor,-0.5,simulation.simx_opmode_blocking);
    pause(1);
    [jointReturn] = simulation.simxSetJointTargetVelocity(clientId,leftMotor,0.0,simulation.simx_opmode_blocking);
    [jointReturn]  = simulation.simxSetJointTargetVelocity(clientId,rightMotor,0.0,simulation.simx_opmode_blocking);
end
%------------Helper function to get the direction the robot head is facing by checking position change-------%
function directionOfHead = determineHeadFromPositionChange(robotPositionInitial,robotPositionNew)
    initialX = robotPositionInitial(1,1); initialY = robotPositionInitial(1,2);
    newX = robotPositionNew(1,1); newY = robotPositionNew(1,2);
    if initialX == newX% We're moving vertically thus check y
        if initialY > newY
            % robot is facing downwards
            directionOfHead = ["Downwards"];
        else
            directionOfHead = ["Upwards"];
        end
    elseif initialY == newY% We're moving horizontally thus check x
         if initialX > newX
             % Robot is facing left
                directionOfHead = ["Leftwards"];
         else
                directionOfHead = ["Rightwards"];
         end
    % Diagonall movements
    elseif initialX<newX && initialY>newY
         directionOfHead = ["R","D"];
    elseif initialX<newX && initialY<newY
         directionOfHead = ["R","U"];
    elseif initialX>newX && initialY<newY
        directionOfHead = ["L","U"];
    elseif initialX>newX && initialY>newY
        directionOfHead = ["L","D"];  
    end
    
end
%------------Helper function to get the position of the goal relative to the robot------------------%
function [xOrientation,yOrientation] = goalDirection(xStart,xGoal)
    %{
    goalIsToTheRight = false;
    goalIsToTheLeft = false;
    goalIsDownwards = false;
    goalIsUpwards = false;
    goalXCoordIsstartXCoord = false;
    goalYCoordIsstartYCoord = false;
    %}
    % Focusing on the x coordinate and y coordinates
    startXCoord = xStart(1,1); startYCoord = xStart(1,2);
    goalXCoord = xGoal(1,1); goalYCoord = xGoal(1,2);
    % Orienting wrt x direction
    if (startXCoord > goalXCoord)
        goalIsToTheLeft = true;
        xOrientation = "L";
    elseif(startXCoord < goalXCoord)
        goalIsToTheRight = true;
        xOrientation = "R";
    else
        goalXCoordIsstartXCoord
        xOrientation = "O";
    end
    % Orienting wrt y direction
    if (startYCoord > goalYCoord)
        goalIsDownwards = true;
        yOrientation = "D";
    elseif (startYCoord < goalYCoord)
        goalIsUpwards = true;
        yOrientation = "U";
    else
        goalYCoordIsstartYCoord = true;
        yOrientation = "O";
    end
end
%------------Helper function to break/bring the robot to a halt-----------%


%-----------Helper function to get the proximity sensor object handles----%
function [sensor1,sensor2,sensor3,sensor4] = getProximitySensorHandles(firstSensorId,secondSensorId,thirdSensorId,fourthSensorId,clientId,simulation)
    [returnCode1,sensor1] = simulation.simxGetObjectHandle(clientId,firstSensorId,simulation.simx_opmode_blocking);
    [returnCode2,sensor2] = simulation.simxGetObjectHandle(clientId,secondSensorId,simulation.simx_opmode_blocking);
    [returnCode3,sensor3] = simulation.simxGetObjectHandle(clientId,thirdSensorId,simulation.simx_opmode_blocking);
    [returnCode4,sensor4] = simulation.simxGetObjectHandle(clientId,fourthSensorId,simulation.simx_opmode_blocking);
    if isequal((returnCode1*returnCode2*returnCode3*returnCode4),nan)
        disp("Error in getting proximity sensor handles!");
        return;
    end
end

%-----------Helper function to get the left and right motor object handles----%
function [robotLeftMotor, robotRightMotor] = getMotorHandles(leftMotorString,rightMotorString,ClientId,simulation)
    [returnCode1,robotLeftMotor] = simulation.simxGetObjectHandle(ClientId,leftMotorString,simulation.simx_opmode_blocking);
    [returnCode2,robotRightMotor] = simulation.simxGetObjectHandle(ClientId,rightMotorString,simulation.simx_opmode_blocking);
    % Sanity check
    if isequal((returnCode1*returnCode2),nan)
        disp("Error in getting motor handles.");
        return;
    else
    end
end

%-----------Helper function to get the robot's object handle----%
function robotHandle = getRobotHandle(clientId, simulation)
    [returnCode,robotHandle] = simulation.simxGetObjectHandle(clientId,'Pioneer_p3dx',simulation.simx_opmode_blocking);
    % Sanity check
    if isequal(robotHandle,nan)
        disp("Error in getting the robot's handle.")
        return;
    end
end

%-----------Helper function to set the robot start and goal positions----%
function statusCode = setRobotStartAndGoalPositions(startCoordinates,goalCoordinates,robotHandle,simulation,clientId)
    % Setting the robot's initial position
    [statusCode] = simulation.simxSetObjectPosition(clientId,robotHandle,-1,[startCoordinates +0.13879],simulation.simx_opmode_blocking);

end

%-----------Helper function to set the obstacle positions----%
function statusCode = setObstacleAtUserPositions(cuboidPosition,cylinderPosition,spherePosition,clientId,simulation)
    % Getting cuboid handle
    [returnCode1,cuboidHandle] = simulation.simxGetObjectHandle(clientId,'Cuboid',simulation.simx_opmode_blocking);
    % Getting cylinder handle
    [returnCode2,cylinderHandle] = simulation.simxGetObjectHandle(clientId,'Cylinder',simulation.simx_opmode_blocking);
    % Getting sphere handle
    [returnCode3,sphereHandle] = simulation.simxGetObjectHandle(clientId,'Sphere',simulation.simx_opmode_blocking);
    % Sanity check
    if isequal((returnCode1||returnCode2||returnCode3),nan)
       disp("Error in getting object handles!");
        return
    end

    % Setting each shape's position
    [statusCode1] = simulation.simxSetObjectPosition(clientId,cuboidHandle,-1,[cuboidPosition +0.075],simulation.simx_opmode_blocking); 
    [statusCode2] = simulation.simxSetObjectPosition(clientId,cylinderHandle,-1,[cylinderPosition +0.1500],simulation.simx_opmode_blocking); 
    [statusCode3] = simulation.simxSetObjectPosition(clientId,sphereHandle,-1,[spherePosition +0.1500],simulation.simx_opmode_blocking); 
    % Sanity check
    if isequal((statusCode1||statusCode2||statusCode3),nan)
        disp("Error in setting the object position!");
        return
    end
    statusCode = statusCode1;
end

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

%------------Helper function to get the obstacle coordinates---------%
function [cuboidList,sphereList,cylinderList] = getObstacleCoordinates()
    cuboidList = getCuboidCoordinates();
    sphereList = getSphereCoordinates();
    cylinderList = getCylinderCoordinates();
    pause(5);
end
%-------------Helper function to get the cuboid coordinates---------%
function cuboidCoords = getCuboidCoordinates()
    cuboidX = input("What is the cuboid's x-coordinate?In range: [-2.4...0...2.4]");
    cuboidY = input("What is the cuboid's y-coordinate?In range: [-2.4...0...2.4]");
    cuboidCoords = [cuboidX,cuboidY];
end
%-------------Helper function to get the sphere coordinates---------%
function sphereCoords = getSphereCoordinates()
    sphereX = input("What is the sphere's x-coordinate?In range: [-2.4...0...2.4]");
    sphereY = input("What is the sphere's y-coordinate?In range: [-2.4...0...2.4]");
    sphereCoords = [sphereX,sphereY];
end
%-------------Helper function to get the cylinder coordinates---------%
function cylinderCoords = getCylinderCoordinates()
    cylinderX = input("What is the cylinder's x-coordinate?In range: [-2.4...0...2.4]");
    cylinderY = input("What is the cylinder's y-coordinate?In range: [-2.4...0...2.4]");
    cylinderCoords = [cylinderX,cylinderY];
end
%------------Helper function to get the start coordinates------------%
function startCoords = getStartCoordinates()
    % Getting the start x & y-coordinates
    startX = input("What is the x-coordinate of the start position?In range: [-2.4...0...2.4]");
    startY = input("What is the y-coordinate of the start position?In range: [-2.4...0...2.4]");
    startCoords = [startX,startY];
    % pause(5);
end
%------------Helper function to get the goal coordinates-------------%
function goalCoords = getGoalCoordinates()
    % Getting the goal x & y-coordinates
    goalX = input("What is the x-coordinate of the goal position?In range: [-2.4...0...2.4]");
    goalY = input("What is the y-coordinate of the goal position?In range: [-2.4...0...2.4]");
    goalCoords = [goalX,goalY];
end
