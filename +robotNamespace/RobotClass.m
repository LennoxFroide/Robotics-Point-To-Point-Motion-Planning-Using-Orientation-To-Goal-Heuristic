classdef RobotClass
    properties
        clientId;
        scenarioName;
        robotHandle;
        leftMotor;
        rightMotor;
        start;
        goal;
    end
    methods
        %-------------------------UTILITY METHODS-------------------------%
        
        %--------------Function to set the properties of the class--------%
        function object = RobotClass(userId,simEnvironment)
            if nargin == 2
                object.clientId = userId;
                object.scenarioName = simEnvironment;
            end
            % Setting the robot handle property
            [statusCode,object.robotHandle] = getRobotHandle(object);
            if statusCode < 0
                disp("Error in retrieving the robot handle!");
                return
            end
            % Getting the start and goal positions
            object.start = object.getStartCoordinates();
            object.goal = object.getGoalCoordinates();
        end

        %-----------------------HELPER FUNCTIONS--------------------------%

        %-----------Helper function to get the robot's object handle------%
        % function robotHandle = getRobotHandle(clientId, simulation)
        function [returnCode,robotHandle] = getRobotHandle(object)
            [returnCode,robotHandle] = object.scenarioName.simxGetObjectHandle(object.clientId,'Pioneer_p3dx',object.scenarioName.simx_opmode_blocking);
            % Sanity check
            if isequal(robotHandle,nan)
                disp("Error in getting the robot's handle.")
                return;
            end
        end
        %------------Helper function to get the start coordinates------------%
        function startCoords = getStartCoordinates(object)
            % Getting the start x & y-coordinates
            startX = input("What is the x-coordinate of the start position?In range: [-2.4...0...2.4]");
            startY = input("What is the y-coordinate of the start position?In range: [-2.4...0...2.4]");
            startCoords = [startX,startY];
            % pause(5);
        end
        %------------Helper function to get the goal coordinates-------------%
        function goalCoords = getGoalCoordinates(object)
            % Getting the goal x & y-coordinates
            goalX = input("What is the x-coordinate of the goal position?In range: [-2.4...0...2.4]");
            goalY = input("What is the y-coordinate of the goal position?In range: [-2.4...0...2.4]");
            goalCoords = [goalX,goalY];
        end
        %-----------Helper function to set the robot start and goal positions----%
        function statusCode = setRobotStartPosition(object,startCoordinates)
            % Setting the robot's initial position
            [statusCode] = object.scenarioName.simxSetObjectPosition(object.clientId,object.robotHandle,-1,[startCoordinates +0.13879],object.scenarioName.simx_opmode_blocking);
        
        end
        %------------Helper function to check whether we've arrive at the goal position-------%
        function reachedGoal = checkIfAtGoalPosition(object, goalPosition)
            % Getting the current robot position
            [returnCode,robotPositionCurrent] = object.scenarioName.simxGetObjectPosition(object.clientId,object.robotHandle,-1,object.scenarioName.simx_opmode_blocking);
            % Checking if we're at the goal position
            goalX = goalPosition(1,1);goalY = goalPosition(1,2);
            currentX = robotPositionCurrent(1,1);currentY = robotPositionCurrent(1,2);
            if (((goalX - .5<= currentX) && (currentX <= goalX + .5)) && ((goalY - .5 <= currentY)  && (currentY <= goalY + .5)))
                reachedGoal = true;
            else
                reachedGoal = false;
            end
        end

        %------------Helper function to detect any obstacles-------%
        function [obstacleDetected,position] = checkForObstacles(object,firstSensor,secondSensor,thirdSensor,fourthSensor)
            position = nan;
            % Changing the operation mode to buffer
            [returnCode, firstdetectionState,detectedPointOne,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,firstSensor,object.scenarioName.simx_opmode_buffer);
            [returnCode, seconddetectionState,detectedPointTwo,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,secondSensor,object.scenarioName.simx_opmode_buffer);
            [returnCode, thirddetectionState,detectedPointThree,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,thirdSensor,object.scenarioName.simx_opmode_buffer);
            [returnCode, fourthdetectionState,detectedPointFour,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,fourthSensor,object.scenarioName.simx_opmode_buffer);  
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

        %------------Helper function to avoid any obstacles-------%
        function obstacleAvoided = avoidObstacles(object,leftMotor,rightMotor,obstacleLocation)
            % Getting obstacles x and y coordinates
            obstacleX = obstacleLocation(1,1);
            obstacleY = obstacleLocation(1,2);
            % Getting the robot's position
            [robotPosition,robo1] = object.scenarioName.simxGetObjectPosition(object.clientId,object.robotHandle,-1,object.scenarioName.simx_opmode_blocking);
            robotX = robo1(1,1);
            robotY = robo1(1,2);
            % Setting the joint velocities to 0
            % Breaking
            [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
            [jointReturn]  = object.scenarioName.simxSetJointTargetVelocity(object.clientId,rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
            % Turning by 90 degrees
            % Rotating robot by dynamically picking a motor for rotation
            if robotX > obstacleX
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                % Breaking
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
            elseif robotX < obstacleX
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                % Breaking
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
            else
               [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                % Breaking
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,leftMotor,0.0,object.scenarioName.simx_opmode_blocking); 
            end
             % The obstacle avoidance maneuver is complete
            obstacleAvoided = true;
        end
    end
end