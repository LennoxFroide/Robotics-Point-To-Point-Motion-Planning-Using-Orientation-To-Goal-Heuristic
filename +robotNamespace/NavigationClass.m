classdef NavigationClass
    properties
        clientId;
        scenarioName;
        robotHandle;
        leftMotor;
        rightMotor;
        % Proximity sensors
        firstSensor, secondSensor, thirdSensor, fourthSensor;
    end

    methods
        %--------------Function to set the properties of the class--------%
        function object = NavigationClass(userId,simEnvironment)
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
            % Getting the left and right motors to maneuver the robot
            [object.leftMotor, object.rightMotor] = object.getMotorHandles('Pioneer_p3dx_leftMotor','Pioneer_p3dx_rightMotor');
        end
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
        %-----------Helper function to get the left and right motor object handles----%
        function [leftMotor, rightMotor] = getMotorHandles(object,leftMotorString,rightMotorString)
            [returnCode1,leftMotor] = object.scenarioName.simxGetObjectHandle(object.clientId,leftMotorString,object.scenarioName.simx_opmode_blocking);
            [returnCode2,rightMotor] = object.scenarioName.simxGetObjectHandle(object.clientId,rightMotorString,object.scenarioName.simx_opmode_blocking);
            % Sanity check
            if isequal((returnCode1*returnCode2),nan)
                disp("Error in getting motor handles.");
                return;
            else
            end
        end

        %-----------Helper function to get the proximity sensor object handles----%
        function [sensor1,sensor2,sensor3,sensor4] = getProximitySensorHandles(object,firstSensorId,secondSensorId,thirdSensorId,fourthSensorId)
            [returnCode1,sensor1] = object.scenarioName.simxGetObjectHandle(object.clientId,firstSensorId,object.scenarioName.simx_opmode_blocking);
            [returnCode2,sensor2] = object.scenarioName.simxGetObjectHandle(object.clientId,secondSensorId,object.scenarioName.simx_opmode_blocking);
            [returnCode3,sensor3] = object.scenarioName.simxGetObjectHandle(object.clientId,thirdSensorId,object.scenarioName.simx_opmode_blocking);
            [returnCode4,sensor4] = object.scenarioName.simxGetObjectHandle(object.clientId,fourthSensorId,object.scenarioName.simx_opmode_blocking);;
            if isequal((returnCode1*returnCode2*returnCode3*returnCode4),nan)
                disp("Error in getting proximity sensor handles!");
                return;
            end
        end
        %--------------Helper function to enable proximity sensors--------%
        function [statusCode,sensorState,obstaclePosition] = enableProximitySensors(object,sensorName)
            if sensorName == "first"
                [statusCode, sensorState,obstaclePosition,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,object.firstSensor,object.scenarioName.simx_opmode_streaming);
            elseif sensorName == "second"
                [statusCode, sensorState,obstaclePosition,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,object.secondSensor,object.scenarioName.simx_opmode_streaming);
            elseif sensorName == "third"
                [statusCode, sensorState,obstaclePosition,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,object.thirdSensor,object.scenarioName.simx_opmode_streaming);
            elseif sensorName == "fourth"
                [statusCode, sensorState,obstaclePosition,~,~] = object.scenarioName.simxReadProximitySensor(object.clientId,object.fourthSensor,object.scenarioName.simx_opmode_streaming);
            else
                disp("Error in providing sensor name!!")
                return
            end
        end
        %------------Helper function to get the position of the goal relative to the robot------------------%
        function [xOrientation,yOrientation] = goalDirection(object,xStart,xGoal)
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
                goalXCoordIsstartXCoord = true;
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

        %------------Helper function to get the direction the robot head is facing-------%
        function headDirection = getDirectionRobotIsFacing(object,pioneerRobot)
            % Getting the current robot position
            [returnCode,robotPositionInitial] = object.scenarioName.simxGetObjectPosition(object.clientId,pioneerRobot,-1,object.scenarioName.simx_opmode_blocking);
            % Moving the robot forward for a bit
            [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
            [jointReturn]  = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
            pause(1);
            % Breaking
            [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
            [jointReturn]  = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
            % Getting the current robot position
            [returnCode,robotPositionNew] = object.scenarioName.simxGetObjectPosition(object.clientId,pioneerRobot,-1,object.scenarioName.simx_opmode_blocking);
            headDirection = object.determineHeadFromPositionChange(robotPositionInitial,robotPositionNew);
            % Reversing the robot to the initial position
            [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,-0.5,object.scenarioName.simx_opmode_blocking);
            [jointReturn]  = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,-0.5,object.scenarioName.simx_opmode_blocking);
            pause(1);
            [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
            [jointReturn]  = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
        end

        %------------Helper function to get the direction the robot head is facing by checking position change-------%
        function directionOfHead = determineHeadFromPositionChange(object,robotPositionInitial,robotPositionNew)
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
        %------------Helper function to face the robot's head towards goal-------%
        function returnCode = orientRobotTowardsGoal(object,goalDirection,headDirection,robotHandle)
            xDirectionToGoal = goalDirection(1,1); yDirectionToGoal = goalDirection(1,2);
            if (xDirectionToGoal == "L" && yDirectionToGoal == "D")
                returnMessage = object.orientToLDdirection(headDirection);
            elseif (xDirectionToGoal == "L" && yDirectionToGoal == "U")
                returnMessage = object.orientToLUdirection(headDirection);
            elseif (xDirectionToGoal == "R" && yDirectionToGoal == "U")
                returnMessage = object.orientToRUdirection(headDirection);
            elseif (xDirectionToGoal == "R" && yDirectionToGoal == "D")
                returnMessage = object.orientToRDdirection(headDirection);
            else
            end
            if returnMessage == "DONE"
                returnCode = 1;
                return
            else
                disp("Error in picking function to re-orient robot with!")
            end
        end
        %------------Helper function to face the robot's head towards LeftDown direction-------%
        function message = orientToLDdirection(object,headDirection)
            % Getting head direction
            headXDirection = headDirection(1,1);
            headYDirection = headDirection(1,2);
            if (headXDirection == "L" && headYDirection == "D")
                message = "DONE";
                return
            elseif (headXDirection == "R" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "R" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(4);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            end
        end
        %------------Helper function to face the robot's head towards LeftUp direction-------%
        function message = orientToLUdirection(object,headDirection)
            % Getting head direction
            headXDirection = headDirection(1,1);
            headYDirection = headDirection(1,2);
            if (headXDirection == "L" && headYDirection == "U")
                message = "DONE";
                return
            elseif (headXDirection == "R" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "R" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            end
        end

        %------------Helper function to face the robot's head towards RightUp direction-------%
        function message = orientToRUdirection(object,headDirection)
            % Getting head direction
            headXDirection = headDirection(1,1);
            headYDirection = headDirection(1,2);
            if (headXDirection == "R" && headYDirection == "U")
                message = "DONE";
                return
            elseif (headXDirection == "R" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            end
        end

        %------------Helper function to face the robot's head towards RightDown direction-------%
        function message = orientToRDdirection(object,headDirection)
            % Getting head direction
            headXDirection = headDirection(1,1);
            headYDirection = headDirection(1,2);
            if (headXDirection == "R" && headYDirection == "D")
                message = "DONE";
                return
            elseif (headXDirection == "R" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "U")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(2.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.leftMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            elseif (headXDirection == "L" && headYDirection == "D")
                % Rotating robor using just left wheel 
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.5,object.scenarioName.simx_opmode_blocking);
                pause(1.5);
                [jointReturn] = object.scenarioName.simxSetJointTargetVelocity(object.clientId,object.rightMotor,0.0,object.scenarioName.simx_opmode_blocking);
                message = "DONE";
                return;
            end
        end
    end
end