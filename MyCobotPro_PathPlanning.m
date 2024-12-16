[myCobotBU,arm_info2] = importrobot("C:\ASU\Robotics\FinalProject\RAS_Final_Project\Robot\MyCobot600Pro_Simulink.slx");  

%% Position Adjustments  

% Read the CSV file containing interpolation points
% The CSV file should have two columns representing X and Y coordinates
Input = readtable("C:\ASU\Robotics\FinalProject\RAS_Final_Project\Robot\InterpolationPoints.csv");
X = Input.Var1; % Extract X coordinates
Y = Input.Var2; % Extract Y coordinates

% Define the starting position based on the first interpolation point
x_start = X(1);
y_start = Y(1);

% Calculate the number of interpolation points from the input data
points = length(X); 

% Calculate offsets for the maze path relative to the origin (start position)
xoff = X' - x_start; % Offset in the X direction
yoff = Y' - y_start; % Offset in the Y direction

% Generate a time vector for interpolation points with a step of 5 seconds
time = 0:5:(points-1)*5;

% Create timeseries objects for the offsets to use in the Simulink model
x_Os = timeseries(xoff, time);
y_Os = timeseries(yoff, time);

%% Run Simulink 

% Define the simulation stop time based on the number of points
stoptim = num2str(5*points); % Each point corresponds to 5 seconds of simulation

% Execute the Simulink model and collect the output\out = sim('Cobot600_PathFeedback.slx', 'StopTime', stoptim);

%% Reshaping Data from Simulink Twin

% Reshape the inverse kinematics (IK) target data from the Simulink output
IK_input = reshape(out.IK_target,[],3); % Target positions (X, Y, Z)

% Reshape joint data from the Simulink output
IK_Jo = reshape(out.IKJointData,6,[])'; % Joint angles for six joints

% Correct specific joint angles to account for interdependencies:
% - J4 depends on the interaction between J2 and J3
% - Adjustments for J2 and J5 for compatibility
IK_Jo(:,4) = -180 - IK_Jo(:,2) - IK_Jo(:,3);
IK_Jo(:,2) = IK_Jo(:,2) -90;
IK_Jo(:,5) = 90;

%% Find angles input with minimum error 

% Calculate the total error across all points from the IK error data
error = out.IK_error;
etot = sum(abs(error),2);

% Initialize variables to store results
SolJoints = zeros(points,6); % Solution joint angles for each point
posCheck = zeros(points,3);  % Verified positions for each point
erIx = zeros(points,1);      % Index of minimum error for each point
Ix = 1:10; % Define a search range of 10 samples (0.5-second intervals) for each step

% Iterate through each interpolation point to find the best solution
for i = 1:points
    % Locate the minimum error value within the current range
    k = find(etot==min(etot(Ix))); 
    erIx(i) = k(1)-1; % Select the first minimum instance (adjust for delay)
    SolJoints(i,:) = IK_Jo(erIx(i),:); % Extract joint angles corresponding to minimum error
    posCheck(i,:) = IK_input(erIx(i),:); % Extract corresponding position
    Ix = Ix + 10; % Move to the next range
end

%% Print Joints

% Display the computed joint angles for all interpolation points
disp(SolJoints)

% Write the joint angles to a CSV file for further use
writematrix(SolJoints,'joints','delimiter',',');

%% Validate End-Effector Data with Matlab FK Code (Optional Check)

% Initialize an array to store computed end-effector positions
Code_EE_pos = zeros(points,3);

% Compute forward kinematics for each set of joint angles to validate results
for i = 1:points
    % Extract individual joint angles for the current point
    the1 = SolJoints(i,1);
    the2 = SolJoints(i,2);
    the3 = SolJoints(i,3);
    the4 = SolJoints(i,4);
    the5 = SolJoints(i,5);
    the6 = SolJoints(i,6);
    
    % Calculate the end-effector position using forward kinematics
    [Px, Py, Pz] = Cobot_ForwardKinematics(the1,the2,the3,the4,the5,the6);
    Code_EE_pos(i,:) = [Px, Py, Pz]; % Store the position

    % Check if the end-effector height is below the minimum threshold
    if Pz < 20
        disp("Under 20mm check before running... :(")
    end 
end

% Display the computed end-effector positions
disp(Code_EE_pos)

% Calculate the positional error between IK and FK results
code_error = posCheck - Code_EE_pos;

%% Verify Input Data Consistency

% Check if the positions computed from the IK match the input data
check = [X Y] -posCheck(:,1:2); % Compare X and Y coordinates
if sum(check) == 0 
    disp('Data matches input!')
else 
    disp('FAIL')
end
