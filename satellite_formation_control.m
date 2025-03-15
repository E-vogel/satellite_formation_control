%% Clear Workspace and Setup Environment
clear
close all

% Load star image coordinate data (assumes "star_data" is in the file)
load("star_image_coordinate.mat")

%% Define Physical Constants and Orbit Parameters
G = 6.67430e-11;                  % Gravitational constant [m^3 kg^-1 s^-2]
earthRadius = 6.378e6;            % Earth's radius [m]
earthMass = 5.972e24;             % Earth's mass [kg]
orbitAltitude = 6e6;              % Altitude above Earth's surface for satellite orbit [m]

%% Setup Figure and Graphical Defaults
fig = figure('Position',[100 100 1200 700]);
reset(groot)                                        % Reset graphics settings to default
set(groot,'defaultAxesTickLabelInterpreter','latex')  % Use LaTeX for tick labels
set(groot,'defaultTextInterpreter','latex')           % Use LaTeX for text labels
set(groot,'defaultLegendInterpreter','latex')         % Use LaTeX for legend text
set(groot,'defaultAxesFontSize',13)                   % Set default font size for axes
set(groot,'defaultLineLineWidth',1.5)                 % Set default line width
colormap("winter")

%% Create Tiled Layout for Two Views (3D and 2D)
tiledlayout(1,2)

%% --- Left Panel: 3D View of Earth and Satellite Orbits ---
nexttile(1)
hold on

% Plot Earth as a sphere
[earthX, earthY, earthZ] = sphere(100);
earthSurfacePlot1 = surf(earthRadius * earthX, earthRadius * earthY, earthRadius * earthZ);
earthSurfacePlot1.EdgeColor = "none";

% Add lighting, grid, and set view angle
camlight
grid on;
axis equal
view(70,30)

% Define the number of satellites
numSatellites = 50;

% Generate random initial orbital parameters for satellites
azimuth = -2 * pi * rand(numSatellites, 1) * 0.3;  
% Random azimuth angles [rad]
satelliteRadius = (earthRadius + orbitAltitude) * (ones(numSatellites, 1) + 0.4*(rand(numSatellites,1)*2-1));
% Random orbital radii [m]
elevation = pi*(2*rand(numSatellites, 1)-1)*0.3;  
% Random elevation angles [rad]

% Convert spherical coordinates to Cartesian coordinates for satellites
satelliteX = satelliteRadius .* cos(azimuth) .* cos(elevation);
satelliteY = satelliteRadius .* sin(azimuth) .* cos(elevation);
satelliteZ = satelliteRadius .* sin(elevation);

% Adjust satellite positions on the image plane using star image data
referencePoint = [100, 100];
starImageCoords = star_data;  
% Downsample star image coordinates to match the number of satellites
starImageCoords = starImageCoords(1:200/numSatellites:200,:);
satelliteImageCoords = [satelliteX, satelliteY] + referencePoint .* ones(numSatellites, 2) - starImageCoords * 0.7e4;

% Calculate initial circular orbital speeds
orbitalSpeed = sqrt(G * earthMass ./ satelliteRadius);
satelliteVx = -orbitalSpeed .* sin(azimuth) .* cos(elevation);
satelliteVy = orbitalSpeed .* cos(azimuth) .* cos(elevation);
satelliteVz = 0;

% Plot satellite positions in the 3D view
satelliteScatterPlot1 = scatter3(satelliteX, satelliteY, satelliteZ, 20, "y", "filled");

% Initialize arrays to store satellite positions over time
timeStep = 100;         % Time step [s]
totalTime = 50000;      % Total simulation time [s]
numTimeSteps = totalTime / timeStep;

satelliteXHistory = zeros(numSatellites, numTimeSteps);
satelliteYHistory = zeros(numSatellites, numTimeSteps);
satelliteZHistory = zeros(numSatellites, numTimeSteps);
satelliteXHistory(:,1) = satelliteX;
satelliteYHistory(:,1) = satelliteY;
satelliteZHistory(:,1) = satelliteZ;

% Initialize trajectory plots for each satellite (for path tracing)
satelliteTrajectoryPlots1 = cell(numSatellites,1);
for sat = 1:numSatellites
    satelliteTrajectoryPlots1{sat} = plot3(satelliteXHistory(sat,1), satelliteYHistory(sat,1), satelliteZHistory(sat,1), 'Color',[1 1 0 0.2]);
end

axis([-earthRadius earthRadius -earthRadius earthRadius -earthRadius earthRadius]*3.5)
box on
xlabel("$x$")
ylabel("$y$")
zlabel("$z$")

%% --- Right Panel: 2D Top View of Earth and Satellite Orbits ---
nexttile(2)
% Plot Earth in the top view
earthSurfacePlot2 = surf(earthRadius * earthX, earthRadius * earthY, earthRadius * earthZ);
earthSurfacePlot2.EdgeColor = "none";
hold on

% Draw the circular orbit path
thetaCircle = linspace(0, 2*pi, 100);
plot((earthRadius + orbitAltitude) * cos(thetaCircle), (earthRadius + orbitAltitude) * sin(thetaCircle), 'b--')

% Add lighting, grid, and set view for top-down perspective
camlight
grid on;
axis equal
view(0,90)

% Plot satellites in the 2D view
satelliteScatterPlot2 = scatter3(satelliteX, satelliteY, satelliteZ, 20, "y", "filled");

% Initialize trajectory plots for the 2D view
satelliteTrajectoryPlots2 = cell(numSatellites,1);
for sat = 1:numSatellites
    satelliteTrajectoryPlots2{sat} = plot3(satelliteXHistory(sat,1), satelliteYHistory(sat,1), satelliteZHistory(sat,1), 'Color',[1 1 0 0.2]);
end

axis([-earthRadius earthRadius -earthRadius earthRadius -earthRadius earthRadius]*3.5)
box on
xlabel("$x$")
ylabel("$y$")
zlabel("$z$")
timeTitle = "Time = " + 0 + " s";
timeText = title(timeTitle, 'FontSize', 20);

%% --- Half-Step Velocity Update (Leapfrog Integration) ---
% Compute gravitational acceleration at current satellite positions
radialDistance = sqrt(satelliteX.^2 + satelliteY.^2 + satelliteZ.^2);
gravAccelMagnitude = -G * earthMass ./ radialDistance.^2;
accelX = gravAccelMagnitude .* (satelliteX ./ radialDistance);
accelY = gravAccelMagnitude .* (satelliteY ./ radialDistance);
accelZ = gravAccelMagnitude .* (satelliteZ ./ radialDistance);

% Update velocities using half a time step (leapfrog scheme)
satelliteVx = satelliteVx + 0.5 * accelX * timeStep;
satelliteVy = satelliteVy + 0.5 * accelY * timeStep;
satelliteVz = satelliteVz + 0.5 * accelZ * timeStep;

%% --- Define Control and Interaction Parameters ---
interactionRadius = 0.7e7;         % Distance threshold for satellite interactions [m]
positionCouplingGain = 10e-5;        % Coupling gain for position adjustments
velocityCouplingGain = 7e-5;         % Coupling gain for velocity adjustments

controlGainX = 0.2e-5;  % Control gain for x-direction
controlGainY = 0.2e-5;  % Control gain for y-direction
controlGainZ = 0.2e-5;  % Control gain for z-direction

set(gca, 'LooseInset', get(gca, 'TightInset'));

%% --- Simulation Loop ---
videoWriter = VideoWriter("satellite.avi", 'Uncompressed AVI');
open(videoWriter)

for step = 2:numTimeSteps
    % Combine satellite image coordinates and z-position into one matrix
    combinedPosition = [satelliteImageCoords, satelliteZ];
    
    % Compute the interaction matrix among satellites based on distance
    interactionMatrix = zeros(numSatellites, numSatellites);
    for i = 1:numSatellites
        for j = 1:numSatellites
            distance = sqrt((satelliteX(i) - satelliteX(j))^2 + (satelliteY(i) - satelliteY(j))^2 + (satelliteZ(i) - satelliteZ(j))^2);
            if distance < interactionRadius && i ~= j
                interactionMatrix(i,j) = 1;
                interactionMatrix(j,i) = 1;
            end
        end
    end

    % Compute Laplacian matrix from the interaction matrix
    laplacianMatrix = diag(sum(interactionMatrix,2)) - interactionMatrix;
    
    % Calculate position correction due to satellite interactions
    posCorrection = -positionCouplingGain * laplacianMatrix * combinedPosition;
    
    % Update satellite positions with current velocity and interaction correction
    satelliteX = satelliteX + satelliteVx * timeStep + posCorrection(:,1) * timeStep;
    satelliteY = satelliteY + satelliteVy * timeStep + posCorrection(:,2) * timeStep;
    satelliteZ = satelliteZ + satelliteVz * timeStep + posCorrection(:,3) * timeStep;
    
    % Update image plane coordinates accordingly
    satelliteImageCoords(:,1) = satelliteImageCoords(:,1) + satelliteVx * timeStep + posCorrection(:,1) * timeStep;
    satelliteImageCoords(:,2) = satelliteImageCoords(:,2) + satelliteVy * timeStep + posCorrection(:,2) * timeStep;
    
    % Convert updated Cartesian coordinates to spherical coordinates
    azimuth = atan2(satelliteY, satelliteX);
    elevation = atan2(satelliteZ, sqrt(satelliteX.^2 + satelliteY.^2));
    radialDistance = sqrt(satelliteX.^2 + satelliteY.^2 + satelliteZ.^2);
    
    % Calculate target circular orbital speeds (scaled by factor 2)
    targetOrbitalSpeed = sqrt(G * earthMass ./ radialDistance) * 2;
    targetVx = -targetOrbitalSpeed .* sin(azimuth) .* cos(elevation);
    targetVy = targetOrbitalSpeed .* cos(azimuth) .* cos(elevation);
    targetVz = 0;
    
    % Compute control accelerations to guide satellites toward the desired orbit
    controlAccX = -controlGainX * (satelliteX - (earthRadius + orbitAltitude) .* cos(azimuth) .* cos(elevation));
    controlAccY = -controlGainY * (satelliteY - (earthRadius + orbitAltitude) .* sin(azimuth) .* cos(elevation));
    controlAccZ = -controlGainZ * (satelliteZ - (earthRadius + orbitAltitude) .* sin(elevation));
    
    % Recompute gravitational acceleration including control inputs
    gravAccelMagnitude = -G * earthMass ./ radialDistance.^2;
    accelX = gravAccelMagnitude .* (satelliteX ./ radialDistance) + controlAccX;
    accelY = gravAccelMagnitude .* (satelliteY ./ radialDistance) + controlAccY;
    accelZ = gravAccelMagnitude .* (satelliteZ ./ radialDistance) + controlAccZ;
    
    % Prepare the satellite velocity matrix for coupling correction
    satelliteVelocities = [satelliteVx, satelliteVy, satelliteVz];
    velCorrection = -velocityCouplingGain * laplacianMatrix * satelliteVelocities;
    
    % Update satellite velocities using the full time step and velocity correction
    satelliteVx = satelliteVx + accelX * timeStep + velCorrection(:,1) * timeStep;
    satelliteVy = satelliteVy + accelY * timeStep + velCorrection(:,2) * timeStep;
    satelliteVz = satelliteVz + accelZ * timeStep + velCorrection(:,3) * timeStep;
    
    % Record current positions in the history arrays
    satelliteXHistory(:, step) = satelliteX;
    satelliteYHistory(:, step) = satelliteY;
    satelliteZHistory(:, step) = satelliteZ;
    
end

%% --- Update Trajectories and Create Video ---
historyWindow = 10;  % Number of simulation steps to display in the trajectory history

for step = 1:numTimeSteps
    % Update the 3D view scatter plot and trajectories
    set(satelliteScatterPlot1, 'XData', satelliteXHistory(:,step), 'YData', satelliteYHistory(:,step), 'ZData', satelliteZHistory(:,step));
    currentRadialDistances = sqrt(satelliteXHistory(:,step).^2 + satelliteYHistory(:,step).^2 + satelliteZHistory(:,step).^2);
    for sat = 1:numSatellites
        if step <= historyWindow
            satelliteTrajectoryPlots1{sat}.XData = satelliteXHistory(sat, 1:step);
            satelliteTrajectoryPlots1{sat}.YData = satelliteYHistory(sat, 1:step);
            satelliteTrajectoryPlots1{sat}.ZData = satelliteZHistory(sat, 1:step);
        else
            satelliteTrajectoryPlots1{sat}.XData = satelliteXHistory(sat, step-historyWindow+1:step);
            satelliteTrajectoryPlots1{sat}.YData = satelliteYHistory(sat, step-historyWindow+1:step);
            satelliteTrajectoryPlots1{sat}.ZData = satelliteZHistory(sat, step-historyWindow+1:step);
        end
        
        % Change the trajectory color to red if the satellite is below Earth's surface
        if currentRadialDistances(sat) < earthRadius
            satelliteTrajectoryPlots1{sat}.Color = [1 0 0 0.7];
        end
    end

    % Update the 2D top view scatter plot and trajectories
    nexttile(1)
    set(satelliteScatterPlot2, 'XData', satelliteXHistory(:,step), 'YData', satelliteYHistory(:,step), 'ZData', satelliteZHistory(:,step));
    currentRadialDistances = sqrt(satelliteXHistory(:,step).^2 + satelliteYHistory(:,step).^2 + satelliteZHistory(:,step).^2);
    for sat = 1:numSatellites
        if step <= historyWindow
            satelliteTrajectoryPlots2{sat}.XData = satelliteXHistory(sat, 1:step);
            satelliteTrajectoryPlots2{sat}.YData = satelliteYHistory(sat, 1:step);
            satelliteTrajectoryPlots2{sat}.ZData = satelliteZHistory(sat, 1:step);
        else
            satelliteTrajectoryPlots2{sat}.XData = satelliteXHistory(sat, step-historyWindow+1:step);
            satelliteTrajectoryPlots2{sat}.YData = satelliteYHistory(sat, step-historyWindow+1:step);
            satelliteTrajectoryPlots2{sat}.ZData = satelliteZHistory(sat, step-historyWindow+1:step);
        end
        
        if currentRadialDistances(sat) < earthRadius
            satelliteTrajectoryPlots2{sat}.Color = [1 0 0 0.7];
        end
    end
    
    % Update the time display title
    timeTitle = "Time = " + step * timeStep + " s";
    timeText.String = timeTitle;
    
    % Capture the current figure and write the frame to the video
    frame = getframe(gcf);
    writeVideo(videoWriter, frame)
end

close(videoWriter)


