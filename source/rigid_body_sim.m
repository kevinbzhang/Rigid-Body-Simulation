% SIMPLE_RIGIDBODY_ENGINE.M ▸ gravity −Z, triangular‑prism slope, spheres **and cylinders**
% ---------------------------------------------------------------------
% MATLAB one‑file playground for rolling‑body races.
% • Gravity points in –Z.
% • 3‑m‑wide triangular wedge hosts four lanes.
% • Bodies:
% – Solid / hollow spheres (I = 2/5, 2/3 mR²)
% – Solid / hollow cylinders (axis ‖ Y). Cylinders are
% 0.6 R long; the *solid* cylinder is drawn with filled end‑caps
% so you can instantly spot the difference.
% ---------------------------------------------------------------------
% ADDED: Energy conservation tracking
clear; clc; close all;
%% ---------------- SIM CONSTANTS --------------------------------------
dt = 0.00025; % time‑step (s)
simTime = 5; % total duration (s)
g = [0 0 -9.81]; % gravity (m s⁻²)
mu = 0.30; % kinetic friction coefficient
coefRest = 1; % coefficient of restitution
CYL_LEN_FAC = 0.6; % cylinder length factor (ℓ = 0.6 R)
%% ---------------- SLOPE GEOMETRY -------------------------------------
planeH = 5; % Z‑height at X = 0 (m)
slopeAng = 15*pi/180; % wedge tilt (rad)
prismLen = 10; % X‑extent (m)
halfW = 1.5; % half‑width in Y (m)
% unit normal pointing OUT of the slope surface (toward +Z for small x)
nSlope = [sin(slopeAng) 0 cos(slopeAng)];
nSlope = nSlope./norm(nSlope);
dSlope = -planeH*nSlope(3); % plane equation: n·x + d = 0 at the surface
%% ---------------- CREATE STARTING LINE -------------------------------
R = 0.20; % common radius
lanesY = [-1.125 -0.375 0.375 1.125]; % lane centres (m)
startX = 0; startZ = planeH + R; % just touching slope
bodies(1) = makeSphere (R, 1, [startX lanesY(1) startZ], 'hollow'); % orange
bodies(2) = makeSphere (R, 1, [startX lanesY(2) startZ], 'solid' ); % blue
bodies(3) = makeCylinder(R, 1, [startX lanesY(3) startZ], 'hollow'); % red (open)
bodies(4) = makeCylinder(R, 1, [startX lanesY(4) startZ], 'solid' ); % green (filled)
%% ---------------- VISUAL SET‑UP --------------------------------------
fig = figure('Color','w','Renderer','opengl'); hold on; grid on; axis equal vis3d;
rotate3d(fig,'on'); view(35,20);
plotPrism(slopeAng,planeH,prismLen,halfW);
lighting gouraud; camlight headlight; material dull;
% Draw each body (drawBody now RETURNS the updated struct!)
for k = 1:numel(bodies)
 bodies(k) = drawBody(bodies(k), CYL_LEN_FAC);
end
xlim([-2 prismLen+1]); ylim([-halfW-0.5 halfW+0.5]);
zlim([planeH-3 planeH+R+2]); xlabel X; ylabel Y; zlabel('Z (down)');
%% ---------------- ENERGY TRACKING SETUP -----------------------------
steps = round(simTime/dt);
numBodies = numel(bodies);
% Define end of slope Z position
slopeEndZ = planeH - prismLen*tan(slopeAng); 
slopeBottomThreshold = slopeEndZ + R*1.5; % A bit above the end to catch when they're near bottom
% Create storage for individual body energy tracking
% For each body: [time, KE_trans, KE_rot, PE, Total_mech, friction_loss, reached_end]
for k = 1:numBodies
 energy_logs{k} = zeros(steps, 7);
 energy_logs{k}(:,7) = 1; % Initially all time points are valid
 
 % Add this line to store angular velocity components
 angVel_logs{k} = zeros(steps, 3); % Store x, y, z components
 velocity_logs{k} = zeros(steps,1);
end
% Track when bodies reach end of slope
reached_end = zeros(1, numBodies);
% Previous contact state (for friction tracking)
prev_contact = zeros(1, numBodies);
friction_work = zeros(1, numBodies); % Cumulative work done by friction
%% ---------------- MAIN LOOP ------------------------------------------
for step = 1:steps
 %% ---- store pre-collision state for friction calculation --------
 for k = 1:numBodies
 pre_vel{k} = bodies(k).vel;
 pre_angVel{k} = bodies(k).angVel;
 end
 
 %% ---- integrate free motion -------------------------------------
 for k = 1:numBodies
 bodies(k).vel = bodies(k).vel + dt*g;
 bodies(k).pos = bodies(k).pos + dt*bodies(k).vel;
 end
 
 %% ---- body–slope collisions -------------------------------------
 %% ---- body–slope collisions -------------------------------------
for k = 1:numBodies
 % Store contact state before collision
 is_on_slope = (bodies(k).pos(1)>=0 && bodies(k).pos(1)<=prismLen && ...
 abs(bodies(k).pos(2))<=halfW && ...
 dot(nSlope,bodies(k).pos)+dSlope-bodies(k).R < 0);
 
 % Process collision
 bodies(k) = collideTop(bodies(k), nSlope, dSlope, prismLen, halfW, coefRest, mu);
 
 % Calculate friction work if contact happened
 if is_on_slope
 % Get the contact point velocity
 r = -bodies(k).R * nSlope; % vector from CM to contact point
 vcp = bodies(k).vel + cross(bodies(k).angVel, r); % velocity at contact point
 
 % Calculate sliding velocity (tangential component)
 v_slide = vcp - dot(vcp, nSlope) * nSlope; % Tangential velocity component
 v_slide_mag = norm(v_slide);
 
 % Calculate approximate normal force (based on impulse)
 delta_v = bodies(k).vel - pre_vel{k};
 normal_impulse = dot(delta_v, nSlope) * bodies(k).mass;
 normal_force = abs(normal_impulse) / dt; % Approximate normal force
 
 % Calculate friction force
 friction_force = mu * normal_force;
 
 % Calculate work done by friction (F·dx)
 friction_work_step = friction_force * v_slide_mag * dt;
 
 % Accumulate only when sliding occurs
 if v_slide_mag > 1e-6
 friction_work(k) = friction_work(k) + friction_work_step;
 end
 
 prev_contact(k) = 1;
 else
 prev_contact(k) = 0;
 end
end
 
 
 %% ---- graphics refresh ------------------------------------------
 if mod(step,30) == 0 % draw every 4th step for speed
 for k = 1:numBodies
 updateBody(bodies(k), CYL_LEN_FAC);
 end
 drawnow;
 end
 
 %% ---- energy calculations and end-of-slope detection ------------
 currentTime = (step-1)*dt; % current time
 
 for k = 1:numBodies
 % Add this line to store angular velocity at each step
 angVel_logs{k}(step, :) = bodies(k).angVel;
 velocity_logs{k}(step) = norm(bodies(k).vel);
 % Check if body has reached bottom of slope
 if bodies(k).pos(3) <= slopeBottomThreshold && bodies(k).pos(1) >= prismLen*0.8 && ~reached_end(k)
 reached_end(k) = step; % Mark the step when body reached end
 end
 
 % If we're past the point when this body reached the end, mark as invalid
 if reached_end(k) > 0 && step > reached_end(k)
 energy_logs{k}(step:end, 7) = 0;
 end
 
 
 % Store time
 energy_logs{k}(step, 1) = currentTime;
 
 % Translational KE: 1/2 * m * v^2
 ke_trans = 0.5 * bodies(k).mass * sum(bodies(k).vel.^2);
 
 % Rotational KE: 1/2 * I * ω^2
 ke_rot = 0.5 * bodies(k).I * sum(bodies(k).angVel.^2);
 
 % Potential energy: m * g * h (using Z=0 as reference)
 pe = -bodies(k).mass * g(3) * bodies(k).pos(3);
 
 % Store energy components
 energy_logs{k}(step, 2) = ke_trans;
 energy_logs{k}(step, 3) = ke_rot;
 energy_logs{k}(step, 4) = pe;
 energy_logs{k}(step, 5) = ke_trans + ke_rot + pe; % mechanical energy
 energy_logs{k}(step, 6) = friction_work(k); % cumulative friction work
 end
end
%% ---------------- REVISED ENERGY PLOTS --------------------------------------
bodyNames = {'Hollow Sphere', 'Solid Sphere', 'Hollow Cylinder', 'Solid Cylinder'};
bodyColors = {[0.9 0.4 0.2], [0.2 0.6 0.9], [0.9 0.1 0.1], [0.1 0.8 0.1]}; % Orange, Blue, Red, Green
% Skip Figure 2 (Hollow Cylinder Energy)
% Create a new diagnostic figure showing all translational KE separately
figure('Name', 'Translational KE Comparison', 'Position', [50, 50, 800, 600], 'Color', 'w');
% Plot translational KE for all bodies with thick lines
hold on;
for k = 1:numBodies
 valid_indices = find(energy_logs{k}(:, 7) == 1);
 valid_time = energy_logs{k}(valid_indices, 1);
 ke_trans = energy_logs{k}(valid_indices, 2);
 
 % Use very thick lines
 plot(valid_time, ke_trans, 'Color', bodyColors{k}, 'LineWidth', 3);
end
title('Translational Kinetic Energy Comparison', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Energy (J)', 'FontSize', 12);
legend(bodyNames, 'Location', 'best', 'FontSize', 10);
grid on;
hold off;
% Create a figure with 4 subplots (one for each body)
figure('Name', 'Energy Breakdown by Body', 'Position', [50, 50, 1200, 900], 'Color', 'w');
for k = 1:numBodies
 subplot(2, 2, k);
 
 % Get valid data until body reached end of slope
 valid_indices = find(energy_logs{k}(:, 7) == 1);
 valid_time = energy_logs{k}(valid_indices, 1);
 
 % Extract energy components directly
 ke_trans = energy_logs{k}(valid_indices, 2); % Translational KE
 ke_rot = energy_logs{k}(valid_indices, 3); % Rotational KE
 pe = energy_logs{k}(valid_indices, 4); % Potential energy
 mech_energy = energy_logs{k}(valid_indices, 5); % Total mechanical energy
 friction = abs(energy_logs{k}(valid_indices, 6)); % Absolute value of friction energy
 
 % For the hollow cylinder (k=3), use a special style for translational KE
 if k == 3
 % Plot thick translational KE line first
 plot(valid_time, ke_trans, 'b-', 'LineWidth', 4, 'DisplayName', 'Translational KE');
 hold on;
 
 % Plot other components
 plot(valid_time, pe, 'g-', 'LineWidth', 2, 'DisplayName', 'Potential Energy');
 plot(valid_time, ke_rot, 'r-', 'LineWidth', 2, 'DisplayName', 'Rotational KE');
 plot(valid_time, mech_energy, 'k-', 'LineWidth', 2, 'DisplayName', 'Total Mechanical');
 
 % Plot total energy (mechanical + friction)
 total_energy = mech_energy + friction;
 plot(valid_time, total_energy, 'm--', 'LineWidth', 2, 'DisplayName', 'Total System (Mech+Friction)');
 else
 % Normal plotting for other bodies
 plot(valid_time, pe, 'g-', 'LineWidth', 2, 'DisplayName', 'Potential Energy');
 hold on;
 plot(valid_time, ke_trans, 'b-', 'LineWidth', 2, 'DisplayName', 'Translational KE');
 plot(valid_time, ke_rot, 'r-', 'LineWidth', 2, 'DisplayName', 'Rotational KE');
 plot(valid_time, mech_energy, 'k-', 'LineWidth', 2, 'DisplayName', 'Total Mechanical');
 
 % Plot total energy (mechanical + friction)
 total_energy = mech_energy + friction;
 plot(valid_time, total_energy, 'm--', 'LineWidth', 2, 'DisplayName', 'Total System (Mech+Friction)');
 end
 
 hold off;
 
 title(sprintf('%s Energy', bodyNames{k}), 'Color', bodyColors{k}, 'FontSize', 12);
 xlabel('Time (s)', 'FontSize', 11);
 ylabel('Energy (J)', 'FontSize', 11);
 legend('Location', 'best', 'FontSize', 9);
 grid on;
end
% Create a figure showing friction losses
figure('Name', 'Friction Energy', 'Position', [50, 50, 800, 600], 'Color', 'w');
% Plot friction losses for all bodies
for k = 1:numBodies
 valid_indices = find(energy_logs{k}(:, 7) == 1);
 valid_time = energy_logs{k}(valid_indices, 1);
 friction_data = abs(energy_logs{k}(valid_indices, 6)); % Use absolute value
 
 plot(valid_time, friction_data, 'Color', bodyColors{k}, 'LineWidth', 3);
 hold on;
end
hold off;
title('Energy Converted to Heat by Friction', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Energy (J)', 'FontSize', 12);
legend(bodyNames, 'Location', 'best', 'FontSize', 10);
grid on;
% Create angular momentum plot
figure('Name', 'Angular Momentum by Body', 'Position', [50, 50, 800, 600], 'Color', 'w');
for k = 1:numBodies
 valid_indices = find(energy_logs{k}(:, 7) == 1);
 valid_time = energy_logs{k}(valid_indices, 1);
 
 % Calculate angular momentum vector components
 L_x = bodies(k).I * angVel_logs{k}(valid_indices, 1);
 L_y = bodies(k).I * angVel_logs{k}(valid_indices, 2);
 L_z = bodies(k).I * angVel_logs{k}(valid_indices, 3);
 
 % Calculate magnitude of angular momentum
 L_mag = sqrt(L_x.^2 + L_y.^2 + L_z.^2);
 
 % Plot both magnitude and y-component
 plot(valid_time, L_mag, 'Color', bodyColors{k}, 'LineWidth', 3);
 hold on;
end
title('Angular Momentum Magnitude by Body Type', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angular Momentum (kg·m²/s)', 'FontSize', 12);
legend(bodyNames, 'Location', 'best', 'FontSize', 10);
grid on;
% Create a figure with 4 subplots showing only total energy for each body
figure('Name', 'Total Energy by Body Type', 'Position', [50, 50, 1200, 900], 'Color', 'w');
for k = 1:numBodies
 subplot(2, 2, k);
 
 % Get valid data until body reached end of slope
 valid_indices = find(energy_logs{k}(:, 7) == 1);
 valid_time = energy_logs{k}(valid_indices, 1);
 
 % Calculate total energy (mechanical + friction)
 mech_energy = energy_logs{k}(valid_indices, 5);
 friction = abs(energy_logs{k}(valid_indices, 6));
 total_energy = mech_energy + friction;
 
 % Plot total energy with thick line
 plot(valid_time, total_energy, 'Color', bodyColors{k}, 'LineWidth', 3);
 
 % Add reference line for initial energy
 initial_energy = energy_logs{k}(1, 5); % Initial mechanical energy
 yline(initial_energy, 'r--', 'Initial Energy', 'LineWidth', 1.5);
 
 title(sprintf('%s Total Energy', bodyNames{k}), 'Color', bodyColors{k}, 'FontSize', 12);
 xlabel('Time (s)', 'FontSize', 11);
 ylabel('Energy (J)', 'FontSize', 11);
 grid on;
 
 % Set y-axis limits to be consistent across all subplots
 % Add a small buffer (5%) above and below the data range
 ylim_min = min(total_energy) * 0.95;
 ylim_max = max(total_energy) * 1.05;
 ylim([ylim_min ylim_max]);
end
sgtitle('Total System Energy (Mechanical + Friction) by Body Type', 'FontSize', 14, 'FontWeight', 'bold');
%% =====================================================================
%% FUNCTION DEFINITIONS ------------------------------------------------
function b = makeSphere(R,m,pos,type)
 if nargin<4, type='solid'; end
 Icoeff = strcmpi(type,'hollow')*2/3 + strcmpi(type,'solid')*2/5;
 b = initBody('sphere',R,m,Icoeff,pos,type);
end
function b = makeCylinder(R,m,pos,type)
 if nargin<4, type='solid'; end
 Icoeff = strcmpi(type,'hollow')*1 + strcmpi(type,'solid')*1/2;
 b = initBody('cylinder',R,m,Icoeff,pos,type);
end
function b = initBody(shape,R,m,Ic,pos,type)
 b.shape = shape; % 'sphere' | 'cylinder'
 b.R = R; % radius
 b.mass = m;
 b.I = Ic*m*R^2; % moment of inertia (about CM)
 b.kind = type; % 'solid' | 'hollow'
 b.pos = pos(:)';
 b.vel = [0 0 0];
 b.angVel = [0 0 0];
 b.h = []; % main surface handle
 b.capHandles = []; % 2×patch handles for cylinder caps
 b.capData = struct(); % cached circle template for caps
end
%% ---------------- COLLISION ROUTINES ---------------------------------
function b = collideTop(b,n,d,len,halfW,e,mu)
 % Only test against the sloped top if the XY lies within the prism.
 if b.pos(1)>=0 && b.pos(1)<=len && abs(b.pos(2))<=halfW
 b = collidePlane(b,n,d,e,mu);
 end
end
function b = collidePlane(b,n,d,e,mu)
 dist = dot(n,b.pos)+d - b.R; % signed distance from CM to surface minus R
 if dist<0 % penetration
 b.pos = b.pos - dist*n; % project CM out of the plane
 r = -b.R*n; % contact‑point vector (CM → contact)
 vcp = b.vel + cross(b.angVel,r); % velocity of contact point
 vn = dot(vcp,n); % normal component
 vt = vcp - vn*n; % tangential component
 
 % normal impulse for restitution
 Jn = -(1+e)*vn / (1/b.mass);
 impulse = Jn*n;
 
 % tangential (Coulomb) impulse for friction / rolling
 if norm(vt) > 1e-6
 k_t = 1/b.mass + (dot(r,r) - (dot(r,n))^2)/b.I;
 Jt = min(mu*abs(Jn), norm(vt)/k_t);
 impulse = impulse - Jt*vt/norm(vt);
 end
 
 % update linear & angular momentum
 b.vel = b.vel + impulse/b.mass;
 b.angVel = b.angVel + cross(r,impulse)/b.I;
 end
end
%% ---------------- DRAWING & UPDATE ------------------------------------
function b = drawBody(b,lenFac)
 switch b.shape
 case 'sphere'
 persistent sx sy sz
 if isempty(sx); [sx,sy,sz] = sphere(20); end
 b.h = surf(b.R*sx + b.pos(1), ...
 b.R*sy + b.pos(2), ...
 b.R*sz + b.pos(3), ...
 'FaceColor', sphereColor(b), ...
 'EdgeColor', 'none', ...
 'FaceLighting', 'gouraud');
 case 'cylinder'
 persistent cx cy cz
 if isempty(cx)
 [cx,cy,cz] = cylinder(1,30); cz = cz - 0.5; % unit len = 1
 end
 len = lenFac*b.R;
 % main tube
 b.h = surf(b.R*cx + b.pos(1), ...
 len*cz + b.pos(2), ...
 b.R*cy + b.pos(3), ...
 'FaceColor', cylColor(b), ...
 'EdgeColor', 'none', ...
 'FaceLighting', 'gouraud');
 % --- end‑caps only for solid cylinders -------------------
 if strcmpi(b.kind,'solid')
 nPts = size(cx,2);
 % store template circle (XY‑plane, centred on origin)
 b.capData.cx = cx(1,:); % cosθ
 b.capData.cy = cy(1,:); % sinθ
 b.capData.nPts = nPts;
 b.capData.len = len;
 [topX,topY,topZ,botX,botY,botZ] = getCapsCoords(b);
 topCap = patch('XData',topX,'YData',topY,'ZData',topZ, ...
 'FaceColor',cylColor(b),'EdgeColor','none','FaceLighting','gouraud');
 botCap = patch('XData',botX,'YData',botY,'ZData',botZ, ...
 'FaceColor',cylColor(b),'EdgeColor','none','FaceLighting','gouraud');
 b.capHandles = [topCap botCap];
 end
 end
end
function updateBody(b,lenFac)
 switch b.shape
 case 'sphere'
 persistent sx sy sz
 if isempty(sx); [sx,sy,sz] = sphere(20); end
 set(b.h,'XData', b.R*sx + b.pos(1), ...
 'YData', b.R*sy + b.pos(2), ...
 'ZData', b.R*sz + b.pos(3));
 case 'cylinder'
 persistent cx cy cz
 if isempty(cx); [cx,cy,cz] = cylinder(1,30); cz = cz - 0.5; end
 len = lenFac*b.R;
 set(b.h, 'XData', b.R*cx + b.pos(1), ...
 'YData', len*cz + b.pos(2), ...
 'ZData', b.R*cy + b.pos(3));
 % --- move caps if present --------------------------------
 if strcmpi(b.kind,'solid') && ~isempty(b.capHandles)
 [topX,topY,topZ,botX,botY,botZ] = getCapsCoords(b);
 set(b.capHandles(1), 'XData', topX, 'YData', topY, 'ZData', topZ);
 set(b.capHandles(2), 'XData', botX, 'YData', botY, 'ZData', botZ);
 end
 end
end
% Helper: compute the two cap polygons in world coordinates ------------
function [topX,topY,topZ,botX,botY,botZ] = getCapsCoords(b)
 cx = b.capData.cx; % unit circle X (cosθ)
 cy = b.capData.cy; % unit circle Z (sinθ) — MATLAB Y axis draws „along cylinder"
 nPts = b.capData.nPts;
 len = b.capData.len;
 % --- quick & dirty roll‑angle estimate ---------------------------
 rollAngle = atan2(b.vel(3), b.vel(1)); % crude: aligns with velocity
 if norm(b.angVel) > 1e-6 % prefer actual angular motion
 rollAngle = mod(rollAngle + norm(b.angVel)*0.1, 2*pi);
 end
 % rotate template circle around body‑Y (local axis)
 rotX = cos(rollAngle)*cx - sin(rollAngle)*cy;
 rotZ = sin(rollAngle)*cx + cos(rollAngle)*cy;
 % TOP cap at +len/2; BOTTOM at –len/2 (body space)
 topYoff = len/2; botYoff = -len/2;
 topX = b.R*rotX + b.pos(1); botX = b.R*rotX + b.pos(1);
 topZ = b.R*rotZ + b.pos(3); botZ = b.R*rotZ + b.pos(3);
 topY = (topYoff + b.pos(2))*ones(1,nPts);
 botY = (botYoff + b.pos(2))*ones(1,nPts);
end
%% ---------------- COLOUR HELPERS -------------------------------------
function c = sphereColor(b)
 c = (b.I > 0.6*b.mass*b.R^2)*[0.9 0.4 0.2] + ... % hollow → orange
 (b.I <= 0.6*b.mass*b.R^2)*[0.2 0.6 0.9]; % solid → blue
end
function c = cylColor(b)
 c = (b.I > 0.75*b.mass*b.R^2)*[0.9 0.1 0.1] + ... % hollow → red
 (b.I <= 0.75*b.mass*b.R^2)*[0.1 0.8 0.1]; % solid → green
end
%% ---------------- SLOPE VISUAL ----------------------------------------
function plotPrism(ang,h,len,halfW)
 % vertices (X,Y,Z)
 dz = len*tan(ang);
 V = [0 -halfW h; 0 halfW h; len halfW h-dz; len -halfW h-dz; 0 -halfW h-dz; 0 halfW h-dz];
 F = {[1 2 3 4], [1 4 5], [2 3 6], [4 3 6 5]};
 for f = 1:numel(F)
 patch('Vertices',V, 'Faces',F{f}, 'FaceColor',[0.6 0.6 0.6], ...
 'EdgeColor','k','FaceLighting','gouraud');
 end
end
%% ---------------- VELOCITY PLOTS ------------------------------
% Translational Velocity Plot
figure('Name','Translational Velocity','Position',[50,50,800,600],'Color','w');
hold on;
for k = 1:numBodies
 valid = energy_logs{k}(:,7)==1; % only until they leave the slope
 t = energy_logs{k}(valid,1); % time vector
 vtrans = velocity_logs{k}(valid); % |v|
 plot(t, vtrans, 'LineWidth', 2);
end
title('Translational Velocity Comparison','FontSize',14);
xlabel('Time (s)','FontSize',12);
ylabel('v_{trans} (m/s)','FontSize',12);
legend(bodyNames,'Location','best');
grid on;
hold off;
% Angular Velocity Magnitude Plot
figure('Name','Angular Velocity','Position',[50,50,800,600],'Color','w');
hold on;
for k = 1:numBodies
 valid = energy_logs{k}(:,7)==1;
 t = energy_logs{k}(valid,1);
 omega = sqrt(sum(angVel_logs{k}(valid,:).^2,2)); % ||ω||
 plot(t, omega, 'LineWidth', 2);
end
title('Angular Velocity Magnitude Comparison','FontSize',14);
xlabel('Time (s)','FontSize',12);
ylabel('\omega_{mag} (rad/s)','FontSize',12);
legend(bodyNames,'Location','best');
grid on;
hold off;
%% ---------------- SLIP METRIC SUBPLOTS ------------------------------
figure('Name','Slip Metric','Position',[100,100,1200,800],'Color','w');

for k = 1:numBodies
 % pick out the valid times
 valid = energy_logs{k}(:,7)==1;
 t     = energy_logs{k}(valid,1);
 % compute ω and v
 omega = sqrt(sum(angVel_logs{k}(valid,:).^2,2));  % ||ω||
 vtrans = velocity_logs{k}(valid);                 % |v|
 % slip metric
 slip = omega * bodies(k).R - vtrans;              % m/s
 % subplot in a 2×2 layout
 subplot(2,2,k);
 plot(t, slip, 'LineWidth',2);
 grid on;
 xlabel('Time (s)','FontSize',11);
 ylabel('\omega R - v (m/s)','FontSize',11);
 title(bodyNames{k}, 'FontSize',12);
end

% Add a big title for the entire figure
sgtitle('Slip Metric: \omegaR - v_{trans} for Each Body','FontSize',14,'FontWeight','bold');