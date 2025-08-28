classdef Robot < handle
    % N-link Robot Manipulator (1-3 links supported)
    properties
        Links         % [L1, L2, ..., LN]
        Mass          % [m1, m2, ..., mN]
        Inertia       % [I1, I2, ..., IN]
        g
        numLinks      % Number of links (1, 2, or 3)
    end

    properties (Access = private)
        JointAngle    % [q1, q2, ..., qN]
    end

    methods
        % Constructor
        function this = Robot(links, mass, initial_joint_angles)
            if nargin < 1
                error('Link lengths must be specified.');
            end
            
            % Validate number of links (1-3 supported)
            numLinks = length(links);
            if numLinks < 1 || numLinks > 3
                error('Robot supports only 1, 2, or 3 links. Received %d links.', numLinks);
            end
            
            % Validate mass input
            if length(mass) ~= numLinks
                error('Number of masses (%d) must match number of links (%d).', length(mass), numLinks);
            end
            
            this.numLinks = numLinks;
            this.Links = links;
            this.Mass = mass;
            this.g = 9.81;

            % Calculate inertias assuming uniform rods (I = (1/12)*m*L^2)
            this.Inertia = (1/12) * mass .* (links.^2);

            if nargin < 3
                % Default joint angles to zeros
                this.JointAngle = zeros(1, numLinks);
            else
                % Validate initial joint angles
                if length(initial_joint_angles) ~= numLinks
                    error('Number of initial joint angles (%d) must match number of links (%d).', length(initial_joint_angles), numLinks);
                end
                this.JointAngle = initial_joint_angles;
            end
        end

        % Setter for Joint Angles
        function setJointAngle(this, value)
            this.JointAngle = value;
        end

        % Getter for Joint Angles
        function value = getJointAngle(this)
            value = this.JointAngle;
        end

        %% Forward Kinematics - Simplified for 3 links
        function FK = forward_kinematics(this, dh_table)
            FK = eye(4);
            for i = 1:length(dh_table(:,1))
                H = FK_helper(this, dh_table(i, :));
                FK = FK*H;
            end
            FK = simplify(FK);
        end

        % FK Helper Function
        function H = FK_helper(this, link_params)
            a = link_params(1);
            alpha = link_params(2);
            d = link_params(3);
            theta = link_params(4);
            
            H = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                          0,             sin(alpha),             cos(alpha),            d;
                          0,                      0,                      0,            1];
        end

        % Numeric FK Calculation
        function T = Numeric_FK(this, FK, q_des)
            % Define symbolic variables for joint angles and link lengths
            syms theta1 theta2 theta3 L1 L2 L3

            % Extract link lengths
            links = this.Links;

            % Substitute the joint angles (q_des) into the symbolic FK expression
            P = subs(FK, [theta1, theta2, theta3], q_des);

            % Substitute the link lengths into the FK expression
            P = subs(P, [L1, L2, L3], links);

            % Simplify and evaluate the result
            T = double(vpa(P)); % Use vpa to handle symbolic numbers before conversion
        end

        %% Position Calculation
        % Position of the end of Link 1 (first joint)
        function posA = calcPosA(this)
            L = this.Links;
            q = this.JointAngle;
            
            if this.numLinks >= 1
                % Link 1 rotates about Y-axis from vertical (-Z direction)
                % q1 = 0 means link points straight down (-Z)
                x = L(1) * sin(q(1));      % X displacement
                y = 0;                     % Always zero (2D planar motion)
                z = -L(1) * cos(q(1));     % Z displacement (negative because hanging down)
                posA = [x; y; z];
            else
                posA = [0; 0; 0];
            end
        end

        % Position of the end of Link 2 (second joint)
        function posB = calcPosB(this)
            if this.numLinks >= 2
                L = this.Links;
                q = this.JointAngle;

                % Get position A (end of link 1)
                posA = this.calcPosA();
                
                % Link 2 continues from Link 1 with relative angle q2
                % Total angle of link 2 from vertical is q1 + q2
                theta2_abs = q(1) + q(2);  % Absolute angle of link 2 from vertical
                
                x = posA(1) + L(2) * sin(theta2_abs);
                y = 0;  % Always zero (2D planar motion)
                z = posA(3) - L(2) * cos(theta2_abs);  % Continue downward motion
                
                posB = [x; y; z];
            else
                posB = [0; 0; 0];
            end
        end

        % Position of the end of Link 3 (end effector)
        function posC = calcPosC(this)
            if this.numLinks >= 3
                L = this.Links;
                q = this.JointAngle;

                % Get position B (end of link 2)
                posB = this.calcPosB();
                
                % Link 3 continues from Link 2 with relative angle q3
                % Total angle of link 3 from vertical is q1 + q2 + q3
                theta3_abs = q(1) + q(2) + q(3);  % Absolute angle of link 3 from vertical
                
                x = posB(1) + L(3) * sin(theta3_abs);
                y = 0;  % Always zero (2D planar motion)
                z = posB(3) - L(3) * cos(theta3_abs);  % Continue motion
                
                posC = [x; y; z];
            else
                posC = [0; 0; 0];
            end
        end

        % Get end effector position based on number of links
        function posEnd = getEndEffectorPos(this)
            switch this.numLinks
                case 1
                    posEnd = this.calcPosA();
                case 2
                    posEnd = this.calcPosB();
                case 3
                    posEnd = this.calcPosC();
                otherwise
                    error('Invalid number of links: %d', this.numLinks);
            end
        end

        %% Jacobian Calculation - Modified for variable links
        function J = calcJacobian(this, q)
            % Define small perturbation for numeric differentiation
            delta = 1e-6;

            % Preallocate Jacobian matrix
            J = zeros(3, length(q));

            % Get current end-effector position
            posEnd = this.getEndEffectorPos(); % Current end-effector position

            % Loop through each joint to compute partial derivatives
            for i = 1:length(q)
                % Create perturbed joint angle vector
                q_perturbed = q;
                q_perturbed(i) = q_perturbed(i) + delta;

                % Update joint angles
                this.setJointAngle(q_perturbed);

                % Compute perturbed end-effector position
                pos_perturbed = this.getEndEffectorPos();

                % Numeric differentiation (finite difference)
                J(:, i) = (pos_perturbed - posEnd) / delta;
            end

            % Reset joint angles to the original values
            this.setJointAngle(q);
        end

        %% Inverse Kinematics - Modified for 3 links
        function Thetas = IK(this, dx, dy, dz, phi)
            % Extract link lengths
            L = this.Links;

            % Calculate theta1
            theta1 = atan2(dy, dx);

            % Intermediate terms
            d = dx - L(3) * cos(theta1) * cos(phi);
            e = dy - L(3) * sin(theta1) * cos(phi);
            f = dz - L(1) - L(3) * sin(phi);

            % Calculate theta3
            cos_theta3 = (d^2 + e^2 + f^2 - L(2)^2 - L(3)^2) / (2 * L(2) * L(3));
            cos_theta3 = min(max(cos_theta3, -1), 1);
            theta3 = acos(cos_theta3);

            % Intermediate terms for theta2
            a = L(3) * sin(theta3);
            b = L(2) + L(3) * cos(theta3);
            c = dz - L(1) - L(3) * sin(phi);
            r = sqrt(a^2 + b^2);

            % Calculate theta2
            sqrt_term = max(0, r^2 - c^2); % Ensure no complex numbers
            theta2 = atan2(c, sqrt(sqrt_term)) - atan2(a, b);

            % Return joint angles
            Thetas = [theta1, theta2, theta3];
        end

        %% Trajectory Planning
        function [q_trajectory, q_dot_trajectory, q_ddot_trajectory, coeffs] = CubicPolynomialInterpolation(this, q_initial, q_final, t, n)
            T = linspace(0, t, n);
            num_joints = length(q_initial);
            q_trajectory = zeros(num_joints, n);
            q_dot_trajectory = zeros(num_joints, n);
            q_ddot_trajectory = zeros(num_joints, n);

            % Coefficients matrix A for cubic polynomial
            A = [0,  0,  0, 1;        % Position at t = 0
                 t^3, t^2, t, 1;      % Position at t = t
                 0,  0,  1, 0;        % Velocity at t = 0
                 3*t^2, 2*t, 1, 0];   % Velocity at t = t

            for i = 1:num_joints
                % Boundary conditions: positions and velocities
                B = [q_initial(i); q_final(i); 0; 0]; % Zero velocities at start and end

                % Solve for polynomial coefficients
                coeffs = A \ B;

                % Generate the trajectory for joint i
                q_trajectory(i, :) = coeffs(1) * T.^3 + coeffs(2) * T.^2 + ...
                                     coeffs(3) * T + coeffs(4);

                % Generate the velocities for joint i (first derivative)
                q_dot_trajectory(i, 2:end-1) = 3 * coeffs(1) * T(2:end-1).^2 + ...
                                               2 * coeffs(2) * T(2:end-1) + ...
                                               coeffs(3);

                % Generate the accelerations for joint i (second derivative)
                q_ddot_trajectory(i, 2:end-1) = 6 * coeffs(1) * T(2:end-1) + ...
                                               4 * coeffs(2);

                % Ensure zero velocity at start and end
                q_dot_trajectory(i, 1) = 0;
                q_dot_trajectory(i, end) = 0;
                q_ddot_trajectory(i, 1) = 0;
                q_ddot_trajectory(i, end) = 0;
            end
        end

        %% Robot Dynamics Methods

        % Compute inertia matrix M(q) for 2D planar robot
        function M = computeInertiaMatrix(this, q)
            if length(q) ~= this.numLinks
                error('Joint angles must be a %d-element vector for %d-link robot', this.numLinks, this.numLinks);
            end

            % Extract parameters
            L = this.Links;
            m = this.Mass;
            I = this.Inertia;
            n = this.numLinks;

            % Initialize mass matrix
            M = zeros(n, n);

            switch n
                case 1
                    % Simple 1-DOF pendulum
                    M(1,1) = I(1) + m(1)*(L(1)/2)^2;
                    
                case 2
                    % 2-DOF double pendulum (planar)
                    q2 = q(2);
                    c2 = cos(q2);
                    
                    % Standard planar double pendulum mass matrix
                    M(1,1) = m(1)*(L(1)/2)^2 + m(2)*(L(1)^2 + (L(2)/2)^2 + L(1)*L(2)*c2) + I(1) + I(2);
                    M(1,2) = m(2)*(L(2)/2)^2 + m(2)*L(1)*L(2)/2*c2 + I(2);
                    M(2,1) = M(1,2);  % Symmetric
                    M(2,2) = m(2)*(L(2)/2)^2 + I(2);
                    
                case 3
                    % 3-DOF triple pendulum (planar)
                    q2 = q(2); q3 = q(3);
                    c2 = cos(q2); c3 = cos(q3); c23 = cos(q2 + q3);
                    
                    % Standard planar triple pendulum mass matrix
                    M(1,1) = m(1)*(L(1)/2)^2 + m(2)*(L(1)^2 + (L(2)/2)^2 + L(1)*L(2)*c2) + ...
                             m(3)*(L(1)^2 + L(2)^2 + (L(3)/2)^2 + L(1)*L(2)*c2 + L(1)*L(3)*c23 + L(2)*L(3)*c3) + ...
                             I(1) + I(2) + I(3);
                             
                    M(1,2) = m(2)*(L(2)/2)^2 + m(2)*L(1)*L(2)/2*c2 + ...
                             m(3)*(L(2)^2 + (L(3)/2)^2 + L(1)*L(2)*c2 + L(2)*L(3)/2*c3 + L(1)*L(3)/2*c23) + ...
                             I(2) + I(3);
                             
                    M(1,3) = m(3)*(L(3)/2)^2 + m(3)*L(1)*L(3)/2*c23 + m(3)*L(2)*L(3)/2*c3 + I(3);
                    
                    M(2,1) = M(1,2);  % Symmetric
                    M(2,2) = m(2)*(L(2)/2)^2 + m(3)*(L(2)^2 + (L(3)/2)^2 + L(2)*L(3)*c3) + I(2) + I(3);
                    M(2,3) = m(3)*(L(3)/2)^2 + m(3)*L(2)*L(3)/2*c3 + I(3);
                    
                    M(3,1) = M(1,3);  % Symmetric
                    M(3,2) = M(2,3);  % Symmetric
                    M(3,3) = m(3)*(L(3)/2)^2 + I(3);
            end
        end

        % Compute Coriolis matrix C(q,q_dot) for 2D planar robot
        function C = computeCoriolisMatrix(this, q, q_dot)
            if length(q) ~= this.numLinks || length(q_dot) ~= this.numLinks
                error('Joint angles and velocities must be %d-element vectors', this.numLinks);
            end

            % Extract parameters
            L = this.Links;
            m = this.Mass;
            n = this.numLinks;

            % Initialize Coriolis matrix
            C = zeros(n, n);

            switch n
                case 1
                    % No Coriolis terms for 1-DOF pendulum
                    C = 0;
                    
                case 2
                    % 2-DOF double pendulum (planar)
                    q2 = q(2);
                    dq1 = q_dot(1); dq2 = q_dot(2);
                    s2 = sin(q2);
                    
                    % Standard planar double pendulum Coriolis terms
                    C(1,1) = 0;
                    C(1,2) = -m(2)*L(1)*L(2)/2*s2*dq2;
                    C(2,1) = m(2)*L(1)*L(2)/2*s2*dq1;
                    C(2,2) = 0;
                    
                case 3
                    % 3-DOF triple pendulum (planar)
                    q2 = q(2); q3 = q(3);
                    dq1 = q_dot(1); dq2 = q_dot(2); dq3 = q_dot(3);
                    s2 = sin(q2); s3 = sin(q3); s23 = sin(q2 + q3);

                    % Planar triple pendulum Coriolis terms
                    C(1,1) = 0;
                    C(1,2) = -m(2)*L(1)*L(2)/2*s2*dq2 - m(3)*L(1)*L(2)*s2*dq2 - m(3)*L(1)*L(3)/2*s23*(dq2 + dq3);
                    C(1,3) = -m(3)*L(1)*L(3)/2*s23*(dq2 + dq3) - m(3)*L(2)*L(3)/2*s3*(dq2 + dq3);
                    
                    C(2,1) = m(2)*L(1)*L(2)/2*s2*dq1 + m(3)*L(1)*L(2)*s2*dq1 + m(3)*L(1)*L(3)/2*s23*dq1;
                    C(2,2) = 0;
                    C(2,3) = -m(3)*L(2)*L(3)/2*s3*dq3;
                    
                    C(3,1) = m(3)*L(1)*L(3)/2*s23*dq1 + m(3)*L(2)*L(3)/2*s3*dq1;
                    C(3,2) = m(3)*L(2)*L(3)/2*s3*dq2;
                    C(3,3) = 0;
            end
        end

        % Compute gravity vector G(q) for 2D planar robot
        function G = computeGravityVector(this, q)
            if length(q) ~= this.numLinks
                error('Joint angles must be a %d-element vector', this.numLinks);
            end

            % Extract parameters
            L = this.Links;
            m = this.Mass;
            n = this.numLinks;

            % Initialize gravity vector
            G = zeros(n, 1);

            switch n
                case 1
                    % Simple 1-DOF pendulum
                    q1 = q(1);
                    s1 = sin(q1);
                    G(1) = -this.g*m(1)*L(1)/2*s1;
                    
                case 2
                    % 2-DOF double pendulum (planar)
                    q1 = q(1); q2 = q(2);
                    s1 = sin(q1); s12 = sin(q1 + q2);
                    
                    % Both joints feel gravitational restoring torques
                    G(1) = -this.g*(m(1)*L(1)/2*s1 + m(2)*(L(1)*s1 + L(2)/2*s12));
                    G(2) = -this.g*m(2)*L(2)/2*s12;
                    
                case 3
                    % 3-DOF triple pendulum (planar)
                    q1 = q(1); q2 = q(2); q3 = q(3);
                    s1 = sin(q1); s12 = sin(q1 + q2); s123 = sin(q1 + q2 + q3);
                    
                    % All joints contribute to gravitational restoring torques
                    G(1) = -this.g*(m(1)*L(1)/2*s1 + m(2)*(L(1)*s1 + L(2)/2*s12) + m(3)*(L(1)*s1 + L(2)*s12 + L(3)/2*s123));
                    G(2) = -this.g*(m(2)*L(2)/2*s12 + m(3)*(L(2)*s12 + L(3)/2*s123));
                    G(3) = -this.g*m(3)*L(3)/2*s123;
            end
        end

        % Forward dynamics: compute joint accelerations given torques
        % q_ddot = M^(-1) * (tau - C*q_dot - G - F_friction)
        function q_ddot = forwardDynamics(this, q, q_dot, tau)
            if length(q) ~= this.numLinks || length(q_dot) ~= this.numLinks || length(tau) ~= this.numLinks
                error('All inputs must be %d-element vectors', this.numLinks);
            end

            % Compute dynamics matrices
            M = this.computeInertiaMatrix(q);
            C = this.computeCoriolisMatrix(q, q_dot);
            G = this.computeGravityVector(q);
            
            % Add damping/friction for stability
            friction_coeff = 0.2;  % Viscous friction coefficient
            F_friction = friction_coeff * q_dot;

            % Forward dynamics equation with friction
            q_ddot = M \ (tau - C*q_dot - G - F_friction);
        end

        % Inverse dynamics: compute required torques given desired accelerations
        % tau = M*q_ddot + C*q_dot + G
        function tau = inverseDynamics(this, q, q_dot, q_ddot)
            if length(q) ~= this.numLinks || length(q_dot) ~= this.numLinks || length(q_ddot) ~= this.numLinks
                error('All inputs must be %d-element vectors', this.numLinks);
            end

            % Compute dynamics matrices
            M = this.computeInertiaMatrix(q);
            C = this.computeCoriolisMatrix(q, q_dot);
            G = this.computeGravityVector(q);

            % Inverse dynamics equation
            tau = M*q_ddot + C*q_dot + G;
        end

        % Simulate robot dynamics for one time step using ODE integration
        function [q_next, q_dot_next] = simulateStep(this, q, q_dot, tau, dt)
            % State vector: x = [q; q_dot]
            x0 = [q(:); q_dot(:)];

            % ODE function
            odefun = @(t, x) this.dynamicsODE(t, x, tau);

            % Integrate for one time step
            [~, x_out] = ode45(odefun, [0, dt], x0);
            x_final = x_out(end, :)';

            % Extract final state
            q_next = x_final(1:this.numLinks);
            q_dot_next = x_final(this.numLinks+1:2*this.numLinks);
            
            % Wrap joint angles to [-π, π] to prevent accumulation
            q_next = wrapToPi(q_next);
        end

        % ODE function for dynamics simulation
        function dx = dynamicsODE(this, t, x, tau)
            q = x(1:this.numLinks);
            q_dot = x(this.numLinks+1:2*this.numLinks);

            % Compute acceleration using forward dynamics
            q_ddot = this.forwardDynamics(q, q_dot, tau);

            % State derivative
            dx = [q_dot; q_ddot];
        end

        %% Animation Function
        function animate(this, q_trajectory, targets)
            % Check trajectory data
            if isempty(q_trajectory)
                error('Trajectory data is empty');
            end
            fprintf('Animation: trajectory has %d frames\n', size(q_trajectory, 2));
            
            q_range = [min(q_trajectory, [], 2), max(q_trajectory, [], 2)];
            fprintf('Joint angle ranges:\n');
            for i = 1:this.numLinks
                fprintf('  q%d: [%.3f, %.3f] rad\n', i, q_range(i,1), q_range(i,2));
            end
            
            % Set up figure
            figure('Name', sprintf('%d-Link Planar Robot Animation', this.numLinks));
            hold on;
            axis equal;
            grid on;
            
            % Calculate axis limits
            total_reach = sum(this.Links);
            lim = total_reach * 1.2;
            
            xlim([-lim, lim]); ylim([-0.2, lim]); % Z from slightly below 0 to +lim (upper quadrants)
            xlabel('X [m]'); ylabel('Z [m]');
            title(sprintf('%d-Link Planar Robot (2D Motion)', this.numLinks));
            
            % Add reference lines
            plot([-lim, lim], [0, 0], 'k--', 'LineWidth', 1); % Ground line
            plot([0, 0], [-0.2, lim], 'k--', 'LineWidth', 1); % Vertical reference

            % Number of frames in the trajectory
            num_frames = size(q_trajectory, 2);
            
            % Initialize plot handles
            colors = {'k', 'r', 'b'};
            link_handles = cell(this.numLinks, 1);
            joint_handles = cell(this.numLinks + 1, 1); % +1 for base
            
            % Create link handles
            for i = 1:this.numLinks
                link_handles{i} = plot([0, 0], [0, 0], [colors{i}, '-'], 'LineWidth', 4);
            end
            
            % Create joint handles
            joint_handles{1} = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % base
            for i = 2:this.numLinks + 1
                marker_color = colors{min(i-1, 3)};
                if i == this.numLinks + 1
                    marker_color = 'g'; % end effector in green
                end
                joint_handles{i} = plot(0, 0, [marker_color, 'o'], 'MarkerSize', 6, 'MarkerFaceColor', marker_color);
            end

            % Target position (equilibrium)
            target_handle = [];
            if nargin > 2 && ~isempty(targets)
                if size(targets, 2) == 1  % Column vector
                    targets = targets';
                end
                % Calculate equilibrium position
                this.setJointAngle(targets);
                posEnd_eq = this.getEndEffectorPos();
                if this.numLinks == 1
                    target_handle = plot(posEnd_eq(1), posEnd_eq(3), 'g*', 'MarkerSize', 15);
                    text(posEnd_eq(1), posEnd_eq(3) + 0.05, 'Target', 'FontSize', 8, 'Color', 'g');
                else
                    target_handle = plot3(posEnd_eq(1), posEnd_eq(2), posEnd_eq(3), 'g*', 'MarkerSize', 15);
                    text(posEnd_eq(1), posEnd_eq(2), posEnd_eq(3) + 0.05, 'Target', 'FontSize', 8, 'Color', 'g');
                end
            end
            
            % Draw initial frame
            this.setJointAngle(q_trajectory(:, 1)');
            this.updateAnimationFrame(link_handles, joint_handles);
            drawnow;

            % Animation loop
            skip_factor = max(1, floor(num_frames / 1000));
            fprintf('Animation skip factor: %d (showing every %d frames)\n', skip_factor, skip_factor);
            
            for i = 1:skip_factor:num_frames
                % Update joint angles
                this.setJointAngle(q_trajectory(:, i)');
                
                % Update animation
                this.updateAnimationFrame(link_handles, joint_handles);

                % Update display with timing info
                if mod(i, 100) == 1
                    fprintf('Animation frame %d/%d (t=%.2fs)\n', i, num_frames, (i-1)*0.01);
                end
                drawnow limitrate;
                pause(0.02);
            end
        end
        
        function updateAnimationFrame(this, link_handles, joint_handles)
            % Get positions based on current joint angles
            positions = cell(this.numLinks + 1, 1);
            positions{1} = [0; 0; 0]; % Base position
            
            % Calculate link end positions
            if this.numLinks >= 1
                positions{2} = this.calcPosA();
            end
            if this.numLinks >= 2
                positions{3} = this.calcPosB();
            end
            if this.numLinks >= 3
                positions{4} = this.calcPosC();
            end
            
            % Update links
            for i = 1:this.numLinks
                pos_start = positions{i};
                pos_end = positions{i+1};
                
                set(link_handles{i}, 'XData', [pos_start(1), pos_end(1)], ...
                                     'YData', [pos_start(3), pos_end(3)]);
            end
            
            % Update joint markers
            for i = 1:this.numLinks + 1
                pos = positions{i};
                set(joint_handles{i}, 'XData', pos(1), 'YData', pos(3));
            end
        end
    end
end
