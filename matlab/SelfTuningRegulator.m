classdef SelfTuningRegulator < handle
    properties
        lambda_ % forgetting factor
        goal_state_ % desired goal state []
 
    end

    properties (Access = private)
        curr_state_ % Current joint angles vector
        prev_state_ % Previous joint angles vector
        prev_input_ % Previous inputs vector

        Theta_; % Parameter vector
        phi_    % History vector
        Phi_;   % History matrix
        P_;     % Covariance matrix
        L_;     % Gain matrix
        A_;     % State matrix
        B_;     % Input matrix
        
        num_joints_; 
        n_ % input history size
        m_ % output history size
        p; % system dimension
        r; % history dimension     
                
        % Frequency control for parameter updates
        update_counter_;
        update_frequency_;
        estimate_frequency_;

    end
    
    methods
        function this = SelfTuningRegulator(joints, input_dim, output_dim, lambda, goal_state, covariance, update_freq, estimate_freq)
            
            this.num_joints_ = joints;
            this.lambda_ = lambda;
            this.goal_state_ = goal_state;
            this.m_ = input_dim;
            this.n_ = output_dim;
            
            this.r = this.num_joints_ * this.n_ + this.num_joints_ * this.m_; % output size * n + input_size * m
            this.p = this.num_joints_ * this.r;
            
            % Set default frequencies
            if nargin < 7 || isempty(update_freq)
                this.update_frequency_ = 5;
            else
                this.update_frequency_ = update_freq;
            end
            
            if nargin < 8 || isempty(estimate_freq)
                this.estimate_frequency_ = 1;
            else
                this.estimate_frequency_ = estimate_freq;
            end
            
            this.update_counter_ = 0;

            this.curr_state_ = zeros(this.num_joints_, 1);
            this.prev_state_ = zeros(this.num_joints_*this.n_, 1);
            
            this.prev_input_ = zeros(this.num_joints_*this.m_, 1);
            
            % Parameter estimators
            this.Theta_ = 0.5 * ones(this.p, 1);
            this.phi_ = zeros(this.r, 1);
            this.Phi_ = zeros(this.num_joints_, this.p);

            this.L_ = zeros(this.p, this.num_joints_);
            this.P_ = covariance * eye(this.p);

            % State matrices
            this.A_ = zeros(this.num_joints_, this.num_joints_*this.n_);
            this.B_ = zeros(this.num_joints_, this.num_joints_*this.m_);

            fprintf('Self-tuning regulator initialized');
        end

        function update(this)
            
            % Update Gain
            denL = this.lambda_ * eye(this.num_joints_) + this.Phi_ * this.P_ * this.Phi_';
            
            % Add regularization to prevent singularity
            if rcond(denL) < 1e-12
                denL = denL + 1e-6 * eye(size(denL));
            end
            
            this.L_ = (this.P_ * this.Phi_') / denL;
            
            % update Parameter
            prediction_error = this.curr_state_ - this.Phi_ * this.Theta_;
            this.Theta_ = this.Theta_ + this.L_ * prediction_error;
            
            % Bound parameters
            this.Theta_ = max(-10, min(10, this.Theta_));
            
            % update Covariance
            this.P_ = (this.P_ - this.L_ * this.Phi_ * this.P_) / this.lambda_;
            
            % Ensure P remains positive definite
            [V, D] = eig(this.P_);
            D = diag(max(diag(D), 1e-6));  % Ensure positive eigenvalues
            this.P_ = V * D * V';
  
        end

        function estimate(this)

            % Construct A and B from Theta
            for i = 0:(this.num_joints_-1)
                this.A_(i+1, 1:this.num_joints_*this.n_) = this.Theta_(i*this.r + 1 : i*this.r + this.num_joints_*this.n_);
                this.B_(i+1, 1:this.num_joints_*this.m_) = this.Theta_(i*this.r + this.num_joints_*this.n_ + 1 : (i+1)*this.r);
            end
        end

        function construct_phi(this, angles, inputs)

            this.curr_state_ = angles;

            % Constructing the phi vector from previous history
            this.phi_ = [this.prev_state_; this.prev_input_];
            
            % updating the previous history with new feedback
            this.prev_state_ = [this.curr_state_; this.prev_state_];
            this.prev_state_ = this.prev_state_(1:this.num_joints_*this.n_);

            this.prev_input_ = [inputs; this.prev_input_];
            this.prev_input_ = this.prev_input_(1:this.num_joints_*this.m_);

            % Construct the Phi Matrix from phi vector
            for i = 1 : this.num_joints_
                this.Phi_(i, (1+(i-1)*this.r) : i*this.r) = this.phi_;
            end

        end

        function input = computeControl(this, angles, inputs)
            
            if length(angles) ~= this.num_joints_ || length(inputs) ~= this.num_joints_
                error('Wrong dimensions! angles and inputs must be %d-element vectors', this.num_joints_);
            end

            % Increment counter
            this.update_counter_ = this.update_counter_ + 1;

            % Always append new variables and construct the History matrix
            this.construct_phi(angles, inputs);

            % Update System Matrices
            if mod(this.update_counter_, this.estimate_frequency_) == 0
                this.estimate();
            end

            % Control Effort
            noise = 0.005*(1 - rand(1));
            % noise = 0;
            input = this.OneStepAheadController() + noise;

            % Update Parameter Estimation
            if mod(this.update_counter_, this.update_frequency_) == 0
                this.update();
            end

        end

        function input = OneStepAheadController(this)
            
            % yd = Ax + Bu
            % Bu = yd - Ax --> u = B^-1 (yd - Ax)s
            try
                % Compute desired control input
                error_signal = this.goal_state_ - this.A_ * this.prev_state_;
                
                % Check if B matrix is well-conditioned
                if rank(this.B_) < size(this.B_, 2)
                    % B matrix is rank deficient, use pseudo-inverse
                    input = pinv(this.B_) * error_signal;
                else
                    % B matrix is full rank, use normal solution
                    input = this.B_ \ error_signal;
                end
                
                % Ensure output is finite
                if any(~isfinite(input))
                    input = zeros(size(input));
                end
                
            catch ME
                % If anything fails, return zero control
                fprintf('OneStepAheadController error: %s\n', ME.message);
                input = zeros(this.num_joints_, 1);
            end

        end
        
    end

end