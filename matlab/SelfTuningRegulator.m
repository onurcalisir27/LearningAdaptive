classdef SelfTuningRegulator < handle
    properties
        
        curr_state_ % Current joint angles vector
        prev_states_ % Previous joint angles vector
        prev_inputs_ % Previous inputs vector

        Theta_;  % Parameter vector
        phi_;    % History vector
        P_;      % Covariance matrix
        L_;      % Gain matrix
        lambda_ % forgetting factor
        
        A_;      % State matrix
        B_;      % Input matrix
        Bpast_   % Past Input matrix
        Bc_         

        n_;      % state dimension
        m_;      % input dimension
        p_;      % state history
        r_;      % input history
        s_;      % system dimension

        % Frequency control for parameter updates
        step_;
        update_frequency_;
        estimate_frequency_;

    end
    methods
        function this = SelfTuningRegulator(state_dim, input_dim, state_history, input_history, lambda, covariance, update_freq, estimate_freq)

            this.n_ = state_dim;
            this.m_ = input_dim;
            this.p_ = state_history;
            this.r_ = input_history;
            this.s_ = this.n_ * this.p_ + this.m_ * this.r_;

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

            this.step_ = 0;
            this.lambda_ = lambda;

            this.curr_state_ = zeros(this.n_, 1);
            this.prev_states_ = zeros(this.n_ * this.p_, 1);
            this.prev_inputs_ = zeros(this.m_ * this.r_, 1);

            % Parameter estimators
            this.Theta_ = 0.1 * ones(this.s_, this.n_);
            this.phi_ = zeros(this.s_, 1);
            this.L_ = zeros(this.s_, 1);
            this.P_ = covariance * eye(this.s_);

            % State matrices
            this.A_ = zeros(this.n_, this.n_*this.p_);
            this.B_ = zeros(this.n_, this.m_*this.r_);
            this.Bpast_ = zeros(this.n_, this.m_ * (this.r_ - 1));
            this.Bc_ = zeros(this.n_, this.m_);

            fprintf('Self-tuning regulator initialized');
        end

        function update(this)

            % Update Gain
            denL = this.lambda_ + this.phi_' * this.P_ * this.phi_;

            % Add regularization to prevent singularity
            if abs(denL) < 1e-12
                denL = denL + 1e-6;
            end

            this.L_ = (this.P_ * this.phi_) / denL;

            % update Parameters
            prediction_error = this.curr_state_ - this.Theta_' * this.phi_;
            this.Theta_ = this.Theta_ + this.L_ * prediction_error';

            % Bound parameters
            this.Theta_ = max(-10, min(10, this.Theta_));

            % update Covariance
            this.P_ = (this.P_ - this.L_ * this.phi_' * this.P_) / this.lambda_;

            % Ensure P remains positive definite
            [V, D] = eig(this.P_);
            D = diag(max(diag(D), 1e-6));
            this.P_ = V * D * V';
        end

        function input = computeControl(this, goal_state, state, prev_input)

            if length(state) ~= this.n_ || length(prev_input) ~= this.m_
                error('Wrong dimensions! angles and inputs must be %d-element vectors', this.n_, this.m_);
            end

            % Store current state for parameter update
            this.curr_state_ = state;

            % Always append new variables and construct the vector
            this.phi_ = [this.prev_states_; this.prev_inputs_];
            this.prev_states_ = [state; this.prev_states_(1:this.n_*(this.p_-1))];
            this.prev_inputs_ = [prev_input; this.prev_inputs_(1:this.m_*(this.r_-1))];

            % Update System Matrices
            if mod(this.step_, this.estimate_frequency_) == 0
                params = this.Theta_';
                this.A_ = params(1:this.n_, 1:(this.n_*this.p_));
                this.B_ = params(1:this.n_, (this.n_*this.p_+1):(this.n_*this.p_+this.m_*this.r_));
                if this.r_ > 1
                    this.Bpast_ = this.B_(1:this.n_, (this.m_+1):this.m_*this.r_);
                end

            end

            % Control Effort
            % noise = 0.005*rand(this.m_, 1);
            noise = 0;
            input = this.OneStepAheadController(goal_state) + noise;

            % Update Parameter Estimation
            if mod(this.step_, this.update_frequency_) == 0
                this.update();
            end

            % Increment counter
            this.step_ = this.step_ + 1;
        end

        function input = OneStepAheadController(this, goal_state)

            % yd = Ax + Bu
            % Bu = yd - Ax --> u = B^-1 (yd - Ax)
            try
                % Compute desired control input
                estimated_state = this.A_*this.prev_states_;
                if this.r_ > 1
                    estimated_state = estimated_state + this.Bpast_*this.prev_inputs_(this.m_+1:this.r_*this.m_);
                end 
                
                error_signal = goal_state - estimated_state;

                % Check if B matrix is well-conditioned
                this.Bc_ = this.B_(1:this.n_, 1:this.m_);
                if rank(this.Bc_) < size(this.Bc_, 2)
                    % B matrix is rank deficient, use pseudo-inverse
                    input = pinv(this.Bc_) * error_signal;
                else
                    % B matrix is full rank, use normal solution
                    input = this.Bc_ \ error_signal;
                end

                % Ensure output is finite
                if any(~isfinite(input))
                    input = zeros(size(input));
                end

            catch ME
                % If anything fails, return zero control
                fprintf('OneStepAheadController error: %s\n', ME.message);
                input = zeros(this.m_, 1);
            end

        end

        function showResults(this)
            fprintf('\n=== SELF-TUNING REGULATOR RESULTS ===\n');
            fprintf('Total steps completed: %d\n', this.step_);
            fprintf('Lambda (forgetting factor): %.3f\n', this.lambda_);
            fprintf('Update frequency: %d steps\n', this.update_frequency_);
            fprintf('Estimate frequency: %d steps\n', this.estimate_frequency_);

            fprintf('\nSystem Dimensions:\n');
            fprintf('  State dimension (n): %d\n', this.n_);
            fprintf('  Input dimension (m): %d\n', this.m_);
            fprintf('  State history (p): %d\n', this.p_);
            fprintf('  Input history (r): %d\n', this.r_);
            fprintf('  Total parameters (s): %d\n', this.s_);

            fprintf('\nLearned System Matrices:\n');
            fprintf('State Matrix A (%dx%d):\n', size(this.A_, 1), size(this.A_, 2));
            disp(this.A_);

            fprintf('Input Matrix B (%dx%d):\n', size(this.B_, 1), size(this.B_, 2));
            disp(this.B_);

            if this.r_ > 1
                fprintf('Past Input Matrix Bpast (%dx%d):\n', size(this.Bpast_, 1), size(this.Bpast_, 2));
                disp(this.Bpast_);
            end

            fprintf('Current Input Matrix Bc (%dx%d):\n', size(this.Bc_, 1), size(this.Bc_, 2));
            disp(this.Bc_);

            fprintf('\nParameter Estimation Results:\n');
            fprintf('Parameter Matrix Theta (%dx%d):\n', size(this.Theta_, 1), size(this.Theta_, 2));
            disp(this.Theta_);

            fprintf('Covariance Matrix P (condition number: %.2e):\n', cond(this.P_));
            fprintf('  P matrix size: %dx%d\n', size(this.P_, 1), size(this.P_, 2));
            fprintf('  P eigenvalues range: [%.2e, %.2e]\n', min(eig(this.P_)), max(eig(this.P_)));

            fprintf('\nCurrent State Information:\n');
            fprintf('Current state:\n');
            disp(this.curr_state_');
            fprintf('Previous states (history):\n');
            disp(this.prev_states_');
            fprintf('Previous inputs (history):\n');
            disp(this.prev_inputs_');
            fprintf('Current phi vector:\n');
            disp(this.phi_');
        end

    end

end
