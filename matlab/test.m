clear;
clc;

% Phi = zeros(3,18);
% phi = [1, 2, 3, 4, 5, 6];
% 
% Phi(1, 1:6) = phi;
% Phi(2, 7:12) = phi;
% Phi(3, 13:end) = phi;
% 
% P = 100000 * eye(18);
% 
% result = Phi * P * Phi';
% disp(Phi);
% % disp(phi);
% % disp(P);
% disp(result);
n = 2;
angles = [0; 0; 0; 0];
prev_angles = [1; 1; 1; 1; 1; 1];
inputs = [5; 5; 5];

if size(angles) ~= 3 | size(inputs) ~= 3
    error("Wrong dimensions!");
end
phi = [prev_angles; inputs];

prev_angles = [angles; prev_angles];
prev_angles = prev_angles(1:3*n);

disp(phi)
disp(prev_angles)

tranposed = phi';
disp(tranposed)

% n = 2; m = 1;
% r = 3*n + 3*m;
% p = 3 *r;
% Theta = 1:p;
% Theta = Theta';
% 
% A = zeros(3, 3*n);
% B = zeros(3, 3*m);
% 
% % A(1, 1:3*n) = Theta(1:3*n);
% % A(2, 1:3*n) = Theta(r+1:r+3*n);
% % A(3, 1:3*n) = Theta(2*r+1:2*r+3*n);
% 
% % B(1, 1:3*m) = Theta(3*n+1:r);
% % B(2, 1:3*m) = Theta(r+3*n+1:2*r);
% % B(3, 1:3*m) = Theta(2*r+3*n+1:3*r);
% 
% 
% for i = 0:2
%     A(i+1, 1:3*n) = Theta(i*r + 1 : i*r + 3*n);
%     B(i+1, 1:3*m) = Theta(i*r + 3*n + 1 : (i+1)*r);
% end
% 
% disp(A)
% disp(B)

% x = rand(3);
% disp(x)