% Create the rigid frame to frame simple rotation matrices
syms theta phi p p_dot theta_dot phi_dot
a_R_n = [cos(theta) sin(theta) 0;
         -sin(theta) cos(theta) 0;
         0 0 1]

b_R_a = [cos(phi) 0 -sin(phi);
         0 1 0;
         sin(phi) 0 cos(phi)]

% Multiply the two ratation matrices together to get b_R_n
b_R_n = b_R_a*a_R_n

% Create P vector in B
P_B = [0; 0; p]

% Put P in N
P_N = b_R_n.'*P_B

P_dot_N = [p_dot*cos(theta)*sin(phi) + p*cos(theta)*cos(phi)*phi_dot - p*sin(theta)*sin(phi)*theta_dot;
           p_dot*sin(theta)*sin(phi) + p*sin(theta)*cos(phi)*phi_dot - p*cos(theta)*sin(phi)*theta_dot;
           cos(theta)*p_dot - sin(theta)*p*theta_dot]

P_dot_B = b_R_n*P_dot_N

