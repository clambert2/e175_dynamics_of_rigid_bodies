% Test for calcs_7_13.m
syms ln y_dist theta_dot

r = [0; ln+y_dist; 0]
w = [0; 0; theta_dot]

V = cross(w, r)