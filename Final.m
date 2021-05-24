%% Model P. Paoletti and Iqbal, 
% modified by Melissa Molenaars 24-05-2021
clear all; close all; clc; on = 1; off = 0;
%% Variables
l = 11; %length of slackline (m)
m = 78; %mass (kg)
el = 1; %relative elongation (%) 
D_ep = 0.5*l*(1+(1/100)*el); %elongation (epsilon), according to ...
y_r = sqrt(D_ep*D_ep - 0.5*0.5*l*l); %length radius (m)
T = 20; %sample time (s)
%% State Space model - Paoletti or Iqbal
P = off; % If on (and I off), through the Paoletti
I = on; % If on (and P off), through Iqbal => done in paper
if (P == 1)&(I == 0)
    disp('Through Paoletti')
A = [0, 1, 0, 0; 
    -1/y_r, 0, m/y_r, 0;
    0, 0, 0, 1;
    1/y_r, 0, 1+m-m/y_r, 0];
B = [0; m*(y_r-1)*y_r*y_r; 0; 1+m+m*(1-2*y_r)*y_r*y_r];
C = [1, 0, 0, 1];
D = 0;
SYS = ss(A, B, C, D);
elseif (P == 0)&(I == 1)
    disp('Through Iqbal')
a11 =  -(293*m + 298)/(y_r*(25*m + 298));
a12 = 174*m/(5*y_r*(25*m + 298));
a21 = (3750*y_r + 2930*m - 225*y_r*m + 2980)/(10*y_r*(25*m + 298));
a22 = (4350*y_r - 348*m + 435*y_r*m)/(10*y_r*(25*m + 298));
b1 =  -(60*m)/(y_r*(25*m + 298));
b2 = -(7500*y_r - 600*m + 750*y_r*m)/(10*y_r*(25*m + 298));

A = [0, 0, 1, 0; 
    0, 0, 0, 1;
    a11, a12, 0, 0;
    a21, a22, 0, 0];
B = [0; 0; b1; b2];
C = [1, 0, 0, 1];
D = 0;
SYS = ss(A, B, C, D);
else
    disp('No state space')
    return;
end

%% Checking for...
% ... stability
if eig(A) < 0
    disp('The system is stable');
else 
    disp('The system is unstable')
end
% ... controllability
if rank(ctrb(A,B)) == length(A)
    disp('The system is controllable')
else 
    disp('!! uncontrollable system')
end 
% ... observability
if rank(obsv(A,C)) == length(A)
    disp('The system is observable')
else 
    disp('!! unobservable system')
end 
%% LQR Controller
Co = ctrb(A,B);
Q = Co/Co;  %weighting matrix
R = 1; %weighting matrix
K = lqr(A, B, Q, R); %gain
SYS_LQR = ss(A-B*K, B*K(1), C, D); %new state space

% Is the system stable?
if eig(A-B*K) < 0
    disp('The system with LQR control is stable');
else 
    disp('The system with LQR control is unstable')
end
%% PP Controller 
% Iterative optimal gain for the pole placement (PP) controller
figure(); title('test PP controller'); 
Pot_gains = [-1, -2, -3, -4, -5, -7, -10, -20, -100, -1000]; %potential gains
for i = Pot_gains
    SYS_i = ss(A-B*i, B*i, C, D);
    step(SYS_i, T); hold on
end
hold off;

F = -1; %chosen gain
SYS_PP = ss(A-B*F, B*F, C, D); %PP state space

% Is the system stable?
if eig(A-B*F) < 0
    disp('The system with PP control is stable');
else 
    disp('The system with PP control is unstable')
end
%% Performance 
%plots
figure(); step(SYS, T);
title('Step response without control'); legend('SYS')
ylabel('Amplitude (-)'); xlabel('Time');
figure(); step(SYS_LQR, T);
title('Step response with LQR control'); legend('SYS')
ylabel('Amplitude (-)'); xlabel('Time');
figure(); step(SYS_LQR, 'r', SYS_PP, '--b', T);
title('Step response with control'); legend('LQR', 'PP')
ylabel('Amplitude (-)'); xlabel('Time');

%paramters
Specs_LQR = stepinfo(SYS_LQR); %Overshoot, rise time, settling time, etc.
Specs_PP = stepinfo(SYS_PP); %Overshoot, rise time, settling time, etc.
