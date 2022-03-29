clc 
close all
clear all

% Longitud de los eslabones en [MILIMETROS] :
A = 50; 
B = 200;
C = 50;
D = 230;
SEG2 = 230;
L1 = D;
L2 = SEG2 - 50;
L3 = 30;
Lx = sqrt( L2^2 + L3^2 );
%Alpha = 10:0.5:109; %Intervalo del ángulo Alpha [en Grados]
%Phi = 30:-0.5:-150;

X_C = [ -96.22  -96.22  -96.22  -104.33  -111.47  -118.62  -118.62  -118.62  -118.62  -118.62  -118.62  -118.62  -111.47  -104.33  -96.22  -96.22  -96.22 ];
Y_C = [ -311.94 -302.78 -293.43 -293.43  -293.43  -293.43  -304.72  -315.34  -326.77  -339.43  -351.06  -361.06  -361.06  -361.06  -361.06 -351.06 -341.06];  
X_o = [ -78.7  -78.7  -78.7  -78.7 -72.5 -65.8 -59.1 -59.1  -59.1  -59.1  -59.1  -59.1 -59.1 -65.8 -72.5 -78.7 -78.7 -78.7  -78.7  ];
Y_o = [ -341.1 -347.2 -354.6 -361  -361  -361  -361  -354.6 -347.2 -341.1 -334.1 -328  -320  -320  -320  -320  -328  -334.1 -341.1 ];
X_v = [ -44.1 -42.9  -41.3 -39.5   -38.2   -36.7   -33.92 -31.5  -29.5  -27.8  -26.4 -24.8 -23.2 ];
Y_v = [ -320  -328   -334  -341.1  -347.2  -354.6  -361   -354.6 -347.2 -341.1 -334  -328  -320 ];
X_i = [ -8.37  -8.37  -8.37 -8.37 -8.37 -8.37  -8.37  -8.37  -8.37 ];
Y_i = [ -300.1 -307.6 -320  -328  -334  -341.1 -347.2 -354.6 -361  ];
X_d = [ 28.53  20.86  13.1   7.9  7.9  7.9    7.9    7.9    13.1 20.86 28.53 28.53  28.53  28.53  28.53 28.53 28.53  28.53  28.53 28.53 28.53 ];
Y_d = [ -320.9 -320.9 -320.9 -328 -334 -341.1 -347.2 -354.6 -361 -361  -361  -354.6 -347.2 -341.1 -334  -328  -320.9 -312.9 -306  -300  -293  ];
X__ = [ 42.1   50.5   59.4 ];
Y__ = [ -334.1 -334.1 -334.1];
X_1 = [ 75.68  75.68 75.68  75.68 75.68 75.68 75.68 75.68  75.68  75.68  75.68 ];
Y_1 = [ -293  -300   -306  -313   -320  -328  -334  -341.1 -347.2 -354.6 -361  ];
X_9 = [ 94.2   94.2   94.2  101.4 108.7 115.27 116.4  116.4  116.4 116.4  116.4 116.4 116.4 116.4 116.4 116.4 108.7 101.4 94.2 94.2 94.2 94.2  94.2 94.2 101.4 108.7 116.4  ];
Y_9 = [ -347.2 -354.6 -361  -361  -361  -361   -354.6 -347.2 -341.1 -334  -328   -320  -313  -306  -300 -293  -293  -293  -293 -300 -306 -313  -320 -328 -328  -328  -328   ];

X = [ X_C X_o X_v X_i X_d X__ X_1 X_9];
Y = [ Y_C Y_o Y_v Y_i Y_d Y__ Y_1 Y_9];

X = X - 30;
Y = Y + 14;
% Rango H:   270 <= H >= 393                                                                                 ==>
% X_inicio  218.6474
% Y_inicio  -172.2531
% Y = - sqrt (H.^2 - X.^2);
H = sqrt (X.^2 + Y.^2);
% X_Y = [ X ; Y ]

%% CINEMÁTICA INVERSA
Phi_1 = atan2d(Y,X);
cos_Phi_3 = (L1^2 + H.^2 - Lx^2)./(2*L1*H);
Phi_3 = atan2d(sqrt(1 - cos_Phi_3.^2), cos_Phi_3);
    q1 = Phi_1 + Phi_3;
    
cos_Theta_1 = (L1^2 + Lx^2 - H.^2)/(2*L1*Lx);
Theta_1 = atan2d(sqrt(1 - cos_Theta_1.^2), cos_Theta_1);
Theta_2 = atan2d(L3, L2);
    q2 = Theta_1 - Theta_2;
%q1_q2=[q1;q2]
    
% q1 = phi-alpha
%vect_A + vect_B + vect_C = vect_D
vect_D = D*cosd(q1) + i*D*sind(q1);
vect_C = C*cosd(q1+q2-180) + i*C*sind(q1+q2-180);

% Se forma un triangulo entre : A, B y D-C
% Ley de los cosenos   :   A^2 = B^2 + C^2 - 2BC cos(a)
% Por lo tanto :
R =                     abs( vect_D - vect_C );
angle_d_c = radtodeg( angle( vect_D - vect_C ) );
r = ((A^2 + B^2 - R.^2) ./ (2*A*B));
a = ((B^2 + R.^2 - A^2) ./ (2*B*R));
b = ((R.^2 + A^2 - B^2) ./ (2*R*A));
r = acosd(r);
a = acosd(a);
b = acosd(b);

vect_A = A*cosd(angle_d_c + b) + i*A*sind(angle_d_c + b);
vect_B = B*cosd(angle_d_c - a) + i*B*sind(angle_d_c - a);
%vect_zero = vect_A + vect_B + vect_C - vect_D

q1Motor = q1;
q2Motor = radtodeg( angle(vect_A) );
qMotor = [q1Motor; q2Motor];

Alpha = q2Motor - q1Motor;
Phi = q2Motor;
Phi_Alpha = [ Phi ; Alpha ]

% vect_SEG2 = SEG2*vect_C / C;
% TCP = vect_A + vect_B + (i*30*vect_C/50) + vect_SEG2;
%         TCPx = real(TCP);
%         TCPy = imag(TCP);
% TCP = [ TCPx ; TCPy ]

%% CINEMÁTICA DIRECTA
for n=1:size(Alpha,2)
    phi = Phi(n);
    alpha = Alpha(n);
    int_phi = phi <= 30 & phi > -150;
    int_alpha = alpha < 110 & alpha >= 10;
    if alpha <= (160 + phi) & alpha >= phi & int_phi & int_alpha
        vect_A_d = A*cosd(phi) + i*A*sind(phi);
        vect_D_d = D*cosd(phi-alpha) + i*D*sind(phi-alpha);

% Se forma un triangulo entre : B, C y D-A
% Ley de los cosenos   :   A^2 = B^2 + C^2 - 2BC cos(a)
% Por lo tanto :
        R =                     abs( vect_D_d - vect_A_d );
        angle_d_a = radtodeg( angle( vect_D_d - vect_A_d ) );
        r = acosd((B^2 + C^2 - R^2) / (2*B*C));
        b = acosd((C^2 + R^2 - B^2) / (2*C*R));
        c = acosd((R^2 + B^2 - C^2) / (2*R*B)); 

        vect_B_d = B*cosd(angle_d_a + c) + i*B*sind(angle_d_a + c);
        vect_C_d = -C*cosd(180 + angle_d_a - b) - i*C*sind(180 + angle_d_a - b);

        vect_SEG2 = SEG2*vect_C_d / C;
        TCP_d = vect_A_d + vect_B_d + (i*30*vect_C_d/50) + vect_SEG2;
        TCPx_d(n) = real(TCP_d);
        TCPy_d(n) = imag(TCP_d);
        
        %Mag = sqrt(real(TCP)^2 + imag(TCP)^2)
        %vect_zero_d = vect_A_d + vect_B_d + vect_C_d - vect_D_d
    end
end

%% GRÁFICA
scatter(TCPx_d, TCPy_d, 4,'filled');
title('Workspace')
grid on
ylim([-500 100])
xlim([-500 500])

%Mag = sqrt(TCPx_d.^2 + TCPy_.^2)
TCP_d = [ TCPx_d ; TCPy_d ]


%% JACOBIANA INVERSA
Vel_xy = 100;
X_ini = 218.6474; Y_ini = -172.2531;
Q1_ini = 30; Q2_ini = 0;
for h=1:size(Alpha,2)
    if(h == 1)
        Dir = (X(h) - X_ini) + i*(Y(h) - Y_ini);
    else
        Dir = (X(h) - X(h-1)) + i*(Y(h) - Y(h-1));
        %q1_vel(h) = (q1(h)-q1(h-1))/( abs(Dir) )
    end
    Dir_ang = radtodeg( angle(Dir) );
    Vel_X(h) = Vel_xy*cosd(Dir_ang);
    Vel_Y(h) = Vel_xy*sind(Dir_ang);
    
    C_q1 = cosd(q1(h));          S_q1 = sind(q1(h));
    C_q12 = cosd(q1(h)+(q2(h)-90)); S_q12 = sind(q1(h)+(q2(h)-90));    

    Jac_inv_1 = [ -L2*S_q12-L3*C_q12           L2*C_q12-L3*S_q12         ];
    Jac_inv_2 = [ L1*C_q1+L2*S_q12+L3*C_q12    L1*S_q1-L2*C_q12+L3*S_q12 ];
    % Determinante
    Determi = L1*L2*C_q12.*C_q1 + L1*L3*C_q12.*S_q1 - L1*L3*S_q12.*C_q1 + L1*L2*S_q12.*S_q1;
    Jac_inv = [ Jac_inv_1 ; Jac_inv_2 ]./Determi;
    q_vel = (180/pi)*Jac_inv*[ Vel_X ; Vel_Y ];
end


%% CONSOLA DE COMANDOS
q_vel = abs(q_vel)
fprintf('Valor de Phi \n');
fprintf('%2.0f, ', Phi); fprintf('\n');
fprintf('Valor de Alpha \n');
fprintf('%2.0f, ', Alpha); fprintf('\n');
fprintf('Valor de q2_Vel \n');
fprintf('%2.0f, ', q_vel(1,:)); fprintf('\n');
fprintf('Valor de q1_Vel \n');
fprintf('%2.0f, ', q_vel(2,:)); fprintf('\n');


Servo = ones(1,120);
SizXC=size(X_C,2);SizXo=size(X_o,2);SizXv=size(X_v,2);SizXi=size(X_i,2);SizXd=size(X_d,2);
SizX_=size(X__,2);SizX1=size(X_1,2);SizX9=size(X_9,2);
Servo(1) = 0;
Servo(SizXC+1) = 0;
Servo(SizXC+SizXo+1) = 0;
Servo(SizXC+SizXo+SizXv+1) = 0;
Servo(SizXC+SizXo+SizXv+3) = 0;
Servo(SizXC+SizXo+SizXv+SizXi+1) = 0;
Servo(SizXC+SizXo+SizXv+SizXi+SizXd+1) = 0;
Servo(SizXC+SizXo+SizXv+SizXi+SizXd+SizX_+1) = 0;
Servo(SizXC+SizXo+SizXv+SizXi+SizXd+SizX_+SizX1+1) = 0;
fprintf('Valor de boolServo \n');
fprintf('%1.0f, ', Servo); fprintf('\n');