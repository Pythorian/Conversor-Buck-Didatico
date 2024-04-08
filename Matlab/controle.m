clc
clear all
close all
format long eng

%% Parâmetros %%
Vin = 30;
L = 2.8E-3;
C = 22E-6;
Ro = 22;
D = 0.5;
fs = 10E3;
fclk = 16e6; %64e6;
Ts = 1/fs;
Tclk = 1/fclk;
M = fclk/fs/2;

s = tf('s');
Gild = Vin/L * ((s+1/(C*Ro))/(s^2 + s/(C*Ro) + 1/(C*L)));
GildM = Gild * M;

Gvi = (1-D)*(1/(C*s + (1/Ro)));

% Simulação

time_step = Tclk/100;
total_time = 50*Ts;

IL = D*Vin/Ro;
Vo = D*Vin;

% Discretização

 Gild_z = c2d(GildM, Ts, 'zoh');
 
 Gatrasoi = tf([0 1],[1 0],Ts);
 Gild_z2 = Gild_z * Gatrasoi;
 
 Gild_w = d2c(Gild_z2, 'tustin');
 
 % Projeto do Controle
 
fc = fs/10; %frequencia de corte, maximo fs/8
wc = 2*pi*fc;

%Projeto do PI
wp_pi = 0;
wz_pi = wc/50;

Cpi = tf([1 wz_pi],[1 wp_pi]);

%Projeto do PD
theta = 45*pi/180;
wz_pd = wc*sqrt((1-sin(theta))/(1+sin(theta)));
wp_pd = wc*sqrt((1+sin(theta))/(1-sin(theta)));

Cpd = tf([1 wz_pd],[1 wp_pd]);

Cpid = Cpi * Cpd;
% K = 0.00094093;
% Cpid2 = K * Cpid;
% 
% %Discretizar o pid do plano w
% Gpid_z = c2d(Cpid2, Ts, 'tustin');
% 
% [num, den] = tfdata(Gpid_z, 'v');
% 
% a1 = num(1)
% a2 = num(2)
% a3 = num(3)
% b1 = den(2)
% b2 = den(3)

K = 0.00043996;
Cpi2 = K * Cpi;

%Discretizar o pid do plano w
Gpi_z = c2d(Cpi2, Ts, 'tustin');

[num, den] = tfdata(Gpi_z, 'v');

a1 = num(1)
a2 = num(2)
b1 = den(1)
b2 = den(2)
