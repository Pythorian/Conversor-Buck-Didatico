%-------------------------------------------------------------------------%
%               UFSM - Universidade Federal de Santa Maria                %
%              Curso de Engenharia de Controle e Automa��o                %
%                    Trabalho de Conclus�o de Curso                       %
%                                                                         %
%   Programadores:                                                        %
%       Acad. Milton Brenner Machado Matoso                               %
%       Prof. Rafael Concatto Beltrame                                    %
%                                                                         %
%   Vers�o: 3.0                                             02/02/2024    %
%=========================================================================%
%                          Descri��o do Programa                          %
%=========================================================================%
%	Projeto dos controladores das malhas de corrente (interna) e          %
%   tens�o (externa) de um conversor CC-CC Buck.                          %
%                                                                         %
%   v3.0 - Projeto do compensador de tens�o. Melhor fiferencia��o das     %
%          vari�veis das malhas de tens�o e corrente.                     %  
%   v2.0 - Projeto realizado exclusivamente no plano s (cont�nuo).        %
%   v1.0 - Vers�o inicial.                                                %
%-------------------------------------------------------------------------%
close all                               % Fecha todos os gr�ficos
clear all                               % Exclui todas as vari�veis
clc                                     % Limpa a tela
format long eng                         % Formato para exibi��o num�rica

%-------------------------------------------------------------------------%
% Especifica��es                                                          %
%-------------------------------------------------------------------------%
Vin  = 30;                              % Tens�o de entrada (V)
L    = 2.8e-3;                          % Indut�ncia de filtro (H)
C    = 22e-6;                           % Capacit�ncia de filtro (F)
Ro   = 22;                              % Resist�ncia de carga (Ohm)
D    = 0.5;                             % Raz�o-c�clica
fs   = 10e3;                            % Frequ�ncia de chaveamento (Hz)
Ts   = 1/fs;                            % Per�odo de chaveamento (s)
fclk = 64e6; %16e6; %64e6;              % Frequ�ncia de clock (Hz)
Tclk = 1/fclk;                          % Per�odo de clock (s)

%% -----------------------------------------------------------------------%
% Modelo da Planta (corrente para raz�o-c�clica)                          %
%-------------------------------------------------------------------------%
s = tf('s');                            % Vari�vel de Laplace

num  = Vin/L * [1 1/(C*Ro)];            % Fun��o de transfer�ncia da planta
den  = [1 1/(C*Ro) 1/(C*L)];
GiLd = tf(num,den);

cont_max = (fclk/fs)/2;                 % Valor m�ximo da triangular
M        = 1/cont_max;                  % Ganho do modulador PWM
GiLdM    = M * GiLd;                    % Adi��o do ganho do modulador                                        % Planta 

%----------------------%
% Valida��o no PSIM    %
%----------------------%
time_step  = Tclk;                      % Passo de simula��o (s)
total_time = 50*Ts;                     % Tempo total (s)

% Condi��es iniciais
Vo = D*Vin;                             % Tens�o m�dia no capacitor (V)
IL = D*Vin/Ro;                          % Corrente m�dia no indutor (A)

%-------------------------------------------------------------------------%
% Modelo do Atraso de Implementa��o                                       %
%-------------------------------------------------------------------------%
% Refer�ncia: TCC de Jeferson Calai (p�g. 50)

fai = fs;                               % Frequ�ncia de amostragem (Hz)
Tai = 1/fai;                            % Per�odo de amostragem (s)

Gatrasoi = tf([-1 2/Tai],[1 2/Tai]);    % Fun��o de transfer�ncia do atraso
                                        % (Aproxima��o de Pade)

GiLdMa = GiLdM * Gatrasoi;              % Adi��o do atraso de um sample
 
%-------------------------------------------------------------------------%
% Projeto da Malha de Controle da Corrente                                %
%-------------------------------------------------------------------------%
fci = fai/10;                           % Frequ�ncia de cruzamento desejada (Hz)
wci = 2*pi*fci;                         % (rad/s)

%----------------------%
% SISO Tool            %
%----------------------%
% sisotool

%----------------------%
% Controlador PI       %
%----------------------%
wp_pi = 0;                              % Polo (rad/s)
wz_pi = wci/20;                         % Zero (rad/s)
Cpii   = tf([1 wz_pi],[1 wp_pi]);       % Fun��o de transfer�ncia

% OBS: Como a margem de fase ficou em 62.2�, n�o foi necess�ria a
%      inclus�o de um compensador PD.

k = 1200;                             % Ganho proporcional (sisotool)
Cpii2 = k * Cpii;                       % Adi��o do ganho proporcinal

%----------------------%
% Discretiza��o        %
%----------------------%
Gpii_z = c2d(Cpii2, Tai, 'tustin');     % Discretiza��o

[num, den] = tfdata(Gpii_z, 'v');       % Extra��o dos coeficientes

ai1 = num(1)
ai2 = num(2)

%% -----------------------------------------------------------------------%
% Modelo da Planta (tens�o para corrente)                                 %
%-------------------------------------------------------------------------%
num = [1/C];                               % Fun��o de transfer�ncia da planta
den = [1 1/(C*Ro)];
Gvi = tf(num, den);

% %----------------------%
% % Valida��o no PSIM    %
% %----------------------%
% time_step  = Tclk;                      % Passo de simula��o (s)
% total_time = 50*Ts;                     % Tempo total (s)
% 
% % Condi��es iniciais
% Vo = D*Vin;                             % Tens�o m�dia no capacitor (V)
% IL = D*Vin/Ro;                          % Corrente m�dia no indutor (A)

%-------------------------------------------------------------------------%
% Modelo do Atraso de Implementa��o                                       %
%-------------------------------------------------------------------------%
% Refer�ncia: TCC de Jeferson Calai (p�g. 50)

fav = fai;                              % Frequ�ncia de amostragem (Hz)
Tav = 1/fav;                            % Per�odo de amostragem (s)

Gatrasov = tf([-1 2/Tav],[1 2/Tav]);    % Fun��o de transfer�ncia do atraso
                                        % (Aproxima��o de Pade)

Gvia = Gvi * Gatrasov;                  % Adi��o do atraso de um sample
 
%-------------------------------------------------------------------------%
% Projeto da Malha de Controle da Corrente                                %
%-------------------------------------------------------------------------%
% OBS: A malha de tens�o deve ser, pelo menos 10 vezes mais lenta que a
%      malha de corrente
fcv = fci/10;                           % Frequ�ncia de cruzamento desejada (Hz)
wcv = 2*pi*fcv;                         % (rad/s)

%----------------------%
% SISO Tool            %
%----------------------%
% sisotool

%----------------------%
% Controlador PI       %
%----------------------%
wp_pi = 0;                              % Polo (rad/s)
wz_pi = wcv/20;                         % Zero (rad/s)
Cpiv   = tf([1 wz_pi],[1 wp_pi]);       % Fun��o de transfer�ncia

% OBS: Como a margem de fase ficou em 156�, n�o foi necess�ria a
%      inclus�o de um compensador PD.

k = 0.04747;                            % Ganho proporcional (sisotool)
Cpiv2 = k * Cpiv;                       % Adi��o do ganho proporcinal

%----------------------%
% Discretiza��o        %
%----------------------%
Gpiv_z = c2d(Cpiv2, Tav, 'tustin');     % Discretiza��o

[num, den] = tfdata(Gpiv_z, 'v');       % Extra��o dos coeficientes

av1 = num(1)
av2 = num(2)
