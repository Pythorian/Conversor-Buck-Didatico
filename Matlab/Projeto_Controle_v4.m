%-------------------------------------------------------------------------%
%               UFSM - Universidade Federal de Santa Maria                %
%              Curso de Engenharia de Controle e Automação                %
%                    Trabalho de Conclusão de Curso                       %
%                                                                         %
%   Programadores:                                                        %
%       Acad. Milton Brenner Machado Matoso                               %
%       Prof. Rafael Concatto Beltrame                                    %
%                                                                         %
%   Versão: 3.0                                             02/02/2024    %
%=========================================================================%
%                          Descrição do Programa                          %
%=========================================================================%
%	Projeto dos controladores das malhas de corrente (interna) e          %
%   tensão (externa) de um conversor CC-CC Buck.                          %
%                                                                         %
%   v3.0 - Projeto do compensador de tensão. Melhor fiferenciação das     %
%          variáveis das malhas de tensão e corrente.                     %  
%   v2.0 - Projeto realizado exclusivamente no plano s (contínuo).        %
%   v1.0 - Versão inicial.                                                %
%-------------------------------------------------------------------------%
close all                               % Fecha todos os gráficos
clear all                               % Exclui todas as variáveis
clc                                     % Limpa a tela
format long eng                         % Formato para exibição numérica

%-------------------------------------------------------------------------%
% Especificações                                                          %
%-------------------------------------------------------------------------%
Vin  = 30;                              % Tensão de entrada (V)
L    = 2.8e-3;                          % Indutância de filtro (H)
C    = 22e-6;                           % Capacitância de filtro (F)
Ro   = 22;                              % Resistência de carga (Ohm)
D    = 0.5;                             % Razão-cíclica
fs   = 10e3;                            % Frequência de chaveamento (Hz)
Ts   = 1/fs;                            % Período de chaveamento (s)
fclk = 64e6; %16e6; %64e6;              % Frequência de clock (Hz)
Tclk = 1/fclk;                          % Período de clock (s)

%% -----------------------------------------------------------------------%
% Modelo da Planta (corrente para razão-cíclica)                          %
%-------------------------------------------------------------------------%
s = tf('s');                            % Variável de Laplace

num  = Vin/L * [1 1/(C*Ro)];            % Função de transferência da planta
den  = [1 1/(C*Ro) 1/(C*L)];
GiLd = tf(num,den);

cont_max = (fclk/fs)/2;                 % Valor máximo da triangular
M        = 1/cont_max;                  % Ganho do modulador PWM
GiLdM    = M * GiLd;                    % Adição do ganho do modulador                                        % Planta 

%----------------------%
% Validação no PSIM    %
%----------------------%
time_step  = Tclk;                      % Passo de simulação (s)
total_time = 50*Ts;                     % Tempo total (s)

% Condições iniciais
Vo = D*Vin;                             % Tensão média no capacitor (V)
IL = D*Vin/Ro;                          % Corrente média no indutor (A)

%-------------------------------------------------------------------------%
% Modelo do Atraso de Implementação                                       %
%-------------------------------------------------------------------------%
% Referência: TCC de Jeferson Calai (pág. 50)

fai = fs;                               % Frequência de amostragem (Hz)
Tai = 1/fai;                            % Período de amostragem (s)

Gatrasoi = tf([-1 2/Tai],[1 2/Tai]);    % Função de transferência do atraso
                                        % (Aproximação de Pade)

GiLdMa = GiLdM * Gatrasoi;              % Adição do atraso de um sample
 
%-------------------------------------------------------------------------%
% Projeto da Malha de Controle da Corrente                                %
%-------------------------------------------------------------------------%
fci = fai/10;                           % Frequência de cruzamento desejada (Hz)
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
Cpii   = tf([1 wz_pi],[1 wp_pi]);       % Função de transferência

% OBS: Como a margem de fase ficou em 62.2º, não foi necessária a
%      inclusão de um compensador PD.

k = 1200;                             % Ganho proporcional (sisotool)
Cpii2 = k * Cpii;                       % Adição do ganho proporcinal

%----------------------%
% Discretização        %
%----------------------%
Gpii_z = c2d(Cpii2, Tai, 'tustin');     % Discretização

[num, den] = tfdata(Gpii_z, 'v');       % Extração dos coeficientes

ai1 = num(1)
ai2 = num(2)

%% -----------------------------------------------------------------------%
% Modelo da Planta (tensão para corrente)                                 %
%-------------------------------------------------------------------------%
num = [1/C];                               % Função de transferência da planta
den = [1 1/(C*Ro)];
Gvi = tf(num, den);

% %----------------------%
% % Validação no PSIM    %
% %----------------------%
% time_step  = Tclk;                      % Passo de simulação (s)
% total_time = 50*Ts;                     % Tempo total (s)
% 
% % Condições iniciais
% Vo = D*Vin;                             % Tensão média no capacitor (V)
% IL = D*Vin/Ro;                          % Corrente média no indutor (A)

%-------------------------------------------------------------------------%
% Modelo do Atraso de Implementação                                       %
%-------------------------------------------------------------------------%
% Referência: TCC de Jeferson Calai (pág. 50)

fav = fai;                              % Frequência de amostragem (Hz)
Tav = 1/fav;                            % Período de amostragem (s)

Gatrasov = tf([-1 2/Tav],[1 2/Tav]);    % Função de transferência do atraso
                                        % (Aproximação de Pade)

Gvia = Gvi * Gatrasov;                  % Adição do atraso de um sample
 
%-------------------------------------------------------------------------%
% Projeto da Malha de Controle da Corrente                                %
%-------------------------------------------------------------------------%
% OBS: A malha de tensão deve ser, pelo menos 10 vezes mais lenta que a
%      malha de corrente
fcv = fci/10;                           % Frequência de cruzamento desejada (Hz)
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
Cpiv   = tf([1 wz_pi],[1 wp_pi]);       % Função de transferência

% OBS: Como a margem de fase ficou em 156º, não foi necessária a
%      inclusão de um compensador PD.

k = 0.04747;                            % Ganho proporcional (sisotool)
Cpiv2 = k * Cpiv;                       % Adição do ganho proporcinal

%----------------------%
% Discretização        %
%----------------------%
Gpiv_z = c2d(Cpiv2, Tav, 'tustin');     % Discretização

[num, den] = tfdata(Gpiv_z, 'v');       % Extração dos coeficientes

av1 = num(1)
av2 = num(2)
