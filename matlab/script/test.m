clear all
clc

s = tf('s');

T = -502.7 / (s^3 + 313.2*s^2 - 813.4*s - 255800);

pole(T)

