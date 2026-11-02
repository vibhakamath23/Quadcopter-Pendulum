%% LQR Controller Design
clear; clc; close all
load('linear_system.mat')
A = sys.A;  B = sys.B;  C = sys.C;  D = sys.D;
n = size(A,1);  m = size(B,2);