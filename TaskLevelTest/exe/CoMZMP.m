clc
clear all
close all

data = load('data.txt');

figure
plot(data(:,18))
hold on
plot(data(:,64)/1000)
plot(data(:,52))

figure
plot(data(:,18))
hold on
plot(data(:,97))
