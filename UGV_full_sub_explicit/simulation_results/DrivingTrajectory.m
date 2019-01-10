clc; clear all; close all;

h = 10;
file_name = sprintf('EX_sub_%dms_Run_C.csv',h);
data = dlmread(file_name,',',1,0);

path = dlmread('../car_sim_path_filtered.csv',',',1,0);

figure
set(gcf,'color',[1,1,1])
plot(path(:,1),path(:,2),'r','LineWidth',3)
hold on
plot(data(:,2),data(:,3),'b--','LineWidth',2.5)
grid on
title('Path & Trajectory')
xlabel('X [m]')
ylabel('Y [m]')
set(gca,'fontsize',14)