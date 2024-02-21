clc;
clear;
close all;
load("VehiclesPosition.mat");

figure(1);
hold on;
grid on;
box on;
xlim([115.9135,115.919]);
ylim([39.3675,39.3720]);
xlabel("经度(deg)");
ylabel("纬度(deg)");
% axis equal;
% vehicle 1 62759(line 1531) Start Circle 1931
plot(Vehicle1POS(1931:2731,4)+0.001,Vehicle1POS(1931:2731,3),LineWidth=0.8,Color="blue",Marker="o",MarkerIndices=1:50:(2731-1931));
% vehicle 2 40161(line 946) Start Circle 1446
plot(Vehicle2POS(1446:2246,4)-0.0005,Vehicle2POS(1446:2246,3),LineWidth=0.8,Color="blue",Marker="square",MarkerIndices=1:50:(2246-1446));
% vehicle 3 39462(line 932) Start Circle 1232
plot(Vehicle3POS(1232:1932,4),Vehicle3POS(1232:1932,3)+0.001,LineWidth=0.8,Color="red",Marker="^",MarkerIndices=1:50:(1932-1232));
% vehicle 4 40229(line 952) Start Circle 
plot(Vehicle4POS(1152:2052,4),Vehicle4POS(1152:2052,3),LineWidth=0.8,Color="red",Marker="+",MarkerIndices=1:50:(2052-1152));
legend("防御无人机1","防御无人机2","攻击无人机1","攻击无人机2");

figure(2);
hold on;
grid on;
box on;
xlim([115.9125,115.9175]);
ylim([39.3675,39.3720]);
xlabel("经度(deg)");
ylabel("纬度(deg)");
% vehicle 1 Start Mission 2732
plot(Vehicle1POS(2732:3332,4)+0.0001,Vehicle1POS(2732:3332,3),LineWidth=0.8,Color="blue",Marker="o",MarkerIndices=1:50:(3332-2732));
% vehicle 2 Start Mission 2247
plot(Vehicle2POS(4047:4547,4)-0.002,Vehicle2POS(4047:4547,3),LineWidth=0.8,Color="blue",Marker="square",MarkerIndices=1:50:(4547-4047));
% vehicle 3 Start Mission 1933
plot(Vehicle3POS(3033:3533,4),Vehicle3POS(3033:3533,3)+0.0005,LineWidth=0.8,Color="red",Marker="^",MarkerIndices=1:50:(3533-3033));
% vehicle 4 Start Mission 2053
plot(Vehicle4POS(4853:5353,4)-0.001,Vehicle4POS(4853:5353,3),LineWidth=0.8,Color="red",Marker="+",MarkerIndices=1:50:(5353-4853));
legend("防御无人机1","防御无人机2","攻击无人机1","攻击无人机2");

figure(3);
hold on;
grid on;
box on;
% xlim([115.9125,115.9175]);
% ylim([39.3675,39.3720]);
xlabel("经度(deg)");
ylabel("纬度(deg)");
% vehicle 1 Start Mission 2732
plot(Vehicle1POS(3333:3833,4),Vehicle1POS(3333:3833,3),LineWidth=0.8,Color="blue",Marker="o",MarkerIndices=1:50:(3833-3333));
% vehicle 2 Start Mission 2247
plot(Vehicle2POS(4548:5048,4),Vehicle2POS(4548:5048,3),LineWidth=0.8,Color="blue",Marker="square",MarkerIndices=1:50:(5048-4548));
% vehicle 3 Start Mission 1933
plot(Vehicle3POS(3534:4034,4),Vehicle3POS(3534:4034,3),LineWidth=0.8,Color="red",Marker="^",MarkerIndices=1:50:(4034-3534));
% vehicle 4 Start Mission 2053
plot(Vehicle4POS(5354:5854,4),Vehicle4POS(5354:5854,3),LineWidth=0.8,Color="red",Marker="+",MarkerIndices=1:50:(5854-5354));
legend("防御无人机1","防御无人机2","攻击无人机1","攻击无人机2");

figure(4);
hold on;
grid on;
box on;
% xlim([115.9125,115.9175]);
% ylim([39.3675,39.3720]);
xlabel("经度(deg)");
ylabel("纬度(deg)");
% vehicle 1 Start Mission 2732
plot(Vehicle1POS(3834:4834,4),Vehicle1POS(3834:4834,3),LineWidth=0.8,Color="blue",Marker="o",MarkerIndices=1:50:(4834-3834));
% vehicle 2 Start Mission 2247
plot(Vehicle2POS(5049:6049,4),Vehicle2POS(5049:6049,3),LineWidth=0.8,Color="blue",Marker="square",MarkerIndices=1:50:(6049-5049));
% vehicle 3 Start Mission 1933
plot(Vehicle3POS(4035:5035,4),Vehicle3POS(4035:5035,3),LineWidth=0.8,Color="red",Marker="^",MarkerIndices=1:50:(5035-4035));
% vehicle 4 Start Mission 2053
plot(Vehicle4POS(5855:6855,4),Vehicle4POS(5855:6855,3),LineWidth=0.8,Color="red",Marker="+",MarkerIndices=1:50:(6855-5855));
legend("防御无人机1","防御无人机2","攻击无人机1","攻击无人机2");