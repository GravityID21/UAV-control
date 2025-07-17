clc;clear all;
%%
Kix = 0;
Kpx = 2;
Kdx = 0;
Kfx = 0;
%%
Kiy = 0;
Kpy = 2;
Kdy = 0;
Kfy = 0;
%%
Kiz = 0.5;
Kpz = 2;
Kdz = 0;
Kfz = 0;
%%
Kipsi = 1;
Kppsi = 6;
Kdpsi = 0.35;
Kfpsi = 0;
%%
Kixd = 1;
Kpxd = 25;
Kdxd = 0;
Kfxd = 0;
%%
Kiyd = 1;
Kpyd = 25;
Kdyd = 0;
Kfyd = 0;
%%
Kizd = 15;
Kpzd = 25;
Kdzd = 0;
Kfzd = 0;
%%
Kipsid = 16.7;
Kppsid = 120;
Kdpsid = 0;
Kfpsid = 0;
%%
Kiphi = 3;
Kpphi = 6;
Kdphi = 0;
Kfphi = 0;
%%
Kit = 3;
Kpt = 6;
Kdt = 0;
Kft = 0;
%%
Kiphid = 500;
Kpphid = 250;
Kdphid = 2.5;
Kfphid = 0;
%%
Kitd = 500;
Kptd = 250;
Kdtd = 2.5;
Kftd = 0;
%%
noiseonxyz = 0; %28; % 5% mean error added to 1m radius helix
noiseonppt = 0; %10; % 5% mean error added to maximum of 20 degrees roll and pitch angle
