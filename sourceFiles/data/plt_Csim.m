set(0,'defaultAxesFontSize',16);
set(0,'defaultAxesFontName','Times New Roman');
set(0,'defaultTextFontSize',14);
set(0,'defaultTextFontName','Times New Roman');
set(0,'defaultaxeslinewidth',2);

%%
close all; clear; clc;
csim2mat(pwd)
load('Csim.mat')


figure;
plot(time,pos_ref,time,yd,time,pos_fb,'linewidth',2); grid on;
xlabel('Time [s] (total)'); ylabel('Position [m]');
legend('pos\_ref','yd','pos\_fb','Location','best');

figure;
plot(time,iq0_ref_fb,time,iq0_ref_ff,'linewidth',2); grid on;
xlabel('Time [s] (total)'); ylabel('Current [A]');
legend('iq0\_ref\_fb','iq0\_ref\_ff','Location','best');

%smpPlt = 2501:4001; % for STEPTIME 1.0
smpPlt = 5001:7001; % for STEPTIME 2.0
figure;
plot(time_ff(smpPlt),iq0_ref_fb(smpPlt),time_ff(smpPlt),iq0_ref_ff(smpPlt),'linewidth',2); grid on;
xlabel('Time [s] (15th poly step)'); ylabel('Current [A]');
legend('iq0\_ref\_fb','iq0\_ref\_ff','Location','best');

figure;
plot(time_ff(smpPlt),pos_ref(smpPlt),time_ff(smpPlt),pos_fb(smpPlt),'linewidth',2); grid on;
xlabel('Time [s] (15th poly step)'); ylabel('Position [m]');
legend('pos\_ref','pos\_fb','Location','best');

