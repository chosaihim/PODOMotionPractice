file = load('data.txt');

desCOMx = file(:,19);
desCOMy = file(:,20);
lipmCOMx = file(:,17);
lipmCOMy = file(:,18);

ZMPx = file(:,1);
ZMPy = file(:,4);

RFx = file(:,8);
RFy = file(:,9);
LFx = file(:,11);
LFy = file(:,12);

% figure
% hold on;
% plot(ZMPy);
% plot(desCOMy);
% plot(lipmCOMy);
% plot(RFy);
% plot(LFy);
% legend('ZMP','desCOM','lipmCOM','RF','LF');

figure
hold on;
plot(ZMPx,ZMPy);
plot(desCOMx, desCOMy);
plot(lipmCOMx, lipmCOMy);
plot(RFx, RFy);
plot(LFx, LFy);
legend('ZMP','desCOM','lipmCOM','RF','LF');