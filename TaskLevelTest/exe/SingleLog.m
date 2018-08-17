clear

file = load('data.txt');

zmpx = file(:,1);
zmpy = file(:,4);

RFx = file(:,8);
RFy = file(:,9);
RFz = file(:,10);

LFx = file(:,11);
LFy = file(:,12);
LFz = file(:,13);

LIPMx = file(:,17);
LIPMy = file(:,18);

COMx = file(:,19);
COMy = file(:,20);

desRFz = file(:,43);
desLFz = file(:,44);

globalRFz = file(:,45);
globalLFz = file(:,46);

CONTX = file(:,85);
CONTY = file(:,86);

targetLFx = file(:,89);
targetLFy = file(:,90);

targetRFx = file(:,92);
targetRFy = file(:,93);

desRFx = file(:,99);
desRFy = file(:,100);
desLFx = file(:,101);
desLFy = file(:,102);

LIPMxn = file(:,123);
LIPMyn = file(:,128);

DSPx = file(:,124);
DSPy = file(:,131);

Ref_RHP = file(:,134);
Ref_LHP = file(:,135);
Ref_RAP = file(:,136);
Ref_LAP = file(:,137);

Pos_RHP = file(:,138);
Pos_LHP = file(:,139);
Pos_RAP = file(:,140);
Pos_LAP = file(:,141);

globalRFx = file(:,143);
globalRFy = file(:,144);

pCenterx = file(:,145);
pCentery = file(:,146);
pCenterz = file(:,147);

temp1x = file(:,152);
temp1y = file(:,153);
temp1z = file(:,154);

RAP = file(:,155);
RAR = file(:,156);
LAP = file(:,157);
LAR = file(:,158);

wRAP = file(:,159);
wRAR = file(:,160);
wLAP = file(:,161);
wLAR = file(:,162);


Rxori = file(:,163);
Ryori = file(:,164);
Rdeflec = file(:,165);
Rrolll = file(:,166);
Rpitttt= file(:,167);

Lxori = file(:,168);
Lyori = file(:,169);
Ldeflec = file(:,170);
Lrolll = file(:,171);
Lpitttt= file(:,172);

desRFxhat = file(:,173);
desRFyhat = file(:,174);
desRFzhat = file(:,175);
desLFxhat = file(:,176);
desLFyhat = file(:,177);
desLFzhat = file(:,178);



figure
hold on;
plot(-COMy,COMx);
plot(-LIPMy,LIPMx);

% figure
% hold on;
% plot(desRFxhat);
% plot(desRFyhat);
% plot(desRFzhat);
% plot(globalRFz);
% legend('Rxhat','yhat','zhat','glo');
% 
% 
% figure
% hold on;
% plot(desLFxhat);
% plot(desLFyhat);
% plot(desLFzhat);
% plot(globalLFz);
% legend('Lxhat','yhat','zhat','glo');
% figure
% hold on;
% plot(Rxori);
% plot(Ryori);
% plot(Rdeflec);
% plot(Rrolll);
% plot(Rpitttt);
% legend('RRRxori','yori','def','rol','pit');
% 
% figure
% hold on;
% plot(Lxori);
% plot(Lyori);
% plot(Ldeflec);
% plot(Lrolll);
% plot(Lpitttt);
% legend('LLLxori','yori','def','rol','pit');

% figure
% hold on;
% % plot(Ref_RAP);
% % plot(Pos_RAP);
% % plot(RAP);
% plot(wRAP*180.0/3.141592);
% legend('ref','pos','fw','wbik');
% 
% % 
% figure
% hold on;
% plot(desRFx);
% plot(desRFy);
% plot(desRFz);
% legend('x','y','z');

% figure
% hold on;
% plot(globalRFx);
% plot(globalRFy);
% legend('RFXXXX','RFYYYY');
% figure
% %plot(-LIPMy,LIPMx);
% hold on;
%plot(-COMy,COMx);
%plot(-LIPMyn,LIPMxn);

%legend('LIPM','LIPMn');

% plot(Ref_RHP);
% plot(Ref_RAP);
% plot(file(:,125));
% legend('Ref_RHP','Ref_RAP','flag');

% 
% figure
% hold on;
% plot(LIPMx);
% plot(LIPMy);
% plot(LIPMxn);
% plot(LIPMyn);
% %plot(DSPx);
% %plot(DSPy);
% legend('LIPMx','LIPMy','LIPMxn','LIPMyn');
% % 
% figure
% hold on;
% % plot(COMx);
% plot(desLFx);
% plot(desLFy);
% % plot(desLFx);
% plot(desLFz);
% %plot(globalRFx);
% legend('desLFx','desLFy','desLFz');
% % legend('com','rf','lf','rfz','global');
% % 
% figure
% hold on;
% plot(RFx);
% plot(RFy);
% plot(RFz);
% legend('RFx','RFy','RFz');
% % 
% % 
% figure
% hold on;
% plot(LFx);
% plot(LFy);
% plot(LFz);
% legend('LFx','LFy','LFz');