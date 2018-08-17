file = load('WBWALK.txt');

des_rhx = file(:,185);
des_rhy = file(:,186);
des_rhz = file(:,187);

FK_rhx = file(:,188);
FK_rhy = file(:,189);
FK_rhz = file(:,190);

ref_rhx = file(:,191);
ref_rhy = file(:,192);
ref_rhz = file(:,193);

cur_rhx = file(:,194);
cur_rhy = file(:,195);
cur_rhz = file(:,196);

tar_rhx = file(:,200);
tar_rhy = file(:,201);
tar_rhz = file(:,202);

crt_rhx = file(:,197);
crt_rhy = file(:,198);
crt_rhz = file(:,199);

Fz = file(:,235);
lpf_Fz = file(:,236);
posx = file(:,237);
measuredz = file(:,238);


rsp = file(:,221);
rsy = file(:,222);
rsr = file(:,223);
reb = file(:,224);
rwy = file(:,225);
sharedrsr = file(:,230);
Srsp = file(:,241);
Srsr = file(:,240);
Rrsr = file(:,242);

fkREB = file(:,244);
desREB = file(:,245);

figure
hold on;
plot(des_rhx);
plot(posx);
plot(crt_rhx);
legend('des','posx','crt');

figure
hold on;
plot(fkREB);
plot(desREB);
legend('fkreb','desreb');

figure
hold on;
% plot(des_rhx);
% plot(des_rhy);
% plot(des_rhz);
plot(rsr);
% plot(rsp);
plot(Srsr);
% plot(Srsp);
plot(Rrsr);
% legend('x','y','z','rsr','rsp','Srsr','Srsp','Rrsr');
legend('rsr','Srsr','Rrsr');
