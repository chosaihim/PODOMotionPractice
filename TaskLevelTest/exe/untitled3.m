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

des_lhx = file(:,203);
des_lhy = file(:,204);
des_lhz = file(:,205);

FK_lhx = file(:,206);
FK_lhy = file(:,207);
FK_lhz = file(:,208);

ref_lhx = file(:,209);
ref_lhy = file(:,210);
ref_lhz = file(:,211);

cur_lhx = file(:,212);
cur_lhy = file(:,213);
cur_lhz = file(:,214);

tar_lhx = file(:,218);
tar_lhy = file(:,219);
tar_lhz = file(:,220);

crt_lhx = file(:,215);
crt_lhy = file(:,216);
crt_lhz = file(:,217);

rsp = file(:,221);
rsy = file(:,222);
rsr = file(:,223);
reb = file(:,224);
rwy = file(:,225);

figure
hold on;
plot(des_rhy);
%plot(ref_rhx);
%plot(cur_rhx);
plot(tar_rhy);
plot(crt_rhy);
plot(FK_rhy);
%plot(q_rsp);
%plot(shared_q_rsp);
legend('des','RRRref','cur','tar','crt','FK','fkik.q','shared.q');

figure
hold on;
plot(des_lhy);
%plot(ref_lhx);
%plot(cur_lhx);
plot(tar_lhy);
plot(crt_lhy);
plot(FK_lhy);
%plot(q_rsp);
%plot(shared_q_rsp);
legend('des','LLLref','cur','tar','crt','FK','fkik.q','shared.q');

