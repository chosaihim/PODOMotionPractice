clear

file = load('data.txt');

comx = file(:,103);
comy = file(:,104);
lipmx = file(:,17);
lipmy = file(:,18);
fkcomx = file(:,183);
fkcomy = file(:,184);
zmpx = file(:,1);
zmpy = file(:,4);
rfx = file(:,8);
rfy = file(:,9);
dspx = file(:,124);
lipmnx = file(:,123);
lipmny = file(:,128);
localx = file(:,182);
delxzmp = file(:,33);
izmpx = file(:,35);


Ffkcomx = file(:,185);
Ffkcomy = file(:,186);
Sfkcomx = file(:,187);
Sfkcomy = file(:,188);
Ffkrhx = file(:,189);
Ffkrhy = file(:,190);
Ffklhx = file(:,192);
Ffklhy = file(:,193);

Ifkcomx = file(:,195);
Ifkcomy = file(:,196);
I2fkcomx = file(:,197);
I2fkcomy = file(:,198);
Ifkrhx = file(:,199);
Ifkrhy = file(:,200);
Ifklhx = file(:,202);
Ifklhy = file(:,203);
I2fkrhx = file(:,205);
I2fkrhy = file(:,206);
I2fklhx = file(:,208);
I2fklhy = file(:,209);
fk0comx = file(:,211);
fk0comy = file(:,212);
fkcomx = file(:,214);
fkcomy = file(:,215);

rhx = file(:,217);
rhy = file(:,218);
rhz = file(:,219);

fkrhx = file(:,220);
fkrhy = file(:,221);
fkrhz = file(:,222);

figure
plot(comx,comy);
hold on;
plot(zmpx,zmpy);
plot(lipmx,lipmy);
plot(fkcomx,fkcomy);
legend('descom','zmp','lipm','fkcom');

figure
plot(fkcomx);


figure
plot(lipmnx);
hold on;
plot(dspx);
legend('lipmn','dsp');

figure
plot(localx);
hold on;
plot(delxzmp);
plot(izmpx);
legend('local','delxzmp','izmp');



figure
plot(Ffkcomx);
hold on;
plot(Sfkcomx);
plot(Ifkcomx);
plot(I2fkcomx);
plot(fk0comx);
plot(comx);
plot(fkcomx);
legend('Fcom','Scom','Icom','I2com','fk0com','descom','fkcom');

figure
hold on;
plot(comx);
plot(rhx);
plot(fkrhx);
legend('Des comx','Des rhx','FK rhx');