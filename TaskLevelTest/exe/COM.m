file = load('WBWALK.txt');

descomx = file(:,19);
descomy = file(:,20);
fkcomx = file(:,183);
fkcomy = file(:,184);
zmpx = file(:,1);
zmpy = file(:,2);
delcomx = file(:,17);
delcomy = file(:,18);

figure
hold on;
plot(descomx);
plot(fkcomx);
%plot(zmpy);
plot(delcomx);
legend('des','fk','zmp','del');