file = load('data.txt');

desCOMx = file(:,19);
desCOMy = file(:,20);

des_RFx = file(:,99);
des_LFx = file(:,101);

figure
plot(des_RFx);