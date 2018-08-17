file1 = load('data_orin.txt');
file2 = load('data_whole.txt');

startT = 1;
stopT = 1800;
Orin_ZMPx = file1([startT:stopT],1);
Orin_ZMPy = file1([startT:stopT],4);
Orin_COMx = file1([startT:stopT],103);
Orin_COMy = file1([startT:stopT],104);
Orin_RFx = file1([startT:stopT],99);
Orin_RFy = file1([startT:stopT],100);
Orin_RHP = file1([startT:stopT],105);

Whole_ZMPx = file2([startT:stopT],1);
Whole_ZMPy = file2([startT:stopT],4);
Whole_COMx = file2([startT:stopT],103);
Whole_COMy = file2([startT:stopT],104);
Whole_RFx = file2([startT:stopT],99);
Whole_RFy = file2([startT:stopT],100);
Whole_RHP = file2([startT:stopT],105);

figure
subplot(2,1,2);
plot(Orin_RHP*180/3.141592);
%plot(Orin_RFx, Orin_RFy, Orin_ZMPx,Orin_ZMPy);

subplot(2,1,1);
plot(Whole_RHP*180/3.141592);
%plot(Whole_RFx, Whole_RFy,Whole_ZMPx,Whole_ZMPy);
legend('zmp','com');