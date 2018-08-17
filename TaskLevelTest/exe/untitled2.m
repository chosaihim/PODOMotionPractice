file = load('data.txt');

% X_ZMP_Local = file(:,1);         
% Y_ZMP_Local = file(:,2);               
% DX = file(:,3);        
% DY = file(:,4);        
% ZMPControlX = file(:,5);                  
% ZMPControlY = file(:,6);                  
% index = file(:,7);
% FRF = file(:,8);
% MRF = file(:,10);
% Center = file(:,12);
% COMx = file(:,13);
% COMy = file(:,14);
% Localx = file(:,15);
% Localy = file(:,16);
% Ix = file(:,17);
% Iy = file(:,18);

% X_ZMP_Local = file(:,106);         
% Y_ZMP_Local = file(:,107);               
% DX = file(:,108);        
% DY = file(:,109);        
% ZMPControlX = file(:,110);                  
% ZMPControlY = file(:,111);                  
% index = file(:,112);
% FRF = file(:,113);
% MRF = file(:,115);
% Center = file(:,117);
% COMx = file(:,118);
% COMy = file(:,119);
% Localx = file(:,120);
% Localy = file(:,121);
% Iy = file(:,123);
% Ix = file(:,122);


figure
plot(index,X_ZMP_Local,index,Y_ZMP_Local,index,DX,index,DY,index,ZMPControlX,index,ZMPControlY);
legend('XZMPLocal','YZMPLocal','DelPC_X','DelPC_Y','ZMPControlX','ZMPControlY');

figure
plot(index,Y_ZMP_Local,index,DY*0.001,index,ZMPControlY);
legend('YZMPLocal','DelPC_Y','ZMPControlY');

figure
plot(index,FRF,index,MRF,index,Center);
legend('FRF','MRF','Center');

figure
plot(index,COMy,index,Localy,index,COMx,index,Localx);
legend('COMy','LOCALy','COMx','LOCALx');

figure
plot(Ix);
legend('Ix');
  