RAWDATAimport = importdata("Book7.xlsx");
RAWDATA = RAWDATAimport.data;

DataStruct = struct();
% Data order Accel - Gyro - Mag - Temp - Press - Humid - (UV) 1 - 2 - 3
% SYNC
AX = RAWDATA(:,1);
AY = RAWDATA(:,2);
AZ = RAWDATA(:,3);
GX = RAWDATA(:,4);
GY = RAWDATA(:,5);
GZ = RAWDATA(:,6);
MX = RAWDATA(:,7);
MY = RAWDATA(:,8);
MZ = RAWDATA(:,9);
Temp = RAWDATA(:,10);
Press = RAWDATA(:,11);
Humid = RAWDATA(:,12);
UV1 = RAWDATA(:,13);
UV2 = RAWDATA(:,14);
UV3 = RAWDATA(:,15);
SYNC = RAWDATA(:,16);

DataStruct.A.Ax = AX;
DataStruct.A.Ay = AY;
DataStruct.A.Az = AZ;
DataStruct.M.Mx = MX;
DataStruct.M.My = MY;
DataStruct.M.Mz = MZ;
DataStruct.G.Gx = GX;
DataStruct.G.Gy = GY;
DataStruct.G.Gz = GZ;

points = length(AX);

seconds = 1:1:points;
figure(1)
plot(seconds, [AX,AY,AZ]);
figure(2)
plot(seconds, [GX,GY,GZ]);
figure(3)
plot(seconds, [MX,MY,MZ]);
figure(4)
