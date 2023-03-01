fid=fopen('./data/fused.bin','rb');
data=fread(fid,inf,'double');
data=reshape(data,8,[]);
data=data';
% data(:,2:4)=data(:,2:4)-data(1,2:4);
%%
figure
plot3(data(:,2),data(:,3),data(:,4))
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
%%
figure
plot(data(:,1),data(:,2))
hold on
plot(data(:,1),data(:,3))
plot(data(:,1),data(:,4))
return
%%
fid=fopen('./data/gps.bin','rb');
data=fread(fid,inf,'double');
data=reshape(data,4,[]);
data=data';
% plot3(data(1:2000,2),data(1:2000,3),data(1:2000,4))

lat=data(:,2);
lon=data(:,3);
h=data(:,4);
[xEast,yNorth,zUp] = geodetic2enu(lat,lon,h,lat(1),lon(1),h(1),wgs84Ellipsoid);
figure
plot3(xEast,yNorth,zUp)
%%
figure
plot(data(:,1),xEast)
hold on
plot(data(:,1),yNorth)
plot(data(:,1),zUp)
%%
path='./data/';
gt=load([path 'gt.txt']','-ascii');
gt_pos=gt(:,4:4:12);
plot3(gt_pos(:,1),gt_pos(:,2),gt_pos(:,3))
legend('fused','gt');
fused_pos=data(:,2:4);
norm(fused_pos-gt_pos,2)