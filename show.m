fid=fopen('./data/fused.bin','rb');
fused_data=fread(fid,inf,'double');
fused_data=reshape(fused_data,8,[]);
fused_data=fused_data';
% fused_data(:,2:4)=fused_data(:,2:4)-fused_data(1,2:4);
%%
figure
plot3(fused_data(:,2),fused_data(:,3),fused_data(:,4),'LineWidth',0.5)
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
hold on
%%
path='./data/';
gt=load([path 'gt.txt']','-ascii');
gt_pos=gt(:,4:4:12);
plot3(gt_pos(:,1),gt_pos(:,2),gt_pos(:,3),'LineWidth',0.5)
fused_pos=fused_data(:,2:4);
norm(fused_pos-gt_pos,2)
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
% figure
plot3(xEast,yNorth,zUp,'.','MarkerSize',5)
legend('fused','ground truth','measured');
xlim([0,600])
return
%%
figure
plot(data(:,1),data(:,2))
hold on
plot(data(:,1),data(:,3))
plot(data(:,1),data(:,4))
return
%%
%%
figure
plot(data(:,1),xEast)
hold on
plot(data(:,1),yNorth)
plot(data(:,1),zUp)
%%