%%
bag = rosbag('2021-04-27-15-00-18.bag');
bSel = select(bag,'Topic','/flower_position');

timeseries = bSel.timeseries;
time = timeseries.Time;
positions = timeseries.Data;
normals_pos = [];
for i = 1:size(positions,1)
    normals_pos(i,:) = positions(i,:) / norm(positions(i,:));
end

%%
r_max = 1.50;
r_min = 0.30;
r_n = 7;

th_max = pi/2;
th_min = -pi/2;
th_n = 12;

ph_max = pi/2;
ph_min = -pi/2;
ph_n = 12;

dr = (r_max - r_min) / (r_n-1);  
dth = (th_max - th_min) / (th_n-1);  
dph = (ph_max - ph_min) / (ph_n-1);

r = [];
th = [];
ph = [];

for i=0:r_n-1
    for j=0:th_n-1
        for k=0:ph_n-1
            r = [r; r_min + i*dr];
            th = [th; th_min + j*dth];
            ph = [ph; ph_min + k*dph];
        end
    end
end

%%
close all
% figure  
% hold on
T_o = eye(4);     
% trplot(T_o, 'frame', 'origin','rgb', 'view',[45 45]);
% axis equal 

R = roty(rad2deg(pi/2))*rotz(rad2deg(-pi/2));
T_optical = rt2tr(R, [0,0,0]);
% trplot(T_o*T_base_link*T_optical, 'frame', 'optical','rgb', 'view',[45 45]);


T_base_link = zeros(4,4,size(positions,1));
T_observer = zeros(4,4,size(positions,1));
xyz = [];

flower_pose_at_global_frame = eye(4);
flower_pose_at_global_frame(3,4) = 1;

flower_normal_at_camera_frame = zeros(size(positions,1),3);
for i = 1:size(positions,1)
    xyz(i,1) = r(i) * sin(ph(i)) * cos(ph(i));
    xyz(i,2) = r(i) * sin(ph(i)) * sin(th(i));
    xyz(i,3) = r(i) * cos(ph(i));
    
    gamma = th(i)+pi;
    Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
    
    beta = pi/2-ph(i);
    Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    
    R = Rz*Ry;
    
%     R = rotz(rad2deg(th(i)+pi))*roty(rad2deg(pi/2-ph(i)));
    d = [0,0,0];
%     d = [positions(i,1);positions(i,2);positions(i,3)];
%     d = [xyz(i,1),xyz(i,2),xyz(i,3)];
%     T_base_link(:,:,i) = rt2tr(R, 10*);
    T_base_link(:,:,i) = T_o*rt2tr(R, d');
    T_observer(:,:,i) = T_base_link(:,:,i)*T_optical;
%     h = trplot(T_observer(:,:,i), 'frame', 'observer','rgb', 'view',[45 45]);  
%     pause (0.1)
%     delete(h) 
    
    R_obs = T_observer(1:3,1:3,i); 
    d_obs = T_observer(1:3,4,i);
    
    invT = [R_obs', -R_obs'*d_obs; zeros(1,3), 1];
    
    flower_pose_at_camera_frame = invT*flower_pose_at_global_frame;
    
    flower_normal_at_camera_frame(i,1:3) =  (flower_pose_at_camera_frame(1:3,4)/norm(flower_pose_at_camera_frame(1:3,4)))';

end 

figure, hold on
stairs(time,flower_normal_at_camera_frame(:,1),'r-')
stairs(time,flower_normal_at_camera_frame(:,2),'g-')
stairs(time,flower_normal_at_camera_frame(:,3),'b-')

%%
% gif_filename = 'saved_gif.gif';
% del = 0.1; % time between animation frames
load('pcd_full/filenames.mat')
time_pcd = zeros(size(filenames,1),1);
positions_pcd = zeros(size(filenames,1),3);
normals_pcd = zeros(size(filenames,1),3);

%%
for i= 1:size(filenames,1)
    
    time_pcd(i) = str2double(filenames{i,:}(1:(end-4)));
    s = strcat(filenames{i,:});
    disp(s);
    ptCloud = pcread(strcat('pcd_full/',s));

%     minDistance = 0.1;
%     [labels,numClusters] = pcsegdist(ptCloud,minDistance);
%     pcshow(ptCloud.Location,labels);

    maxDistance = 0.1;
    [~,inliers,outliers] = pcfitplane(ptCloud,maxDistance);
    ptCloudWithoutGround = select(ptCloud,outliers);

    minDistance = 0.01;
    minPoints = 30;
    [labels,numClusters] = pcsegdist(ptCloudWithoutGround,minDistance,'NumClusterPoints',minPoints);

%     pcshow(ptCloudWithoutGround.Location);

    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudWithoutGround,idxValidPoints);

%     figure
%     colormap(hsv(numClusters));
%     pcshow(segmentedPtCloud.Location,labelColorIndex);
%     title('Point Cloud Clusters');
%
    if segmentedPtCloud.Count > 3
        normals = pcnormals(segmentedPtCloud, segmentedPtCloud.Count);
        
%         x = segmentedPtCloud.Location(1:10:end,1);
%         y = segmentedPtCloud.Location(1:10:end,2);
%         z = segmentedPtCloud.Location(1:10:end,3);
        x_mean = mean(segmentedPtCloud.Location(:,1));
        y_mean = mean(segmentedPtCloud.Location(:,2));
        z_mean = mean(segmentedPtCloud.Location(:,3));

        positions_pcd(i,:) = [x_mean, y_mean, z_mean];

        u = normals(1:10:end,1);
        v = normals(1:10:end,2);
        w = normals(1:10:end,3);

        normals_pcd(i,:) = [u(1), v(1), w(1)];
    else
        x_mean = NaN;
        y_mean = NaN;
        z_mean = NaN;
        positions_pcd(i,:) = [NaN, NaN, NaN];
        normals_pcd(i,:) = [NaN, NaN, NaN];
    end
    
%     figure

%     pcshow(segmentedPtCloud.Location)
%     title('Estimated Normals of Point Cloud')
%     hold on
%     plot3(x_mean,y_mean,z_mean,'.','MarkerSize',30)
%     plot3([x_mean, x_mean + 0.03*u(1)],[y_mean, y_mean + 0.03*v(1)],[z_mean, z_mean + 0.03*w(1)])
%     plot3([x_mean, x_mean + 0.03*normals_pos(60,1)],[y_mean, y_mean + 0.03*normals_pos(60,2)],[z_mean, z_mean + 0.03*normals_pos(60,3)])
%     plot3(positions(:,1),positions(:,2),positions(:,3)+0.3)
%     plot3(positions(60,1),positions(60,2),positions(60,3)+0.3,'.','MarkerSize',30)% quiver3(x,y,z,-u,-v,-w);
%     drawnow

%     frame = getframe(gca);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     if i == 1
%         imwrite(imind,cm,gif_filename,'gif','Loopcount',inf,'DelayTime',del);
%     else
%         imwrite(imind,cm,gif_filename,'gif','WriteMode','append','DelayTime',del);
%     end

end

%%

for i=1:size(normals_pcd,1)
    if normals_pcd(i,3) > 0 
        normals_pcd(i,:) = -1*normals_pcd(i,:);
    end
end

%%
% close all

% cov = zeros(5680,3);
% r_full = zeros(5680,1);
% theta_full = zeros(5680,1);
% phi_full = zeros(5680,1);
% 
% counter = 1;
% for i=1:5680
%     r_full(i) = r(counter);
%     theta_full(i) = th(counter);
%     phi_full(i) = ph(counter);
%     cov(i,:) = (normals_pcd(i,:) - flower_normal_at_camera_frame(counter,1:3)).^2;
%     if (time_pcd(i) >= time(counter))
%         counter = counter + 1;
%     end
% end
% 
% cov_x = cov(:,1);
% cov_y = cov(:,2);
% cov_y(cov_y>2)=NaN;
% cov_z = cov(:,3);

%%

figure, hold on
plot(time_pcd, positions_pcd(:,1),'r.')
plot(time_pcd, positions_pcd(:,2),'g.')
plot(time_pcd, positions_pcd(:,3),'b.')

%%
figure
subplot(411)
hold on
plot(time_pcd(1:5680), phi_full,'k-')
plot(time_pcd(1:5680), theta_full,'r-')
plot(time_pcd(1:5680), r_full,'b-')
xlim([0 1190])
legend('\phi [deg]','\theta [deg]','r [m]')

subplot(412)
hold on
stairs(time,flower_normal_at_camera_frame(:,1),'r-')
stairs(time,flower_normal_at_camera_frame(:,2),'g-')
stairs(time,flower_normal_at_camera_frame(:,3),'b-')
xlim([0 1190])
legend('n_x','n_y','n_z')

subplot(413)
hold on
plot(time_pcd, normals_pcd(:,1),'r.')
plot(time_pcd, normals_pcd(:,2),'g.')
plot(time_pcd, normals_pcd(:,3),'b.')
xlim([0 1190])
legend('n_x (pcd)','n_y (pcd)','n_z (pcd)')

subplot(414)
hold on
plot(time_pcd(1:5680), cov_x,'r-')
plot(time_pcd(1:5680), cov_y,'g-')
plot(time_pcd(1:5680), cov_z,'b-')
xlim([0 1190])
legend('cov_{nx}','cov_{ny}','cov_{nz}')


%%
close all

figure
subplot(331)
bubblechart(theta_full(1:820),phi_full(1:820),cov_x(1:820));
xlabel('\theta')
ylabel('\phi')
subplot(332)
bubblechart(theta_full(1:820),phi_full(1:820),cov_y(1:820));
title('r=0.3')
xlabel('\theta')
ylabel('\phi')
subplot(333)
bubblechart(theta_full(1:820),phi_full(1:820),cov_z(1:820));
xlabel('\theta')
ylabel('\phi')

subplot(334)
bubblechart(theta_full(821:1625),phi_full(821:1625),cov_x(821:1625));
xlabel('\theta')
ylabel('\phi')
subplot(335)
bubblechart(theta_full(821:1625),phi_full(821:1625),cov_y(821:1625));
title('r=0.5')
xlabel('\theta')
ylabel('\phi')
subplot(336)
bubblechart(theta_full(821:1625),phi_full(821:1625),cov_z(821:1625));
xlabel('\theta')
ylabel('\phi')

subplot(337)
bubblechart(theta_full(1626:2439),phi_full(1626:2439),cov_x(1626:2439));
xlabel('\theta')
ylabel('\phi')
subplot(338)
bubblechart(theta_full(1626:2439),phi_full(1626:2439),cov_y(1626:2439));
title('r=0.7')
xlabel('\theta')
ylabel('\phi')
subplot(339)
bubblechart(theta_full(1626:2439),phi_full(1626:2439),cov_z(1626:2439));
xlabel('\theta')
ylabel('\phi')

figure
subplot(331)
bubblechart(theta_full(2440:3257),phi_full(2440:3257),cov_x(2440:3257));
xlabel('\theta')
ylabel('\phi')
subplot(332)
bubblechart(theta_full(2440:3257),phi_full(2440:3257),cov_y(2440:3257));
title('r=0.9')
xlabel('\theta')
ylabel('\phi')
subplot(333)
bubblechart(theta_full(2440:3257),phi_full(2440:3257),cov_z(2440:3257));
xlabel('\theta')
ylabel('\phi')

subplot(334)
bubblechart(theta_full(3258:4074),phi_full(3258:4074),cov_x(3258:4074));
xlabel('\theta')
ylabel('\phi')
subplot(335)
bubblechart(theta_full(3258:4074),phi_full(3258:4074),cov_y(3258:4074));
title('r=1.1')
xlabel('\theta')
ylabel('\phi')
subplot(336)
bubblechart(theta_full(3258:4074),phi_full(3258:4074),cov_z(3258:4074));
xlabel('\theta')
ylabel('\phi')

subplot(337)
bubblechart(theta_full(4075:4879),phi_full(4075:4879),cov_x(4075:4879));
xlabel('\theta')
ylabel('\phi')
subplot(338)
bubblechart(theta_full(4075:4879),phi_full(4075:4879),cov_y(4075:4879));
title('r=1.3')
xlabel('\theta')
ylabel('\phi')
subplot(339)
bubblechart(theta_full(4075:4879),phi_full(4075:4879),cov_z(4075:4879));
xlabel('\theta')
ylabel('\phi')


figure
subplot(131)
bubblechart(theta_full(4880:5680),phi_full(4880:5680),cov_x(4880:5680));
xlabel('\theta')
ylabel('\phi')
subplot(132)
bubblechart(theta_full(4880:5680),phi_full(4880:5680),cov_y(4880:5680));
title('r=1.5')
xlabel('\theta')
ylabel('\phi')
subplot(133)
bubblechart(theta_full(4880:5680),phi_full(4880:5680),cov_z(4880:5680));
xlabel('\theta')
ylabel('\phi')


%%

cov_x_fit = griddata(r_full,theta_full,phi_full,cov_x,r,th,ph)
cov_y_fit = griddata(r_full,theta_full,phi_full,cov_y,r,th,ph)
cov_z_fit = griddata(r_full,theta_full,phi_full,cov_z,r,th,ph)

