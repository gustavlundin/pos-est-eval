% Read AJ
testfile = 'testdata_3.txt';
% testfile = 'testdata_3.txt';

b_use_sbg = 1;   % Use attitude from SBG 
% Zero coordinate, ONERA, Toulouse
lat0 =  43.57049;
lon0 =  1.47239;
alt0 =  144.7;
sma_earth = 6378137;
ecc_earth = 0;%0.0818;
% testdata = importdata(testfile);
testdata = importdata(testfile);

% startind = 2540;
startind = 5336+1100;
indsel = startind:size(testdata,1);
t_imu = testdata(indsel,1)*0.02;
t_gps = testdata(indsel,2); t_gps = (t_gps-t_gps(1))*10e-4;
lat = testdata(indsel,3);
lon = testdata(indsel,4);
alt = testdata(indsel,5);
P = geo2ned([lat, lon, alt], [lat(1), lon(1), alt0]);
% P = geo2ned(lat', lon', alt', lat(1), lon(1), alt0, sma_earth, ecc_earth);
% P = P';
px = P(:,1); py = P(:,2); pz = P(:,3);
vy = testdata(indsel,6);
vx = testdata(indsel,7);
vz = testdata(indsel,8);
psi = testdata(indsel,9);
theta = testdata(indsel,10);
phi = testdata(indsel,11);
p = testdata(indsel,12);
q = testdata(indsel,13);
r = testdata(indsel,14);
ax = testdata(indsel,15);
ay = testdata(indsel,16);
az = testdata(indsel,17);
state.x = testdata(indsel,37);
state.y = testdata(indsel,38);
state.z = testdata(indsel,39);
baro = testdata(indsel,49);
mx = testdata(indsel,64);
my = testdata(indsel,68);
mz = testdata(indsel,69);
nbsat = testdata(indsel,71);
stat_gps = testdata(indsel,72);

% filename = '\\PERSEPHONE\goman\Matlab\GLR\Toulouse_car_190617\data_vol_17Juin2019_voiture1.txt'; 
% [data, sel] = read_AJ_log(filename);

% tvec = data.timeUp;

t0 = t_imu(1);
tvec = t_imu-t0;
dtvec = 0.02*ones(size(tvec));

% Concatenate vectors
% gyro_in = [tvec, data.IMU.p, data.IMU.q, data.IMU.r];
% acc_in = [tvec, data.IMU.acc_x, data.IMU.acc_y, data.IMU.acc_z];
% magraw_in = [tvec, data.IMU.mag_x, data.IMU.mag_y, data.IMU.mag_z];
% xyz_gps_in = [tvec, data.GPS.x, data.GPS.y, data.GPS.z];
% v_xyz_gps_in = [tvec, data.GPS.vN, data.GPS.vE, data.GPS.vH];

% % Concatenate vectors
gyro_in = [tvec, p, q, r];
acc_in = [tvec, ax, ay, az];
magraw_in = [tvec, mx, my, mz];
xyz_gps_in = [t_gps, px, py, pz];
% v_xyz_gps_in = [t_gps, vx.*(nbsat>0), vy.*(nbsat>0), vz.*(nbsat>0)];
v_xyz_gps_in = [t_gps, vx, vy, vz];
nbsat_in = [tvec, nbsat];

% euler_sbg = [tvec, data.IMU.phi, data.IMU.theta, data.IMU.psi];
euler_sbg = [tvec, phi, theta, psi];
rpy_in = euler_sbg;

%% Calibrate magnetometer
norm_acc = zeros(size(tvec));
norm_magraw = zeros(size(tvec));
norm_vgps = zeros(size(tvec));
for k = 2:numel(dtvec)
    norm_acc(k) = norm(acc_in(k,2:4));
    norm_magraw(k) = norm(magraw_in(k,2:4));
    norm_vgps(k) = norm(v_xyz_gps_in(k,2:4));
end

% isel = 1:20000;
isel_start = find(tvec>278); isel_stop = find(tvec<295);
% [ofs_cal,gain_cal,rotM_cal]=ellipsoid_fit(magraw_in(intersect(isel_start,isel_stop),2:4),3);
% [ofs_cal,gain_cal,rotM_cal]=ellipsoid_fit(magraw_in(intersect(isel_start,isel_stop),2:4),'xyz');

% Find magnetic reference
fprintf('Loading data + calibration...\n')

% load('mk7_magrefcheck.mat')
mag_in = magraw_in;
norm_mag = norm_magraw;

% for kl = 1:size(mag_in,1)
%     mag_in(kl,2:4) = (1./gain_cal).*(rotM_cal*(magraw_in(kl,2:4)'-ofs_cal));
%     norm_mag(kl) = norm(mag_in(kl,2:4));
% end

%% Post treatment
% Filter gyro to find LP-bias
% tau = 1000;
% G = tf([1],[tau 1]);
% pf = lsim(G,gyro_in(:,2),gyro_in(:,1));
% qf = lsim(G,gyro_in(:,3),gyro_in(:,1));
% rf = lsim(G,gyro_in(:,4),gyro_in(:,1));
% gyro_filt = [tvec, pf, qf, rf];

% bias_lpgyro = mean(gyro_filt(end-1000:end,2:4))';
% % % bias_lpgyro = [bias_lpgyro(1:2); (78-28)*pi/180/(2574-2494)];
% bias_lpgyro = [bias_lpgyro(1:2); -10*(pi/180)/50];
% % % bias_lpgyro = zeros(3,1);


psi_gps = zeros(size(tvec));
psi_mag = zeros(size(tvec));


tau = 500;
alpha = 1 - exp(-0.02/tau);
% bhat_lp = bias_lpgyro;





norm_vgps_valid = norm_vgps.*(nbsat>3);
ind_vgps_psibad = find(norm_vgps_valid<0.5);
norm_vgps_psiok = norm_vgps_valid;
norm_vgps_psiok(ind_vgps_psibad) = 0; 

for k = 1:numel(dtvec)
    if norm_vgps_psiok(k) > 0
        psi_gps(k) = atan2(v_xyz_gps_in(k,3),v_xyz_gps_in(k,2));
    end
%     mag_inert = eul2chr([phi(k), theta(k), psi(k)]);
    psi_mag(k) = atan2(mag_in(k,3),mag_in(k,2)); 
end

ind_zerohead = find(abs(psi_gps)<0.04); ind_zero = find(psi_gps==0); 
ind_zh =setxor(ind_zerohead,ind_zero);
mag_ref2 = mean(mag_in(ind_zh,2:4))';

% ==== Heading observer: IMU, GPS fusion ====
% Calculate attitude from gyro
% euler0 = [phi(1);theta(1);60*pi/180];
% q0 = Euler2Quaternion(euler0(1),euler0(2),euler0(3));
% qa = q0;
% euler_in = zeros(size(gyro_in)); euler_in(1,2:4) = euler0';
% % bhat_lp_out(:,1) = bhat_lp;
% tau_psi = 1/3; tau_psi_bias = 500/3;
% kpsi = 1-exp(-0.02/tau_psi);
% kpsi_bias = 1-exp(-0.02/tau_psi_bias);
% h1 = waitbar(0,'Simulating heading observer...');
% niter = numel(dtvec);
% % for k = 2:numel(dtvec)
% %     % Heading observer, IMU GPS fusion
% % %     l_gyro_out = gyro_in(k,2:4)*pinv(diag(EKF.cov.gyro))*gyro_in(k,2:4)';
% % %         % LP filtering of gyro to estimate gyro bias in open loop
% % %     if l_gyro_out < 14
% % %        bhat_lp = sat((1-alpha)*bhat_lp + alpha*gyro_in(k,2:4)',-EKF.bsat.bmax,EKF.bsat.bmax);
% % %     end
% %     [~, ~, psi_qa] = Quaternion2Euler(qa);
% %     qa = Euler2Quaternion(phi(k),theta(k),psi_qa);
% %     % Prediction
% %     qa = quatintegrate(qa, gyro_in(k,2:4)'-bhat_lp,dtvec(k));
% %     % Correction
% %     if abs(psi_gps(k)) > 10e-6
% %         [~, ~, psi_qa] = Quaternion2Euler(qa);    
% %         dpsi = psi_gps(k) - psi_qa;
% %         if abs(dpsi) > pi/6
% %             dpsi = 0;
% %         end
% %         Deltapsi = kpsi*dpsi/2;
% %         dqpsi = [cos(Deltapsi); 0; 0; sin(Deltapsi)];
% % %         bhat_lp(3) = bhat_lp(3) + kpsi_bias*dpsi;
% %         qa = QuaternionProduct(dqpsi,qa);
% %         %psi_qa = psi_qa + kpsi*dpsi;
% %         %qa = Euler2Quaternion(phi(k),theta(k),(psi_gps+psi_qa)/2);
% % 
% % %         qa = QuaternionProduct(psi_gps);
% %     end
% %     %bhat_lp_out(:,k) = bhat_lp;
% %     euler_in(k,2:4) = qua2eul(qa)';
% %     waitbar((k/niter),h1)
% % end,
% close(h1);
euler_in(:,1) = tvec;

if b_use_sbg
    euler_in(:,2:4) = [phi, theta, psi];
end

%% Reconstitude position measurement
% xyz_gps_recon = zeros(size(xyz_gps));
% xyz_gps_tmp = xyz_gps;
% 
% cnt = 1;
% for k = 1:numel(t_gps)
%     if k == 1
%         xyz_gps_recon(cnt,1) = xyz_gps_tmp(k,1);
%         cnt = cnt+1;
%     elseif k>1 && (nbsat(k) > 3 && nbsat(k-1) > 3 ) 
%         xyz_gps_recon(cnt,1) = xyz_gps_tmp(k,1);
%         cnt = cnt+1;
%     elseif k>1 && nbsat(k) > 3 && nbsat(k-1) < 3
%         xyz_gps_tmp(k:end,1) = xyz_gps_tmp(k:end,1) - (xyz_gps_tmp(k,1)-xyz_gps_tmp(k-1,1)); 
%         xyz_gps_recon(cnt,1) = xyz_gps_tmp(k,1);
%         cnt = cnt+1;
%         fprintf('%6.3f\n',t_gps(k));
%     end
% end

%% Plotty

% LAT, LON, ALT +  NB SAT
figure, 
h(1) = subplot(411); plot(tvec,px), 
h(2) = subplot(412); plot(tvec,py) , 
h(3) = subplot(413); plot(tvec,pz), 
h(4) = subplot(414); plot(tvec, nbsat)
linkaxes(h,'x');
clear h

% V GPS
figure,
for k = 1:3
    h(k) = subplot(4,1,k); hold on
    plot(v_xyz_gps_in(:,1),v_xyz_gps_in(:,k+1))
    plot(v_xyz_gps_in(:,1),v_xyz_gps_in(:,k+1).*(nbsat>0))
end
% h(4) = subplot(414); plot(tvec, stat_gps)
h(4) = subplot(414); plot(tvec, nbsat)
linkaxes(h,'x');
clear h

% Gyro
figure,
for k = 1:3
    h(k) = subplot(3,1,k);
    plot(gyro_in(:,1),gyro_in(:,k+1))
end
linkaxes(h,'x');
clear h

% Acc
figure,
for k = 1:3
    h(k) = subplot(3,1,k);
    plot(acc_in(:,1),acc_in(:,k+1))
end
linkaxes(h,'x');
clear h

% Mag
figure,
for k = 1:3
    h(k) = subplot(3,1,k);
    plot(mag_in(:,1),mag_in(:,k+1))
end
linkaxes(h,'x');
clear h

% Euler
figure,
for k = 1:3
    h(k) = subplot(3,1,k); hold on
    plot(euler_in(:,1),euler_in(:,k+1))
    plot(euler_sbg(:,1),euler_sbg(:,k+1))
end
linkaxes(h,'x');
clear h