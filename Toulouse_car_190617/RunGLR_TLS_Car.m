% read_car
clear u_in y_in z_in zcov_in

load euler_gyro_out
load xhat_out_A2B

clk = fix(clock);
time_str_short = ['-', num2str(clk(1)), '-' ,num2str(clk(2)), '-', ...
    num2str(clk(3)), '-', num2str(clk(4)), '-', num2str(clk(5))];

expart = '2';

simcase = 'A2'; % A1 = Pos meas. only, A2 = Pos + Spd meas
r2d = 180/pi;
d2r = 1/r2d;
b_simu = 0;
% Init sim
if strcmp(simcase,'V1') || strcmp(simcase,'V1B')
    dt = 0.2;
    dty = dt;
    dtvec = [t_gps(2)-t_gps(1); diff(t_gps)];
else
    dt = 0.02;
    dty = 10*dt;
    dtvec = [t_imu(2)-t_imu(1); diff(t_imu)];
end

% Measurement noise
% pos_std = 1*[1*[0.05, 0.05], 0.05]/3; % 10*
% vel_std = 1*[1*[0.04, 0.04], 0.04]/3; % 0.1*

pos_std = [5*[1, 1], 10]/3; % 10*
vel_std = [1*[1, 1], 1]/3; % 0.1*

% Input noise
u_in_acc_std = 5*diag([1, 1, 1]/3);
u_in_spd_std = diag(vel_std);
acc_bias_cov = (1/3/sqrt(60))^2;


t_vgps = t_gps;

sel_gps = 1:size(v_xyz_gps_in(1));
sel_vgps = 1:size(v_xyz_gps_in(1));

t_gps_cut = t_gps;
t_vgps_cut = t_vgps;

dtcorr = 0; % Shift between avionic clock and gps clock

v_xyz_gps = v_xyz_gps_in(:,2:4);
xyz_gps = xyz_gps_in(:,2:4);
% t_imu = avionic_filtered.timeUp-avionic_filtered.timeUp(1);
% tau = 1; Gd = c2d(tf([tau 0],[tau 1]),dt); % Discrete washout filter
%% Synchronise GPS data between avionic and novatel

% sbas_z_tmp = interp1(novatel_BESTPOS.t_epoch0, novatel_BESTPOS.data.hgt, t_imu,'pchip',novatel_BESTPOS.data.hgt(1))-novatel_BESTPOS.data.hgt(1)-(avionic_filtered.GPS.z(1));
% 
% rtk_z_tmp = interp1(t_imu(t_imu<t_imu(end)*0.6), -avionic_filtered.GPS.z(t_imu<t_imu(end)*0.6), t_imu,'pchip',0);
% 
% [C, LAGS] = xcorr(sbas_z_tmp, rtk_z_tmp);
% [~, imaxx] = max(C);
% 
% deltat = lags(imaxx); 
% % figure, 
% % plot(t_imu, sbas_z_tmp, t_imu, [rtk_z_tmp(-deltat:end-1); rtk_z_tmp(end)*ones(abs(deltat),1)])
% 
% dtcorr = t_imu(abs(deltat));

%% Fix data/nbsat synchronisation issue

dtgps = [0;diff(t_gps)];
ind_dtgps = find(abs(dtgps)>0.01); % Extract valid timestamps (new data)

% nbsat_valid = nbsat(ind_dtgps); % Vaild nbsat echelons
% vcorr = v_xyz_gps_in(ind_dtgps,:);
% for k = 2:size(vcorr,1)-1
%     if nbsat_valid(k+1) < 4 && nbsat_valid(k) > 0
%         vcorr(k,2:4) = 0;
%     end
%     if nbsat_valid(k) > 0 && nbsat_valid(k-1) < 4
%         vcorr(k,2:4) = 0;
%     end
% end
% pcorr = xyz_gps_in(ind_dtgps,:);
% figure, 
% for k = 1:3; h(k) = subplot(3,1,k); hold on,
%     plot(v_xyz_gps_in(:,1),v_xyz_gps_in(:,k+1))
%     plot(vcorr(:,1),vcorr(:,k+1))
% end
% linkaxes(h,'x')
% figure, 
% for k = 1:3; h(k) = subplot(3,1,k); hold on,
%     plot(xyz_gps_in(:,1),xyz_gps_in(:,k+1))
%     plot(pcorr(:,1),pcorr(:,k+1))
% end
% linkaxes(h,'x')

%% Run velocity observer for accelerometer bias estimation
flagt = 1; param = 0; 
V_mes = v_xyz_gps(:,:); 
t_mes = t_gps(:); 
V_ini = v_xyz_gps(1,1:3); 
E_ang = [psi, theta, phi]; 
A_meas = acc_in(:,2:4); t = t_imu;
% [V_est,b_est,Am_est,A_est] = Observateur_Vitesse(t,A_meas,E_ang,V_ini,t_mes,V_mes,param,flagt);


%%
selly = 1:size(t_imu,1);
t_imu_cut = t_imu(selly)-t_imu(selly(1));

% i_avi_end  = find()
% selly = [1:numel(avionic_filtered.timeUp)];

acc_inert_xyz = zeros(numel(selly),3);
acc_inert_xyz_imu = zeros(numel(selly),3);
acc_inert_xyz_gyro = zeros(numel(selly),3);

% euler_gyro_sel = euler_gyro_out.signals.values(selly,:);


% acc_xyz = [avionic.IMU.acc_x(selly(1)) avionic.IMU.acc_y(selly(1)) avionic.IMU.acc_z(selly(1));
%            avionic.IMU.acc_x(selly) avionic.IMU.acc_y(selly) avionic.IMU.acc_z(selly)];
acc_xyz = acc_in(:,2:4);
norm_acc = zeros(size(acc_xyz,1),1);       
% euler = euler_in(:,4:-1:2);
% for k=1:numel(selly)
%     norm_acc(k) = norm(acc_xyz(k,:));
%     acc_inert_xyz_imu(k,1:3) = (eul2chr(euler(k,:))*acc_xyz(k,1:3)')' - [0 0 -9.81];
%     acc_inert_xyz_imu(k,1:3) = acc_inert_xyz_imu(k,1:3) - [0.0305, 0.0012, 0.1691];% - [0.0688, 0.0210, 0.1697];% -[-0.4300   -0.5830   -9.6334];
% %     acc_inert_xyz_gyro(k,1:3) = (eul2chr(euler_gyro_sel(k,:))*acc_xyz(k,1:3)')' - [0 0 9.81];% -[-0.4300   -0.5830   -9.6334];
% end
for k=1:numel(selly)
    norm_acc(k) = norm(acc_xyz(k,:));
    %euler(k,:) = [avionic.IMU.phi(selly(k)), avionic.IMU.theta(selly(k)), avionic.IMU.psi(selly(k))];
    
    acc_inert_xyz(k,1:3) = (Euler2RotMat(euler_in(k,2),euler_in(k,3),euler_in(k,4))*acc_xyz(k,1:3)')' - [0 0 -9.81];
end
% Debia acceleration measurement
% [b,a] = butter(1,0.0001,'low');
% accbias_inert = filtfilt(b,a,acc_inert_xyz);
acc_inert_xyz = acc_inert_xyz;% - accbias_inert;

% acc_inert_xyz = acc_inert_xyz_imu;

% acc_inert_xyz = acc_inert_xyz_gyro;

% Generate biases
% % 3D trajectory- Input: Acc, Meas: Novatel SBAS
switch simcase
    case {'A1','A2','A2B'}
        u_in.time = t_imu_cut;
        u_in.signals.values = acc_inert_xyz;
        u_in.signals.dimension = size(u_in.signals.values);
    case {'V1', 'V1B'}
        u_in.time = t_gps_cut;
        u_in.signals.values = v_xyz_gps;
        u_in.signals.dimension = size(u_in.signals.values);
        
%         u_in.time = vcorr(:,1);
%         u_in.signals.values = vcorr(:,2:4);
%         u_in.signals.dimension = size(u_in.signals.values);
%         dt = 0.2;
%         u_in.time = t_imu_cut;
%         u_in.signals.values = v_xyz_gps_rtk;
%         u_in.signals.dimension = size(u_in.signals.values);
end

% z_in.time = t_gps(sel_gps)-t_gps(sel_gps(1));
% z_in.signals.values = xyz_gps(sel_gps,:);
% z_in.signals.dimension = size(z_in.signals.values);

switch simcase
    case {'A1','V1', 'V1B'}
        dty = dt;
        z_in.time = t_gps_cut;
        z_in.signals.values = xyz_gps;
        z_in.signals.dimension = size(z_in.signals.values);
        
        y_in.time = t_gps_cut;
        y_in.signals.values = xyz_gps;
        y_in.signals.dimension = size(y_in.signals.values);
        
%         z_in.time = pcorr(:,1);
%         z_in.signals.values = pcorr(:,2:4);
%         z_in.signals.dimension = size(z_in.signals.values);
        
        zcov_in.time = [0];
        zcov_in.signals.values = [pos_std].^2;
        zcov_in.signals.dimensions = size(zcov_in.signals.values);
        
%         zcov_in.time = t_gps_cut;
%         zcov_in.signals.values = xyz_gps_sigma.^2;
%         zcov_in.signals.dimensions = size(zcov_in.signals.values);
        
%         zcov_in = [t_gps_cut,  xyz_gps_sigma.^2];
    case {'A2','A2B'}
        dty = 10*[dt;dt];
        z_in.time = t_gps_cut;
        z_in.signals.values = [xyz_gps(:,1) v_xyz_gps(:,1),...
                               xyz_gps(:,2) v_xyz_gps(:,2),...
                               xyz_gps(:,3) v_xyz_gps(:,3)];
        z_in.signals.dimension = size(z_in.signals.values);
        
        y_in.time = t_gps_cut;
        y_in.signals.values = [xyz_gps(:,1) v_xyz_gps(:,1),...
                               xyz_gps(:,2) v_xyz_gps(:,2),...
                               xyz_gps(:,3) v_xyz_gps(:,3)];
        y_in.signals.dimension = size(y_in.signals.values);
        
%         z_in.time = vcorr(:,1);
%         z_in.signals.values = [pcorr(:,2) vcorr(:,2),...
%                                pcorr(:,3) vcorr(:,3),...
%                                pcorr(:,4) vcorr(:,4)];
%         z_in.signals.dimension = size(z_in.signals.values);
%         z_in.time = t_imu_cut;
%         z_in.signals.values = [xyz_gps_rtk(:,1) v_xyz_gps_rtk(:,1),...
%                                xyz_gps_rtk(:,2) v_xyz_gps_rtk(:,2),...
%                                xyz_gps_rtk(:,3) v_xyz_gps_rtk(:,3)];
%         z_in.signals.dimension = size(z_in.signals.values);
        
        zcov_in.time = [0];
        zcov_in.signals.values = ([pos_std(1), vel_std(1),...
                                   pos_std(2), vel_std(2),...
                                   pos_std(3), vel_std(3)]).^2;
        zcov_in.signals.dimensions = size(zcov_in.signals.values);
        
%         z_in.time = xhat_out.time;
%         z_in.signals.values = [
%                 interp1(t_gps_cut,xyz_gps(:,1), xhat_out.time), xhatbiasdata(:,2),...
%                 interp1(t_gps_cut,xyz_gps(:,2), xhat_out.time), xhatbiasdata(:,4),...
%                 interp1(t_gps_cut,xyz_gps(:,3), xhat_out.time), xhatbiasdata(:,6)];
%         z_in.signals.dimension = size(z_in.signals.values);
%         zcov_in = [t_gps_cut, xyz_gps_sigma(:,1), v_xyz_gps_sigma(:,1),...
%                               xyz_gps_sigma(:,2), v_xyz_gps_sigma(:,2),...
%                               xyz_gps_sigma(:,3), v_xyz_gps_sigma(:,3)].^2;
end



dt_in.time = t_vgps_cut;
dt_in.signals.values = dtvec(sel_vgps);
dt_in.dimensions = size(dt_in.signals.values);

%% Split experiment into useable parts

if strcmp(testfile,'testdata_3.txt')
    % Segment 1
    indt1 = [0 522];
    induu = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    indzz = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    indyy = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    u_in_1.time = u_in.time(induu); dtuu = u_in.time(1); u_in_1.time = u_in.time(induu)-dtuu;
    u_in_1.signals.values = u_in.signals.values(induu,:);
    z_in_1.time = z_in.time(indzz); dtzz = z_in.time(1); z_in_1.time = z_in.time(indzz)-dtzz;
    z_in_1.signals.values = z_in.signals.values(indzz,:); dzz = z_in_1.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_1.time); z_in_1.signals.values(k,[1 3 5]) = z_in_1.signals.values(k,[1 3 5]) - dzz; end
    y_in_1.time = y_in.time(indyy); dtyy = y_in.time(1); y_in_1.time = y_in.time(indyy)-dtyy;
    y_in_1.signals.values = y_in.signals.values(indyy,:);% dyy = y_in_1.signals.values(1,:); y_in_1.signals.values = y_in_1.signals.values - dyy;
    ind0 = find(induu>0,1);
    llh0_1 = [lat(ind0), lon(ind0), alt(ind0)];
%     
    % Segment 2
    indt2 = [530 810];
    induu = (u_in.time > indt2(1)) & (u_in.time < indt2(2));
    indzz = (z_in.time > indt2(1)) & (z_in.time < indt2(2));
    indyy = (y_in.time > indt2(1)) & (y_in.time < indt2(2));
    u_in_2.time = u_in.time(induu); dtuu = u_in_2.time(1); u_in_2.time = u_in_2.time-dtuu;
    u_in_2.signals.values = u_in.signals.values(induu,:);
    z_in_2.time = z_in.time(indzz); dtzz = z_in_2.time(1); z_in_2.time = z_in_2.time-dtzz;
    z_in_2.signals.values = z_in.signals.values(indzz,:); dzz = z_in_2.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_2.time); z_in_2.signals.values(k,[1 3 5]) = z_in_2.signals.values(k,[1 3 5]) - dzz; end
    y_in_2.time = y_in.time(indyy); dtyy = y_in_2.time(1); y_in_2.time = y_in_2.time-dtyy;
    y_in_2.signals.values = y_in.signals.values(indyy,:); %dyy = y_in_2.signals.values(1,:); y_in_2.signals.values = y_in_2.signals.values - dyy;
    ind0 = find(induu>0,1);
    llh0_2 = [lat(ind0), lon(ind0), alt(ind0)];
    
    % Segment 3
    indt3 = [950 1705];
    induu = (u_in.time > indt3(1)) & (u_in.time < indt3(2));
    indzz = (z_in.time > indt3(1)) & (z_in.time < indt3(2));
    indyy = (y_in.time > indt3(1)) & (y_in.time < indt3(2));
    u_in_3.time = u_in.time(induu); dtuu = u_in_3.time(1); u_in_3.time = u_in_3.time-dtuu;
    u_in_3.signals.values = u_in.signals.values(induu,:);
    z_in_3.time = z_in.time(indzz); dtzz = z_in_3.time(1); z_in_3.time = z_in_3.time-dtzz;
    z_in_3.signals.values = z_in.signals.values(indzz,:); dzz = z_in_3.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_3.time); z_in_3.signals.values(k,[1 3 5]) = z_in_3.signals.values(k,[1 3 5]) - dzz; end
    y_in_3.time = y_in.time(indyy); dtyy = y_in_3.time(1); y_in_3.time = y_in_3.time-dtyy;
    y_in_3.signals.values = y_in.signals.values(indyy,:);
    ind0 = find(induu>0,1);
    llh0_3 = [lat(ind0), lon(ind0), alt(ind0)];
    
    % Select segment
    % Select segment
    switch expart
        case '1'
            u_in = u_in_1;
            z_in = z_in_1;
            y_in = y_in_1;
            llh0 = llh0_1;
        case '2'
            u_in = u_in_2;
            z_in = z_in_2;
            y_in = y_in_2;
            llh0 = llh0_2;
        case '3'
            u_in = u_in_3;
            z_in = z_in_3;
            y_in = y_in_3;
            llh0 = llh0_3;
    end

     
elseif strcmp(testfile,'testdata_2.txt')
    % Segment 1
    indt1 = [570 895];
    induu = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    indzz = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    indyy = (u_in.time > indt1(1)) & (u_in.time < indt1(2));
    u_in_1.time = u_in.time(induu); dtuu = u_in_1.time(1); u_in_1.time = u_in.time(induu)-dtuu;
    u_in_1.signals.values = u_in.signals.values(induu,:);
    z_in_1.time = z_in.time(indzz); dtzz = z_in_1.time(1); z_in_1.time = z_in.time(indzz)-dtzz;
    z_in_1.signals.values = z_in.signals.values(indzz,:); dzz = z_in_1.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_1.time); z_in_1.signals.values(k,[1 3 5]) = z_in_1.signals.values(k,[1 3 5]) - dzz; end
    y_in_1.time = y_in.time(indyy); dtyy = y_in_1.time(1); y_in_1.time = y_in.time(indyy)-dtyy;
    y_in_1.signals.values = y_in.signals.values(indyy,:);% dyy = y_in_1.signals.values(1,:); y_in_1.signals.values = y_in_1.signals.values - dyy;
    ind0 = find(induu>0,1);
    llh0_1 = [lat(ind0), lon(ind0), alt(ind0)];
%     
    % Segment 2
    indt2 = [1025 2075];
    induu = (u_in.time > indt2(1)) & (u_in.time < indt2(2));
    indzz = (z_in.time > indt2(1)) & (z_in.time < indt2(2));
    indyy = (y_in.time > indt2(1)) & (y_in.time < indt2(2));
    u_in_2.time = u_in.time(induu); dtuu = u_in_2.time(1); u_in_2.time = u_in_2.time-dtuu;
    u_in_2.signals.values = u_in.signals.values(induu,:);
    z_in_2.time = z_in.time(indzz); dtzz = z_in_2.time(1); z_in_2.time = z_in_2.time-dtzz;
    z_in_2.signals.values = z_in.signals.values(indzz,:); dzz = z_in_2.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_2.time); z_in_2.signals.values(k,[1 3 5]) = z_in_2.signals.values(k,[1 3 5]) - dzz; end
    y_in_2.time = y_in.time(indyy); dtyy = y_in_2.time(1); y_in_2.time = y_in_2.time-dtyy;
    y_in_2.signals.values = y_in.signals.values(indyy,:); %dyy = y_in_2.signals.values(1,:); y_in_2.signals.values = y_in_2.signals.values - dyy;
    ind0 = find(induu>0,1);
    llh0_2 = [lat(ind0), lon(ind0), alt(ind0)];
    
    % Segment 3
    indt3 = [2170 2600];
    induu = (u_in.time > indt3(1)) & (u_in.time < indt3(2));
    indzz = (z_in.time > indt3(1)) & (z_in.time < indt3(2));
    indyy = (y_in.time > indt3(1)) & (y_in.time < indt3(2));
    u_in_3.time = u_in.time(induu); dtuu = u_in_3.time(1); u_in_3.time = u_in_3.time-dtuu;
    u_in_3.signals.values = u_in.signals.values(induu,:);
    z_in_3.time = z_in.time(indzz); dtzz = z_in_3.time(1); z_in_3.time = z_in_3.time-dtzz;
    z_in_3.signals.values = z_in.signals.values(indzz,:); dzz = z_in_3.signals.values(1,[1 3 5]); 
    for k = 1:numel(z_in_3.time); z_in_3.signals.values(k,[1 3 5]) = z_in_3.signals.values(k,[1 3 5]) - dzz; end
    y_in_3.time = y_in.time(indyy); dtyy = y_in_3.time(1); y_in_3.time = y_in_3.time-dtyy;
    y_in_3.signals.values = y_in.signals.values(indyy,:);
    ind0 = find(induu>0,1);
    llh0_3 = [lat(ind0), lon(ind0), alt(ind0)];
    
    % Select segment
    switch expart
        case '1'
            u_in = u_in_1;
            z_in = z_in_1;
            y_in = y_in_1;
            llh0 = llh0_1;
        case '2'
            u_in = u_in_2;
            z_in = z_in_2;
            y_in = y_in_2;
            llh0 = llh0_2;
        case '3'
            u_in = u_in_3;
            z_in = z_in_3;
            y_in = y_in_3;
            llh0 = llh0_3;
    end
     
end
    
%% 
switch simcase
    case {'V1','V1B'}
        u_in_std = u_in_spd_std;
    case {'A1','A2','A2B'}
        u_in_std = u_in_acc_std;
end

% System dynamics
switch simcase
    case 'A1'
        A = [1 dt; 0 1];
        B = [dt^2/2; dt];
        C = [1 0];
        D = 0;
        F = 1;
        Vu = eye(2);
    case 'V1'
        A = 1;
        B = dt;
        C = 1;
        D = 0;
        F = 1;
        Vu = 1;
    case 'V1B'
        tau_b = 5;
        A = [1 -dt; 0 1-dt/tau_b];
        B = [dt; 0];
        C = 1;
        D = 0;
        F = 1;
        Vu = diag([1,acc_bias_cov]);
    case 'A2' %
        A = [1 dt; 0 1];
        B = [dt^2/2; dt];
        C = [1 0; 0 1];
        D = 0;
        F = [1; 0];
        Vu = eye(2);
%         F = [1 0; 0 1];
    case 'A2B' % Acceleration bias model with conv to zero
        tau_b = 10; % [s]
        A = [1 dt 0; 
             0 1 0; 
             0 0 1];
        B = [dt^2/2; 0; dt];
        C = [1 0 0; 0 0 1];
        D = 0;
        F = [1; 0];
        Vu = diag([1,acc_bias_cov,1]);
%         tau_b = 5; % [s]
%         A = [1 dt 0; 
%              0 1 0; 
%              0 0 1-dt/tau_b];
%         B = [dt^2/2; dt; 0];
%         C = [1 0 0; 0 1 1];
%         D = 0;
%         F = [1; 0];
end


% tsim = t_gps_cut(end);
tsim = u_in.time(end);  

b_in = [0, zeros(1,size(F,2)*3)];


% Dynamics
% Init system
syskf.dt = dt;
syskf.A = A;
syskf.B = B;
syskf.C = C;
syskf.D = D;
syskf.F = F;
syskf.Q = syskf.B*(syskf.B')*Vu;

% Dimensions
nu =    size(syskf.B,2);        % Input measurement
nx =    size(syskf.A,1);        % State
ny =    size(syskf.C,1);        % Ouput measurement
nb =    size(syskf.F,2);        % Biais to be detected

clear z_in_std
z_in_pos_std = diag(pos_std);
z_in_vel_std = diag(vel_std);
switch simcase
    case {'V1','V1B'}
        for k = 1:3
            z_in_std(k) = z_in_pos_std(k);
        end
    case {'A1','A2','A2B'}
        z_in_std = [z_in_pos_std(1), z_in_vel_std(1),...
                    z_in_pos_std(2), z_in_vel_std(2),...
                    z_in_pos_std(3), z_in_vel_std(3)];
end

% Modèle axe x
syskfx = syskf;
syskfx.zind = (1-1)*ny+1:1*ny;
syskfx.bind = (1-1)*nb+1:1*nb;
syskfx.uind = 1;
syskfx.Q = syskfx.B * u_in_std(1,1)^2 *(syskfx.B');
syskfx.R = syskfx.C *(syskfx.C') * diag(z_in_std(syskfx.zind).^2);

% Modèle axe y
syskfy = syskf;
syskfy.zind = (2-1)*ny+1:2*ny;
syskfy.bind = (2-1)*nb+1:2*nb;
syskfy.uind = 2;
syskfy.Q = syskfy.B * u_in_std(2,2)^2 *(syskfy.B');
syskfy.R = syskfy.C *(syskfy.C') * diag(z_in_std(syskfy.zind).^2);

% Modèle axe z
syskfz = syskf;
syskfz.zind = (3-1)*ny+1:3*ny;
syskfz.bind = (3-1)*nb+1:3*nb;
syskfz.uind = 3;
syskfz.Q = syskfz.B * u_in_std(3,3)^2 *(syskfz.B');
syskfz.R = syskfz.C *(syskfz.C') * diag(z_in_std(syskfz.zind).^2);

%%
%==========================================================================
% Initialisation du filtre
%==========================================================================

%--------------------------------------------------------------------------
% Caractéristiques des pannes
%--------------------------------------------------------------------------
pfa_pwr = 4;
% Fault detection probabilities
p_fa = 10^(-pfa_pwr);                     % False alarm rate
% p_fa = 1-0.9967;
% Smallest detection bias threshold
sd = Confidence_Set(1-p_fa,nb);     % Confidence std
l_det = sd^2;                       % Pour définir le plus petit biais détectable
% l_det = 50;

%--------------------------------------------------------------------------
% Algorithme du GLR
%--------------------------------------------------------------------------
% ***** WARNING ***** mic-mac sur nb et ny à vérifier
% ***** WARNING ***** modèle d'état 1D. Fonctionne aussi en 3D?
% ***** WARNING ***** Il faudrait Lest = 2*L d'après la théorie?

% Sliding window length
Lt      = 5;           % GLR horizon in s
L       = int16(Lt/min(dty));	% Detection window length
Lest	= L;            % Estimation window length

% L = 10;
% Lest = 10;

lamff = 1; % RLS Forgetting factor lambda

paramGLR.l_det = l_det;
paramGLR.GLRthr = l_det;
paramGLR.lamff = lamff;
paramGLR.L = L;
paramGLR.Lest = Lest;
paramGLR.nmax = 10;
paramGLR.table_full = -double(dec2bin(0:(2^(paramGLR.nmax+1)-1)) - '0');
paramGLR.M = floor(min(L/2,5));
paramGLR.BUSESEQELIM = 1;
paramGLR.BUSEGLOBELIM = 1;
paramGLR.BUSETOTELIM = 1;

paramKF = syskfx;
paramKF.dt = dt;
paramKF.dty = dty;

paramSIM.dt = dt;
paramSIM.T = tsim;

switch simcase
    case {'A1','V1', 'V1B'}
        init.xhat	= [z_in.signals.values(1,1:3); zeros(nx-1,1)];
    case {'A2','A2B'}
        init.xhat	= [z_in.signals.values(1,:)];
end

% init.xhat	= [z_in.signals.values(1,3); zeros(nx-1,1)];
init.P      = 10*eye(nx);

init.mu     = zeros(nx,nb*L);
init.muest  = zeros(nx,nb*Lest);

init.nuhatest	= zeros(nb,Lest);
init.Rest       = zeros(nb,nb*Lest);
init.qest       = -ones(1,Lest);

init.nuhat      = zeros(nb,L);      % filter bank estimation [nb x L]
init.LAMb       = zeros(nb,nb*L);	% filter bank estimation information [nb x nb*L]
init.LAMest       = zeros(nb,nb*Lest);	% filter bank estimation information [nb x nb*L]
% init.lhat   = zeros(1,L);
init.q          = 0:-1:-(L-1);      % Filter bank age [1 x L]

init.qLS = 0:-1:-(Lest-1);

initx = init;
inity = init;
initz = init;

initGLRx = init;
initGLRy = init;
initGLRz = init;

% % ***** WARNING ***** Qu'est-ce que c'est?
% nsigma = 7; % Outlier rejection threshold [sigma]
% lamff = 1; % RLS Forgetting factor lambda



%--------------------------------------------------------------------------
% Filtre de Kalman
%--------------------------------------------------------------------------

switch simcase
    case {'A1','V1','V1B'}
        initGLRx.xhat	= [z_in.signals.values(1,1); zeros(nx-1,1)];
        initGLRy.xhat	= [z_in.signals.values(1,2); zeros(nx-1,1)];
        initGLRz.xhat	= [z_in.signals.values(1,3); zeros(nx-1,1)];
        initx.xhat      = [z_in.signals.values(1,1); zeros(nx-1,1)];
        inity.xhat      = [z_in.signals.values(1,2); zeros(nx-1,1)];
        initz.xhat      = [z_in.signals.values(1,3); zeros(nx-1,1)];
    case {'A2','A2B'}
        initGLRx.xhat	= [z_in.signals.values(1,1); zeros(nx-1,1)];
        initGLRy.xhat	= [z_in.signals.values(1,3); zeros(nx-1,1)];
        initGLRz.xhat	= [z_in.signals.values(1,5); zeros(nx-1,1)];
        initx.xhat      = [z_in.signals.values(1,1); zeros(nx-1,1)];
        inity.xhat      = [z_in.signals.values(1,3); zeros(nx-1,1)];
        initz.xhat      = [z_in.signals.values(1,5); zeros(nx-1,1)];
end

% Filtre axe x
% initGLRx.xhat	= [avionic_filtered.GPS.x(selly(1)); zeros(nx-1,1)];
initGLRx.P      = 10*eye(nx);

% Filtre axe y
% initGLRy.xhat	= [avionic_filtered.GPS.y(selly(1)); zeros(nx-1,1)];
initGLRy.P      = 10*eye(nx);

% Filtre axe z
% initGLRz.xhat	= [z_in.signals.values(1,3); zeros(nx-1,1)];
initGLRz.P      = 10*eye(nx);


% Active GLR parameters
Abar = blkdiag(A,1);
Bbar = [B;0];
Cbar = [C,F];
Qbar = blkdiag(syskfx.Q,0);

paramKFbar = syskfx;
paramKFbar.dt = dt;
paramKFbar.dty = dty;

paramKFbar.A = Abar;
paramKFbar.B = Bbar;
paramKFbar.C = Cbar;
paramKFbar.Q = Qbar;

initxbarx = initx;
initxbarx.xhat = [initx.xhat; zeros(nb,1)];
initxbarx.P = blkdiag(initx.P,0);
initxbary = inity;
initxbary.xhat = [inity.xhat; zeros(nb,1)];
initxbary.P = blkdiag(initx.P,0);
initxbarz = initz;
initxbarz.xhat = [initz.xhat; zeros(nb,1)];
initxbarz.P = blkdiag(initx.P,0);

paramAGLR = paramGLR;
initAGLRx = initGLRx;
initAGLRy = initGLRy;
initAGLRz = initGLRz;

%%
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                   Integrity parameters                                 +
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
nomSPE = 'specification_test_TLS';


if ~exist('paramINT','var')
    % Lecture des spécifications
    [specSIM,specINT,specFLT] = lire_specif(nomSPE,1);
    [paramINT] = def_param_INT(specINT,paramKF);
end

if ~exist('paramINTAGLR','var')
    % Lecture des spécifications
%     [specSIM,specINT,specFLT] = lire_specif(nomSPE,1);
%     [paramINT] = def_param_INT(specINT,paramKF);
    paramKFbar2 = paramKFbar;
    paramKFbar2.C = [1 0 0]; 
    [paramINTAGLR] = def_param_INT(specINT,paramKFbar2);
end
%% Plotty
% figure, 
% subplot(311), 
% plot(t_imu,acc_inert_xyz_imu(:,1)), 
% title('Acceleration, inertial')
% subplot(312), 
% plot(t_imu,acc_inert_xyz_imu(:,2)), 
% subplot(313), 
% plot(t_imu,acc_inert_xyz_imu(:,3))
% 
% figure,
% subplot(311)
% plot(t_gps(sel_gps), xyz_gps(sel_gps,1))
% title('Novatel SBAS, position')
% subplot(312)
% plot(t_gps(sel_gps), xyz_gps(sel_gps,2))
% subplot(313)
% plot(t_gps(sel_gps), xyz_gps(sel_gps,3))

% figure,
% subplot(311), hold on
% plot(t_gps_cut, v_xyz_gps(:,1))
% title('Novatel SBAS vs RTK, velocity')
% subplot(312), hold on
% plot(t_gps_cut, v_xyz_gps(:,2))
% subplot(313), hold on
% plot(t_gps_cut, v_xyz_gps(:,3))
% legend('SBAS', 'RTK')

% figure,
% subplot(311), hold on,
% plot(t_gps_cut, xyz_gps(:,1))
% title('Novatel RTK, position')
% subplot(312), hold on,
% plot(t_gps_cut, xyz_gps(:,2))
% subplot(313), hold on,
% plot(t_gps_cut, xyz_gps(:,3))


% figure,
% subplot(311), title('GPS bias')
% plot(t_imu_cut, interp1(t_gps, xyz_gps(:,1), t_imu_cut,'pchip')-avionic_filtered.GPS.x(selly))
% subplot(312)
% plot(t_imu_cut, interp1(t_gps, xyz_gps(:,2), t_imu_cut,'pchip')-avionic_filtered.GPS.y(selly))
% subplot(313)
% plot(t_imu_cut, interp1(t_gps, xyz_gps(:,3), t_imu_cut,'pchip')+avionic_filtered.GPS.z(selly))

% figure,
% subplot(311)
% plot(t_gps(2:end), diff(xyz_gps(:,1)))
% subplot(312)
% plot(t_gps(2:end), diff(xyz_gps(:,2)))
% subplot(313)
% plot(t_gps(2:end), diff(xyz_gps(:,3)))

figure,
for k = 1:3
subplot(3,1,k)
plot(u_in.time, u_in.signals.values(:,k))
end

figure,
for k = 1:3
subplot(3,1,k)
indvec = [1 3 5];
plot(z_in.time, z_in.signals.values(:,indvec(k)))
end


%%
%  q0 = eul2qua([phi(1),theta(1),psi(1)]);
%  qhat = q0; 
%  eulout = zeros(3,numel(t_imu_cut));  
%  
%  for k = 1:numel(t_imu_cut),
%      omx = [0, gyro_in(k,2:4);
%             gyro_in(k,2:4)', skew(gyro_in(:,k))'];
%      qhat = qhat + 0.5*omx*qhat*dt;
%      qhat = qhat/norm(qhat);
%      
%      eul = qua2eul(qhat);
%      eul(3) = psi(k);
%      qhat = eul2qua(eul);
%      
%      eulout(:,k) = (qua2eul(qhat));
%  end

%% Split scenario


     
     
     
     
     