% Toulouse car, Reference reconstruction

rtk_ind = find(abs([0; diff(zdata(:,1))])> 0); % extract unique points from measurement log

p_meas_in  = zdata(rtk_ind,[1 3 5]);
v_meas_in  = zdata(rtk_ind,[2 4 6]);
time_in = glr(1).xhat.time(rtk_ind);

% Pre_filter measurements (zero-lag)
v_meas_filt = zeros(size(v_meas_in));
p_meas_filt = zeros(size(p_meas_in));
bv_meas_filt = zeros(size(v_meas_in));

for k = 1:3
    p_meas_filt(:,k) = filtnocaus(p_meas_in(:,k),time_in,10);
    v_meas_filt(:,k) = filtnocaus(v_meas_in(:,k),time_in,0.1);
%     bv_meas_filt(:,k) = filtnocaus(v_meas_in(:,k),time_in,30);
end

p_est = zeros(size(p_meas_in));
p_est(1,:) = p_meas_in(1,:);

tau = 5;
k_alpha = 1-exp(-dt/tau);
tau_b = 30;
k_beta = 1-exp(-dt/tau_b);
k_vec = [k_alpha, k_alpha, k_beta];
for k = 2:numel(time_in)
    dt_int = time_in(k)-time_in(k-1);
    p_pre = p_est(k-1,:)+(v_meas_filt(k-1,:)-bv_meas_filt(k-1,:))*dt_int;
    innov = p_meas_in(k,:)-p_pre;
    kfac = [1 1 1];
    for kk = 1:3
        if abs(innov(kk)) > 1
            kfac(kk) = 1/abs(innov(kk));
        end
    end
    p_est(k,:) = p_pre + kfac.*k_vec.*(p_meas_in(k,:)-p_pre); 
end

yrcfilt1 = zeros(numel(xhat_out.time),3);
for kk = 1:3
    yrcfilt1(:,kk) = interp1(time_in, p_est(:,kk), xhat_out.time);
end

p_int_z = p_meas_in(1,3)+cumsum((v_meas_filt(:,3)-bv_meas_filt(:,3))*dt);