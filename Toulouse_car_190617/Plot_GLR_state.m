% GLR Plotscript
%%
savename = ['GLR_results_',simcase,'_L',num2str(Lt),'_PFA10m',num2str(pfa_pwr),...
    '_pstdH',strrep(num2str(pos_std(1)),'.','-'),'_pstdV',strrep(num2str(pos_std(3)),'.','-'),...
    '_vstdH',strrep(num2str(vel_std(1)),'.','-'),'_vstdV',strrep(num2str(vel_std(3)),'.','-'),time_str_short];
% save(fullfile('E:\GLR\Toulouse_car_190617\',savename),'glr','yrcfilt2','z_out','u_in', 'paramGLR','paramKF','paramINT');
%%
load ref_trac
zdata = squeeze(z_out.signals.values);
if size(zdata,2) > size(zdata,1), zdata = zdata'; end
xhatdata = squeeze(xhat_out.signals.values);
if size(xhatdata,2) > size(xhatdata,1), xhatdata = xhatdata'; end

xhatbiasdata = squeeze(xhat_H0_out.signals.values);
if size(xhatbiasdata,2) > size(xhatbiasdata,1), xhatbiasdata = xhatbiasdata'; end


rtk_ind = find(abs([0; diff(zdata(:,1))])> 0); % extract unique points from RTK

% rtk_ind = find(abs([0; diff(xyz_gps_rtk(:,1))])> 0); % extract unique points from RTK
% figure, plot(t_imu_cut(rtk_ind), xyz_gps_rtk(rtk_ind,3))
% hold on, plot(t_imu_cut, xyz_gps_rtk(:,3))

switch simcase
    case {'V1','A1'}
        iposx = [1 2 3];
        iposz = [1 2 3];
    case {'A2'}
        iposx = [1 3 5];
        iposz = [1 3 5];
    case 'A2B'
        iposx = [1 4 7];
        iposz = [1 3 5];
    case 'V1B'
        iposx = [1 3 5];
        iposz = [1 2 3];
end


for k = 1:3
    ykf{k} = yrcfilt2(:,k);
end


%% Main GLR result plot
titlevec = {'Willsky GLR','MGLR-RLS','MGLR-LS','Willsky-Single','Jamouli-AGLR','MGLR-Mest'};
filtvec = {'Willsky','MGLR_RLS','MGLR_LS','MGLR-Mest'};
glrlegvec = {'$\epsilon_{\hat{x},Willsky}$','$\epsilon_{\hat{x},MGLR-RLS}$',...
    '$\epsilon_{\hat{x},MGLR-LS}$','$\epsilon_{\hat{x},Willsky-1}$',...
    '$\epsilon_{\hat{x},AGLR}$','$\epsilon_{\hat{x},MGLR-Mest}$'};
axisvec = {'North','East','Down'};
axvec = {'x','y','z'};
if strcmp(simcase,'V1')
    kpvec = [1 2 3];
    kpyvec = kpvec;
else
    kpvec = [1 3 5];
    kpyvec = [1 2 3];
end
kplvec = [1 6 11];
cnt = 0;
for kp = kpvec;
    cnt = cnt+1;
    kpy = kpyvec(cnt);
    kpl = kplvec(cnt);
%     if kp == 1
%         kpl = 1;
%     elseif kp == 2
%         kpl = 6;
%     else
%         kpl = 11;
%     end
%     for kk = [1 3 4 5 6]
      for kk = [1 3]
        xhat{kk} = squeeze(glr(kk).xhat.signals.values); if size(xhat{kk},2) > size(xhat{kk},1); xhat{kk} = xhat{kk}'; end
        xhatbias{kk} = squeeze(glr(kk).xhatbias.signals.values); if size(xhatbias{kk},2) > size(xhatbias{kk},1); xhatbias{kk} = xhatbias{kk}'; end
        error_x = xhat{kk}(rtk_ind,kp) - ykf{kpy}(rtk_ind);
        errorbias_x = xhatbias{kk}(rtk_ind,kp) - ykf{kpy}(rtk_ind);
        
        figvl_state = figure('units','normalized','outerposition',[0 0 1 1]); set(gcf, 'Color', 'w');
        
        s(1) = subplot(211); hold on
        title([titlevec{kk},' - ',axisvec{cnt}],'interpreter','latex')
        xlim([0 max(xhat_out.time)])
        h3 = plot(xhat_out.time(rtk_ind), abs(error_x),'k','Linewidth',2);
        h4 = plot(xhat_out.time(rtk_ind), 3*glr(kk).xhatsig.signals.values(rtk_ind,kp),'b-.','Linewidth',2);
        h5 = plot(xhat_out.time(rtk_ind), glr(kk).PL.signals.values(rtk_ind,kpl),'color',[0 0.75 0],'linewidth',1.5);
        h6 = plot(xhat_out.time(rtk_ind),ones(size(xhat_out.time(rtk_ind)))*paramINT.AL,'r--','linewidth',1.5);
        ylabel('Est. error [m]','interpreter','latex','fontsize',14)

        legend([h3,h4, h5, h6], {glrlegvec{kk},'$3\,\sigma_{\hat{x}}$','$PL$','$AL$'},...
            'interpreter','latex','fontsize',24, 'location','northeastoutside');
        funFigureProperty

        maxy = max([3*max(glr(kk).xhatsig.signals.values(:,kp)),max(abs(error_x))]);
        maxy2 = max(max(glr(kk).PL.signals.values(rtk_ind,kpl)),paramINT.AL);
        
        ylim([0, max(maxy, maxy2)*1.1])
        %     ylim([0 20])
        set(gcf,'color','w')

        s(2) = subplot(212); hold on
        xlim([0 max(xhat_out.time)])
        ylim([0 paramGLR.GLRthr*2])
        plot(glr(kk).lmax.time,glr(kk).lmax.signals.values(:,kpy),'-','Linewidth',1.5)
        plot(glr(kk).lmax.time, ones(size(glr(kk).lmax.time))*paramGLR.GLRthr,'--','Linewidth',2)
        legend({'$l_{GLR}$','$l_{GLR,det}$'},'interpreter','latex','location','northeastoutside')
        ylabel('Test statistic [-]','interpreter','latex')
        xlabel('Time [s]','interpreter','latex','fontsize',14),
        funFigureProperty

        
        p1 = get(s(1),'position');
        p2 = get(s(2),'position');
%         p3 = get(s(3),'position');
        set(s(2),'position',[p2(1:2)+[0 0.03], p1(3), p1(4)])
%         set(s(3),'position',[p3(1:2), p1(3), p1(4)])
        set(s(1),'position',[p1(1:2), p1(3), p1(4)])
        
        linkaxes(s,'x');
        if strcmp(simcase,'V1')
%             export_fig(fullfile('E:\Hem_190503\Thesis\Figures\figures_GLR',['TLSCAR_V1_',axvec{kpy},'_',filtvec{kk},'_',time_str_short]),'-pdf',gcf);
        else
%             export_fig(fullfile('E:\Hem_190503\Thesis\Figures\figures_GLR',['TLSCAR_A2_',axvec{kpy},'_',filtvec{kk},'_',time_str_short]),'-pdf',gcf);
        end
    end
end

%% Plot Input/measurement
for kp = [1 2 3]
    ky = [1 3 5];
    switch simcase
        case {'V1','V1B'}
            figure('units','normalized','outerposition',[0 0 1 1]); set(gcf, 'Color', 'w');
            s1 = subplot(211); hold on
            xlim([0, xhat_out.time(end)])
            try
                plot(u_in(:,1),u_in(:,kp+1),'k-','linewidth',1.5)
            catch
                plot(u_in.time,u_in.signals.values(:,kp),'k-','linewidth',1.5)
            end
            legend({'$u_m$'},'interpreter','latex','location','northeastoutside')
            ylabel('Input [m/s]','interpreter','latex')
            funFigureProperty
            p1 = get(s1,'position');
            s2 = subplot(212); hold on
            xlim([0, xhat_out.time(end)])
            %         plot(y_out.time, y_out.signals.values(:,kp),'k--','linewidth',1.5)
            plot(z_out.time, z_out.signals.values(:,kp),'b','linewidth',1.5)
            legend({'$y_m$'},'interpreter','latex','location','northeastoutside')
            ylabel('Measurement [m]','interpreter','latex')
            funFigureProperty
            
            dtx = 0.0; dty = 0;
            p2 = get(s2,'position');
            set(s2,'position',[p2(1:2)-[dtx, dty], p1(3:4)])
            p1 = get(s1,'position');
            p2 = get(s2,'position');
            set(s1,'position',[p1(1:2)-[dtx, dty], p2(3:4)])
            
        case {'A1','A2','A2B'}
            figure('units','normalized','outerposition',[0 0 1 1]); set(gcf, 'Color', 'w');
            s1 = subplot(311); hold on
            xlim([0, xhat_out.time(end)])
            try
                plot(u_in(:,1),u_in(:,kp+1),'k-','linewidth',1.5)
            catch
                plot(u_in.time,u_in.signals.values(:,kp),'k-','linewidth',1.5)
            end
            legend({'$u_m$'},'interpreter','latex','location','northeastoutside')
            ylabel('Acc. meas. [m/s$^2$]','interpreter','latex')
            axis tight
            funFigureProperty
            p1 = get(s1,'position');
            s2 = subplot(312); hold on
            xlim([0, xhat_out.time(end)])
            
            plot(z_out.time, z_out.signals.values(:,ky(kp)+1),'b','linewidth',1.5)
            legend({'$y_{v,m}$'},'interpreter','latex','location','northeastoutside')
            ylabel('Spd. meas.[m/s]','interpreter','latex')
            
            axis tight
            funFigureProperty
            
            s3 = subplot(313); hold on
            xlim([0, xhat_out.time(end)])
            plot(z_out.time, ykf{kp},'k--','linewidth',1.5)
            plot(z_out.time, z_out.signals.values(:,ky(kp)),'b','linewidth',1.5)
            legend({'$y_{p,true}$','$y_{p,m}$'},'interpreter','latex','location','northeastoutside')
            ylabel('Pos. meas. [m]','interpreter','latex')
            xlabel('Time [s]', 'interpreter','latex')
            axis tight
            funFigureProperty
            
            dtx = 0.0; dty = 0;
            p2 = get(s2,'position');
            p3 = get(s3,'position');
            set(s2,'position',[p2(1:2)-[dtx, dty], p1(3:4)])
            set(s3,'position',[p3(1:2)-[dtx, dty], p1(3:4)])
            p1 = get(s1,'position');
            p2 = get(s2,'position');
            set(s1,'position',[p1(1:2)-[dtx, dty], p2(3:4)])
    end
%     export_fig(fullfile('E:\Hem_190503\Thesis\Figures\figures_GLR',['TLSCAR_',num2str(expart),'_',axvec{kp},'_input',time_str_short]),'-pdf',gcf)
end

%% Plot measurmeent error

for kp = 1:3
    ky = [1 3 5];
    figure('units','normalized','outerposition',[0 0 1 0.7]); set(gcf, 'Color', 'w');
    plot(y_out.time(rtk_ind), z_out.signals.values(rtk_ind,ky(kp)) - ykf{kp}(rtk_ind),'k-','linewidth',1.5)
    legend({'$y_m-y_{true}$'},'interpreter','latex','location','best')
    ylabel('Meas. error [m]','interpreter','latex')
    xlabel('Time [s]','interpreter','latex')
    title(axisvec{kp},'interpreter','latex')
    axis tight
    funFigureProperty
%     export_fig(fullfile('E:\Hem_190503\Thesis\Figures\figures_GLR',['TLSCAR_',num2str(expart),'_',axvec{kp},'_MeasError',time_str_short]),'-pdf',gcf)
end
    
%% Create Google Earth trace
if 1
    gtmat = [ykf{1}, ykf{2}, ykf{3}];
    llh_gt = zeros(size(xhatdata,1),3);
    for k = 1:size(llh_gt,1)
        %     ned2llh(xhatdata(k,iposx)',llh0')
        llh_gt(k,:) = (ned2llh(gtmat(k,[1 2 3])',llh0'))';
    end
    
    llh_xhat = zeros(size(xhatdata,1),3);
    for k = 1:size(llh_xhat,1)
        %     ned2llh(xhatdata(k,iposx)',llh0')
        llh_xhat(k,:) = (ned2llh(xhatdata(k,iposx)',llh0'))';
    end
    llh_xhat_ppl = zeros(size(xhatdata,1),3);
    for k = 1:size(llh_xhat,1)
        %     ned2llh(xhatdata(k,iposx)',llh0')
        llh_xhat_ppl(k,:) = (ned2llh(xhatdata(k,iposx)' + glr(3).PL.signals.values(k,[1 6 11])',llh0'))';
    end
    llh_xhat_mpl = zeros(size(xhatdata,1),3);
    for k = 1:size(llh_xhat_mpl,1)
        %     ned2llh(xhatdata(k,iposx)',llh0')
        llh_xhat_mpl(k,:) = (ned2llh(xhatdata(k,iposx)'- glr(3).PL.signals.values(k,[1 6 11])',llh0'))';
    end
    
    llh_xhatbias = zeros(size(xhatbiasdata,1),3);
    for k = 1:size(llh_xhatbias,1)
        llh_xhatbias(k,:) = (ned2llh(xhatbiasdata(k,iposx)',llh0'))';
    end
    llh_z = zeros(size(zdata,1),3);
    for k = 1:size(llh_z,1)
        llh_z(k,:) = (ned2llh(zdata(k,iposx)',llh0'))';
    end
end
%%
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_xhat',time_str_short]),llh_xhat(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_xhat_ppl',time_str_short]),llh_xhat_ppl(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_xhat_mpl',time_str_short]),llh_xhat_mpl(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_xhatbias',time_str_short]),llh_xhatbias(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_z',time_str_short]),llh_z(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_3_gt',time_str_short]),llh_gt(:,1:2))

% 
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_2_xhat',time_str_short]),llh_xhat(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_2_xhat_ppl',time_str_short]),llh_xhat_ppl(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_2_xhat_mpl',time_str_short]),llh_xhat_mpl(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_2_xhatbias',time_str_short]),llh_xhatbias(:,1:2))
% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617',['Test_2_z',time_str_short]),llh_z(:,1:2))


% pwr_kml(fullfile('E:\GLR\Toulouse_car_190617','GNSS_track_3'),[lat, lon])