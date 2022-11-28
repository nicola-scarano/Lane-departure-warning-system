%% Tutorial 05 Motor Vehicle Mechanics
% SINGLE TRACK MODEL ========== FULL Version ============
% close all; 
clear; clc;
close all;

Camera = struct('ImageSize',[480 640],'PrincipalPoint',[320 240],...
            'FocalLength',[320 320],'Position',[1.8750 0 1.2000],...
            'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
            'LaneDetectionRanges',[8 17],'DetectionRanges',[8 50],...
            'MeasurementNoise',diag([6,1,1]));

% Parameters used in the lane detection logic
left_lane_arr = zeros(1,15);
right_lane_arr = zeros(1,15);
distance_left_lane = zeros(1,15);
distance_right_lane = zeros(1,15);
departure_history = zeros(1,15);
departure = 0;  % -1: left departure;  0: NO departure;  +1: right departure
frame_counter = 1;

UIControl_FontSize_bak = get(0, 'DefaultUIControlFontSize');
% Select simulation
set(0, 'DefaultUIControlFontSize', 18);
caso = menu('Choose CoG position','a = L/2-10cm','a = L/2','a = L/2+10cm','Drift: a=b; Ca=Ca_nom/100');       
set(0, 'DefaultUIControlFontSize', UIControl_FontSize_bak);
if isempty(caso)
    caso=1;
end
% 4 cases depending on CoG position: 
% (1) a = L/2-10cm 
% (2) a = L/2
% (3) a = L/2+10cm 
% (4) a=b; Cr=Cr_nom/100
%% --------- Vehicle and tyre DATA ----------
m=1997.6;       %[kg] mass
L = 2.85;       % [m] wheelbase
a_vet =[L/2-0.1 L/2 L/2+0.1 L/2];  %[m] front wheelbase
a = a_vet(caso);  
b = L-a ;       %[m] rear wheelbase        
Tf=1.54;        %[m] front track
Tr=1.54;        %[m] rear track
Jz=3728;        %[kg m^2] mass moment of inertia 
g = 9.81;       %[m/s^2]
tau_s = 15;     % steering ratio

%----- static loads
FzF = m*g*b/L; 
FzR = m*g*a/L;
Perc_F = FzF /(m*g)*100;
Perc_R = 100-Perc_F;
disp(['static load distribution (%F - %R): ',num2str(round(Perc_F)),'-',num2str(round(Perc_R))])
disp(['Static load front: ',num2str(round(FzF)),'-----','Static load rear: ', num2str(round(FzR))])
%------ Cornering stiffness
eval(['load Dati_txt' filesep 'CornStiff_Vs_Fz'])
% interpolate wheel cornering stiffness versus vertical load
CF_w = interp1(Fz_vet,C_alpha_vet,FzF/2);
CR_w = interp1(Fz_vet,C_alpha_vet,FzR/2);
C_med_w = interp1(Fz_vet,C_alpha_vet,m*g/4);

% axle stiffness
CF = 2*CF_w;    % front
if caso==4
    CR = 2*CR_w/100; % /100 for drift simualtion
else
    CR = 2*CR_w;    % rear
end
% % low friction
% mi_low = 0.37;
% CF = CF*mi_low;
% CR = CR*mi_low;

disp('axle cornering stiffness')
disp(['CF = ',num2str(CF),' N/rad'])
disp(['CR = ',num2str(CR),' N/rad'])
disp(' ')
% check under/over steering
if CR*b-CF*a>0
    disp('============ understeering ============')
elseif CF*a-CR*b==0
    disp('============ neutral vehicle ============')
else
    disp('============ over steering ============')
    V_cr = sqrt(CF*CR*L^2/(m*(a*CF-b*CR)))*3.6;
    disp(['critical speed: ',num2str(round(V_cr*10)/10),' km/h'])
end
% understeering and slip angle gradients
mf = m*b/L; mr = m-mf;
K_us_an = (mf/CF-mr/CR);
K_beta_an = -mr/CR;
% tangent speed (beta=0)
V_beta0 = sqrt(b*L*CR/a/m)*3.6;
disp(' ')
disp(['understeering gradient: K_US = ',num2str(K_us_an),' rad/(m/s^2)'])
disp(['slip agle gradient: K_beta = ',num2str(K_beta_an),' rad/(m/s^2)'])
disp(['tangent speed: V_beta = ',num2str(V_beta0),' km/h'])

%% run Simscape model   
% default parameters
t_end_sim = 20;
dvol_max = 0;

% eval(['load Dati_txt' filesep 'sweep.txt'])

set(0, 'DefaultUIControlFontSize', 18);
sel_man = menu('select manoeuvre','Left Step steer on straight road','Right Step steer on a straight road','Double Lane change', 'Wide radius curvature (Read implementation setup)');
set(0, 'DefaultUIControlFontSize', UIControl_FontSize_bak);
if isempty(sel_man)
    sel_man=1;
end
% Initial position of the vehicle in the curvature setting
x_0 = 0;
y_0 = 0;
switch sel_man
    case 1
        %  (Left Step steer)
        t_end_sim = 10*1;        % [s]
        Vd = 100;                % [km/h]
        dvol_max = input('max steering angle (deg) [default = 5 deg]:');
        
    case 2
         %  (Right Step steer)
        t_end_sim = 10*1;        % [s]
        Vd = 100;                % [km/h]
        dvol_max = input('max steering angle (deg) [default = 5 deg]:');
        
    case 3
        %  (Lane Change) ...
        t_end_sim=20;
        Vd = 100;                   % [km/h]
        dvol_max = input('max steering angle (deg) [default = 5 deg]:');

    case 4 
        % (Wide radius curvature)
        x_0 = -62.1916;
        y_0 = -134.804;
        %   (Left Step steer)
        t_end_sim = 10*1;        % [s]
        Vd = 100;                % [km/h]
        dvol_max = input('max steering angle (deg) [default = 10 deg]:');

end
if isempty(dvol_max)==1
    if sel_man == 4
        dvol_max=10; % max steering angle
    else
        dvol_max=5;
    end
end

%
V = input(['choose speed for Simulink manoeuvre (default = ' num2str(Vd) ' km/h): ']);
if isempty(V)
    V=Vd;       % km/h
end
V=V/3.6; 
Vv=V;

% matrix initialization for Simulink linear model 
% state space matricesi: A,B,C,D
    A_sim=[(-CF-CR)/(m*Vv),(-CF*a+CR*b-m*Vv^2)/(m*Vv^2);
        (-CF*a+CR*b)/Jz,(-CF*a^2-CR*b^2)/(Jz*Vv)];
    B_sim=[CF/(m*Vv) CR/(m*Vv);
        (CF*a/Jz) -(CR*b/Jz)];
    C_sim = [1,0
        0,1
        (-CR-CF)/(m*Vv^2),(-CF*a+CR*b)/(m*Vv^3)
        -1, -a/Vv
        -1, b/Vv
        (-CR-CF)/(m),(-CF*a+CR*b)/(m*Vv)];
    D_sim = [0 0;
        0 0;
        CF/(m*Vv^2) CR/(m*Vv^2)
        1 0
        0 1
        CF/m CR/m];
%

% Run simualtion
dt_sim = 1e-3;          % time step
ay_max = 10;            % [m/s^2] limit acceleration: stop simulation
beta_max = 80;          % [deg] limit slip angle: stop simulation
%return

%% run simscape model 
out_SLX = sim('s_track_model');    % run Simulink model
%out_SSC = sim('s_track_VDB');    % run Simulink model

%return

%% POST PROCESSING
%--------- Plot Steering angle
% if exist('fig_6')==0
%     fig_6 = figure('Name','Steering Angle','NumberTitle','off','PaperType','A4');
% else
%     figure(fig_6)
% end
% figure('Name','steering angle')
% hold all; grid on
% plot(out_SLX.delta_steer,'LineWidth',2)
% xlabel('time [s]'),
% hold on; 
% plot(L*out_SLX.ro*180/pi*tau_s,'--k'); 
% plot(out_SSC.delta_steer,'o')
% legend('\delta','\delta_0','\delta_{SSC}','Fontsize',18,'location','best')
% title('Steering Angle \delta_s')

%--------- Plot beta and psi_dot
% if exist('fig_2')
%     fig_2 = figure('Name','beta, r','NumberTitle','off','PaperType','A4');
% else
%     figure(fig_2)
% end
% figure('Name','States')
% hold all; grid on
% subplot(2,1,1)
% plot(out_SLX.beta,'LineWidth',2)
% xlabel('time [s]')
% hold on
% plot(b*out_SLX.ro*180/pi,'--k'); 
% plot(out_SSC.beta,'o')
% legend('\beta','\beta_0','\beta_{SSC}','Fontsize',16,'location','best')
% title('slip angle \beta [deg]','Fontsize',16)
% grid on
% subplot(2,1,2)
% plot(out_SLX.r,'LineWidth',2)
% hold on
% plot(out_SLX.delta_steer/180*pi,'LineWidth',2),xlabel('time [s]'),
% plot(out_SSC.r,'o')
% title('r [deg/s]','Fontsize',16), xlabel('time [s]')
% ylabel('')
% legend('r [deg/s]','\delta [rad]','r_{SSC}','Fontsize',16,'location','best')
% %ylim([-150 150])
% grid on
% 
% %--------- Plot ay vs t
% % if exist('fig_7')==0
% %     fig_7 = figure('Name','Lateral Acceleration a_y','NumberTitle','off','PaperType','A4');
% % else
% %     figure(fig_7)
% % end
% figure('Name','a_y(t)')
% hold all; grid on
% plot(out_SLX.ay,'LineWidth',2)
% plot(out_SSC.ay,'LineWidth',2)
% title('Lateral Acceleration a_y [m/s^2]','Fontsize',16)
% xlabel('time [s]')
% ylabel('')
% legend('a_y [m/s^2]','a_y SSC [rad]','Fontsize',16,'location','best')
% % ---- plot beta beta_dot
% figure('Name','\beta, \beta_dot')
% hold all; grid on
% plot(out_SLX.beta.Data,out_SLX.beta_dot(:,2),'LineWidth',2) %title('Lateral Acceleration a_y [m/s^2]'),
% xlabel('\beta [deg]','Fontsize',18)
% ylabel('\beta_{dot} [rad/s]','Fontsize',18)
% 
% if sel_man == 2  % plot vs ay only for ramp steer 
% %--------- Plot beta vs ay
% figure('Name','beta vs ay')
% % plot(a_y(:,2),Beta(:,2))
% scatter(out_SLX.a_y(:,2),out_SLX.Beta(:,2),[],out_SLX.a_y(:,1)); colorbar
% %%% scatter(out_SSC.a_y(:,2),out_SSC.Beta(:,2),[],a_y(:,1)); colorbar
% xlabel('a_y [m/s^2]')
% ylabel('\beta [deg]'); grid on
% text(11,2.2,['time[s]'])
% set(gca,'FontName','Times New Roman','FontSize',16)
% %--------- Plot delta vs ay
% figure('Name','delta vs ay')
% % plot(a_y(:,2),Beta(:,2))
% scatter(out_SLX.a_y(:,2),out_SLX.delta_rad(:,2)*180/pi,[],out_SLX.a_y(:,1)); colorbar
% xlabel('a_y [m/s^2]'); ylabel('\delta_{vol} [deg]'); grid on
% text(11,22,['time[s]'])
% set(gca,'FontName','Times New Roman','FontSize',16)
% %--------- Plot delta-delta0 vs ay
% figure('Name','delta-delta_0 vs ay')
% delta0 =L*out_SLX.ro.Data;
% % plot(a_y(:,2),Beta(:,2))
% plot(out_SLX.a_y(:,2),(out_SLX.delta_rad(:,2)-delta0*tau_s)*180/pi,'linewidth',2); 
% hold on
% %plot(out_SSC.ay.Data,(out_SSC.delta_rad(:,2)-delta0*tau_s)*180/pi,'linewidth',2);   % delta0 needs to be re-defined!
% xlabel('a_y [m/s^2]'); ylabel('\delta_{vol}-\delta_0 [deg]'); grid on
% set(gca,'FontName','Times New Roman','FontSize',16)
% end
% 
% %--------- Plot alpha_F e alpha_R 
% figure('Name','alphaF e R'); hold all
% plot(out_SLX.alfaF*180/pi,'LineWidth',2); plot(out_SLX.alfaR*180/pi,'LineWidth',2); 
% xlabel('time [s]'); ylabel('\alpha [deg]'); grid on; 
% legend('\alpha_F','\alpha_R','Fontsize',16,'location','best')
% set(gca,'FontName','Times New Roman','FontSize',14)
% legend({},'FontSize',16)
% 
% %--------- Plot curvature
% figure('Name','rho'); hold all
% plot(out_SLX.ro,'LineWidth',2); 
% xlabel('time [s]'); ylabel('\rho [1/m]'); grid on; 
% set(gca,'FontName','Times New Roman','FontSize',14)
% %--------- Plot trajectory
% spost_x=out_SLX.Var_trajectory(:,1);
% spost_y=out_SLX.Var_trajectory(:,2);
% % if exist('fig_3')==0
% %     fig_3 = figure('Name','trajectory','NumberTitle','off','PaperType','A4');
% % else
% %     figure(fig_3)
% % end
% figure
% hold all; grid on
% % plot(spost_x,spost_y,'LineWidth',2)
% scatter(spost_x,spost_y,[],out_SLX.a_y(:,1))
% title('trajectory'),axis equal,xlabel('X [m]'),ylabel('Y[m]');colorbar
% text(49,12,['time[s]'])
% 
% pause(3)
% 
% %% Vehicle Trajectory
% F_Size=13;
% % if exist('fig_13')==0
% %     fig_13 = figure('Name','Vehicle CG location','NumberTitle','off','PaperType','A4');
% % else
% %     figure(fig_13);
% % end
% figure('Name','Vehicle CG location','NumberTitle','off','PaperType','A4');
% hold all; grid on
% plot(out_SLX.Var_trajectory(:,1),out_SLX.Var_trajectory(:,2),'Linewidth',2);
% title('CG Trajectory'); axis equal
% set(gca,'FontName','Times New Roman','FontSize',F_Size)
% xlabel('X [m]'); ylabel('Y [m]')
% X_G = out_SLX.Var_trajectory(:,1);
% Y_G = out_SLX.Var_trajectory(:,2);
% 
% % figure
% %hold on
% axis equal
% dt_frame = 0.1; % [s] vehicle frame refresh (time) interval 
% decim_frame = dt_frame/dt_sim;
% 
% % cycle to show vehicle motion
% for cont1=1:decim_frame:length(out_SLX.Psi)
%     X = X_G(cont1)+[-b a a -b]';
%     Y = Y_G(cont1)+[-Tf/2 -Tf/2 Tf/2 Tf/2]';
%     vert = [X,Y];
%     fac = [1 2 3 4];
%     hVeicolo = patch('Faces',fac,'Vertices',vert,'FaceColor','red','FaceAlpha',.5);
%     direction = [0 0 1];
%     xlim([X(1)-5 X(2)+3])
%     ylim([Y(1)-5 Y(3)+3])
%     
%     x0 = X_G(cont1);
%     y0 = Y_G(cont1);
%     z0 = 0;
%     ORIGIN = [x0,y0,z0];
%     rotate(hVeicolo,direction,out_SLX.Psi(cont1,2),ORIGIN);
%     pause(0.1)
% end
% 
% plot(out_SLX.Var_trajectory(:,1),out_SLX.Var_trajectory(:,2),'Linewidth',2);
% axis auto
% 
% %% TF estimation 
% if sel_man == 3
%     fs=1/dt_sim; % Hz
%     t_wind = 5; % s
%     nfft=t_wind*fs;	         	% number of points (fft 512)
%     nolap=0.9*nfft;				% overlapping percentage (Hanning window) 90%
%     [T_ay_d,freq1] = tfestimate(out_SLX.delta_steer.Data,out_SLX.ay.Data,nfft,nolap,[],fs);
%     figure
%     plot(freq1,abs(T_ay_d))
%     grid on
%     xlim([0 5])
%     xlabel('frequency [Hz]')
%     ylabel('a_y/\delta [g/deg]')
% end
