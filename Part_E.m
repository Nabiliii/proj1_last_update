function Main()
% Load data, to be played back. This is just one of the may datasets you may use.
file='DataUsr_p021.mat';   %onex of the datasets we can use (simulated data, noise free, for Project 1).
% As it is "noise-free", you can debug your implementation more easily.
% load the data, now.
load(file); 
% It will load a variable named data (it is a structure)  
% Use the data.
ExploreData(data);
%k = ooi_positions;
%disp(k);
end
% ----------------------------------------
function ExploreData(data)
tic();
plot_envirnment = InitCertainPartOfMyProgram(data); %figure 11 -background 
X_0             = data.pose0;            %platform's initial pose; [x0;y0;heading0] [meters;meters;radians]
n_events        = data.n;                % how many events?
table           = data.table;            % table of events.
event0          = table(:,1);            % first event.
t0              = event0(1);             % initial time (the time of event0).
t0              = 0.0001*double(t0);     % "event" is of integer type, that is why we convert t0 to double ( i.e. to "real") and over to s
vw              = [0;0];                 % The last [speed,heading rate] measurement.
pos_buffer      = zeros(3,n_events,'single');     % A buffer for recording my results (estimated poses).  size =3 * n_events.



%............................
% LiDARs' parameters (position and oriantation in car's frame, FoV, etc)
Lidar1Cfg    = data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation,in platform's coordinate frame.). 
Lidar2Cfg    = data.LidarsCfg.Lidar2;
% info: 'LiDAR's pose in UGV's CF=[Lx;Ly;Alpha], in [m,m,RADIANS]'
% It needs to be considered in your calculations.
%............................


X_kin_mod = zeros(2,n_events);
% Loop: read entries, one by one, for sequential processing.

for i = 1: n_events          
       
            pos_buffer(:,i) = X_0;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
            % get event description.
            event     = table(:,i);                %event #i    -->(:,i) gives me columns                   
            sensorID  = event(3);
            index     = event(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
            
            tNow      = 0.0001*double(event(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
            dt        = tNow-t0;                   % dt since last event ( "dt" is needed for prediction steps).
            t0        = tNow ;                     % remember current time, so we can calculate dt in next iteration.            
            
            % Perform prediction X(t+dt) = f(X,vw,dt) ; vw model's inputs (speed and gyroZ) 
            % apply Kinematic model here, using the last values of inputs ( in vw), 
            % and the last value of the system's state, X_0
            
            %part A
            X_0            = MyKinematicModel(X_0,vw,dt); 
            X_kin_mod(1,i) = X_0(1);
            X_kin_mod(2,i) = X_0(2);
       
             %the UGV current pos:
                theta = X_0(3); % Heading angle in radians
                % Define the length of the arrow
                L = 2;
                % Calculate the x and y components of the arrow
                u = L*cos(theta);
                v = L*sin(theta);
             set(plot_envirnment(9),'xdata',X_0(1),'ydata',X_0(2),'UData',u,'VData',v);
      
            switch sensorID    % measurement is from which sensor?
                
                case 1         %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
                %fprintf('Event (LiDAR), dt=[%.1fms], at t=[%.3f]\n',dt*1000,tNow);   % print some info, for debugging our code.
                
                % both LiDARs are sinchronized.
                scan1 = data.scans(:,index);  %get the col of scan1, each col is a scan (301 scans).
                scan2 = data.scans2(:,index);  

               processLiD_cartiesian_plot(plot_envirnment(5:6),plot_envirnment(10:12),scan1,1,X_0,MyKinematicModel(X_0,vw,dt),Lidar1Cfg, data.Context.Landmarks);
               processLiD_cartiesian_plot(plot_envirnment(7:8),plot_envirnment(13:15),scan2,2,X_0,MyKinematicModel(X_0,vw,dt),Lidar2Cfg, data.Context.Landmarks);
        
               pause(0.005);
               continue;                       %"done, next event!"
                
                %...............................
                case 2                          %  It is speed encoder + gyro  (they are packed together, synchonized readings)
                
                vw   = data.vw(:,index);        % speed and gyroZ, last updated copy. keep the variable "vw" updated to the last reading of those sensors.
        
                continue;
            end

end
   
ShowVerification1(data,X_kin_mod);%global_pos_OOI1,OOIs_center,matchedPoints1, matchedPoints2 );
plot_for_part_E_bias(data,X_kin_mod);

fprintf('The angular error = %.1f degrees. \n ', corrct_angle());
fprintf('The bias = %.3f m. \n\n ', set_bias());

end %end function ExploreData(Data) 

% ---------------------------------------------------------------------------------------------------
function plot_envirnment = InitCertainPartOfMyProgram(data)

    % you may initialize things you need.
    % E.g.: context for some dynamic plots,etc.
        
    % for local CF, in a figure.
    % and for global representation, in other one (you decide how to show
    % your results)
    
    % create some figure for your stuff.
    figure(2); clf();    % global CF.
    
    % Show the map landmarks and, if it is of interest to verify your solution, the
    % walls/infrastructure present there.
    % (All of them are provided in Global CF)
    
    Landmarks=data.Context.Landmarks;
    % plot centres of landmarks. 
    plot(Landmarks(1,:),Landmarks(2,:),'ko','markersize',3)
    % later, during play back session, some LiDAR's pixels will appear close to some of these leandmarks. It means he LiDAR scan is
    % detecting those poles. 
    
    
    % plot interior of walls (they have ~20cm thickness; but the provided info just includes the ideal center of the walls
    % your LiDAR scans will appear at ~ 10cm from some of those lines.    
    % Wall transversal section:  :  wall left side [<--10cm-->|<--10cm-->] the other  wall side. 
    hold on;
    Walls = data.Context.Walls;
    plot(Walls(1,:),Walls(2,:),'color',[0,1,0]*0.7,'linewidth',3);
    legend({'Centers of landmarks','Walls (middle planes) '});
    
    title('Global CF (you should show some results here)');
    xlabel('X (m)'); 
    ylabel('Y (m)');
    p0=data.pose0;
    plot(p0(1),p0(2),'r*','markersize',10);
    %legend({'Landmarks','Walls (middle planes)','initial position'});
    
    
    plot_envirnment = CreateFigureToShowScansInPolar();
    %hh=[];  % array of handles you may want to use in other parts of the
    %program, for updating certain graphic objects (for animations)
end
% ---------------------------------------------------------------------------------
function ShowVerification1(data,X_kin_mod)%,global_pos1,OOIs_center,matchedPoints1, matchedPoints2 )

% plot some provided verification points (of platfom's pose).
% Those are the "ground truth".
% Do not expect your solution path to perfectly match those points, as those are
% the real positions, and yours are approximate ones, based on
% predictions, and using sampled inputs. 
% When using a "noise-free"  dataset, the discrepancy should be just fraction of cm, as the inputs are not
% polluted by noise, and the simulated model is the nominal analog model.
% The errors are mostly due to time discretization and sampled inputs.
% Inputs were sampled ~ @100Hz (10ms) (you can infer that from "dt".
figure(2)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
%legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
hold on;
plot(X_kin_mod(1,:),X_kin_mod(2,:), 'k-'); %plot the data from the kinematic model
hold on;



end

       
    
% -----------------------------------------------------------------------------------------------------------
    
function  processLiD_cartiesian_plot(figur_handles_local,figure_handle_global,scan,id,T,X_kin_mod, LidarCfg,Landmarks) %plot scans and OOI in cartes. LiDaR_CF and global
  
    
        %plot dynamicaly the scans in cartesian in global_CF:
        [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg);
       set(figure_handle_global(1),'xdata',scans_global_pos(:,1), 'ydata',scans_global_pos(:,2));


         
    

end
% ---------------------------------------------------------------------------------
function [x,y] = Ranges_in_cartesian_LiDaR_CF(scan)   %used in two fuctions above, returns pos of dots in cartesan local CF
    mask1  = 16383;  
    ranges = single(bitand(scan,mask1))*0.01; 
    angles = [-75:0.5:75]';
    angles = deg2rad(angles);
    % Convert to Cartesian coordinates using the LiDAR's geometry
    x = ranges .* cos(angles);
    y = ranges .* sin(angles);
end
% ---------------------------------------------------------------------------------


function [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg) %Retrun the all dots in Global_CF
    %the heading of the vehicle:
    theta = T(3);
    % lidar position in GCF (assuming scalar):
    x_origin = X_kin_mod(1); 
    y_origin = X_kin_mod(2); 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
%-----------------------------
    % alpha =  LidarCfg.Alpha for lidar1 and + some corrector for lidar2:
    if id == 1
        alpha = LidarCfg.Alpha;
    end
    if id == 2
        tun = corrct_angle();   % =5 for p020  and -6 for p2021
        alpha = LidarCfg.Alpha + deg2rad(tun);
    end
%-----------------------------

    %the rotation matrix:
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];
    
    %alpha = 3.1416;
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];
    
    % now get the scans (dots) in local CF using detect_ooi function
    [x,y] = Ranges_in_cartesian_LiDaR_CF(scan);  %return scan_pos in cartesian local LiDaR_CF (X_l,y_l)
    p_local = zeros(301,2);
    for i = 1: size(x)
        p_local(i,:) = R_LiDaR * [x(i),y(i)]'+[lx;ly]; 
    end
   
    scans_global_pos = R * p_local' + [x_origin;y_origin ];
    scans_global_pos = scans_global_pos'; %//was wrong dim. global pos give n*1 while shoud be n*2
end



% ---------------------------------------------------------------------------------
function figur_handles = CreateFigureToShowScansInPolar()
    figure(1);
    hold on;
    title('Part E Bias');xlabel('X (m)');  ylabel('Y (m)'); axis([-5,20,-5,20]);grid on;
     movegui(gcf, [640 120]);
   

    

     h1 = 0; % plot(aa,r,'.b');

     h1b = 0; %plot(0,0,'r+');

     h3 = 0;% plot(0,0,'.g'); 

     h3b = 0; %plot(0,0,'r*','markersize',2);
%     subplot(223);  
     h2 = 0; % plot(aa,r,'.b'); 

     h2b = 0 ; % plot(0,0,'r+');
% 
%     subplot(224);  
     h4 = 0; %plot(0,0,'.g'); 

     h4b = 0; %plot(0,0,'r*');
%     
    figure(2);
    movegui(gcf, [70 120]);
    hold on;
    


    h5 = quiver(0,0,'O','LineWidth', 2,'MaxHeadSize', 0.5,'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','k');
    title('OOIs globl CF');  xlabel('X (m)');  ylabel('Y (m)'); axis([-5,21,-5,26]); grid on;
    hold on;  
    h5b = plot(0,0,'m.','MarkerSize',1);     %dots from lidar1             %10
    h5c = plot(0,0,'b-');                   %DA from lidar1                %11
    h5d = plot(0,0,'r*','markersize',3);                   %ooi_center dynamicly lidar1 %12
    h5e = plot(0,0,'k.','MarkerSize',1);  %dots from lidar2              %13
    h5f = plot(0,0,'b-');                %DA from lidar2    %11          %14
    h5g = plot(0,0,'r*','markersize',3);                %ooi_center dynamicly lidar2     %15
     legend({'Landmarks','Walls (middle planes)','initial position','UGV','LiDaR1','LiDaR2'});
    %legend({'scans1','scans2'});
    
    
    


    figur_handles = [h1,h1b,h2,h2b,h3,h3b,h4,h4b,h5,h5b,h5c,h5d,h5e,h5f,h5g]; %retrun hadnles to the figurs
end    
% ---------------------------------------------------------------------------------

function X = MyKinematicModel(X,vw,dt)

       tun = set_bias();

       vw_tun = vw + tun;  %take the gyroscope values and correct it, been used in X(3)

     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; 
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw_tun(2) * dt;
end   
% ---------------------------------------------------------------------------------

function plot_for_part_E_bias(data,X_kin_mod)
    figure(1)
    hold on;
    p=data.verify.poseL;
    plot(p(1,:),p(2,:),'r.','markersize',1);
    %legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
    hold on;
    plot(X_kin_mod(1,:),X_kin_mod(2,:), 'k-'); %plot the data from the kinematic model
    legend({'ground Truth','kinematic model'});
   
 end

% ---------------------------------------------------------------------------------
% ---------------------------------------------------------------------------------
% ---------------------------------------------------------------------------------


% Adjust bias and angular error here:

function bias = set_bias()               %For Part E, set to 0 for dataset 002 ,, -0.007 for p020 ,, 0.007 for p2021
bias = 0;%0.007;
end

function angular_error = corrct_angle()   
 angular_error = 0;  % =5 for p020  and -6 for p021
end

% ---------------------------------------------------------------------------------
% ---------------------------------------------------------------------------------
% ---------------------------------------------------------------------------------

