
function Main()
% Load data, to be played back. 
file='DataUsr_002.mat';   
% load the data, now.
load(file); 
% It will load a variable named data (it is a structure)  
% Use the data.
ExploreData(data);
end
% ----------------------------------------
function ExploreData(data)
%tic();
plot_envirnment = InitCertainPartOfMyProgram(data); %figure 11 -background 
X_0             = data.pose0;            %platform's initial pose; [x0;y0;heading0] [meters;meters;radians]
n_events        = data.n;                % how many events?
table           = data.table;            % table of events.
event0          = table(:,1);            % first event.
t0              = event0(1);             % initial time (the time of event0).
t0              = 0.0001*double(t0);     % "event" is of integer type, that is why we convert t0 to double ( i.e. to "real") and over to s
vw              = [0;0];                 % The last [speed,heading rate] measurement.
pos_buffer      = zeros(3,n_events,'single');     % A buffer for recording my results (estimated poses).  size =3 * n_events.

global MyFlags;  %for the buttons paus/start and end etc.
MyFlags=[0,0,0];
% % Create a push button that will pause and start the loop
btn_end=uicontrol('Style','pushbutton','String','END','Position',[5 45 50 20], 'Callback',@MyCallbackBye);
btn=uicontrol('Style','pushbutton','String','Pause/Continue','Position',[5 20 50 20], 'Callback',@MyCallbackPauseContinue);

%............................
% LiDARs' parameters (position and oriantation in car's frame, FoV, etc)
Lidar1Cfg    = data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation,in platform's coordinate frame.). 
Lidar2Cfg    = data.LidarsCfg.Lidar2;
% info: 'LiDAR's pose in UGV's CF=[Lx;Ly;Alpha], in [m,m,RADIANS]'
%............................

%initial som variables:
global_pos_OOI1= zeros(2000,2); 
global_pos_OOI2= zeros(2000,2);


%_______part F

%__________

X_kin_mod = zeros(2,n_events);

MyFlags(1) = 1;
i=1;
%for i = 1: n_events % can use for loop but then buttons do not work.
while (i <= n_events) && (MyFlags(1))
    pause(0.005);
    if MyFlags(2), pause(0.3) ;  continue; end
          
       
            pos_buffer(:,i) = X_0;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
            % get event description.
            event     = table(:,i);                %event #i    -->(:,i) gives me columns                   
            sensorID  = event(3);
            index     = event(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
            
            tNow      = 0.0001*double(event(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
            dt        = tNow-t0;                   % dt since last event ( "dt" is needed for prediction steps).
            t0        = tNow ;                     % remember current time, so we can calculate dt in next iteration.            
            
            
            %part A
            % apply Kinematic model here, using the last values of inputs ( in vw), and the last value of the system's state, X_0

            X_0             = MyKinematicModel(X_0,vw,dt); 
            X_kin_mod(1,i)  = X_0(1);
            X_kin_mod(2,i)  = X_0(2);
            theta           = X_0(3);
      
            %the UGV current pos, part_C:
                % length of the arrow
                L = 2;
                % Calculate the x and y components of the arrow
                u = L*cos(theta);
                v = L*sin(theta);
                %plot the ugv dynamicly with a arrow sticking out:
             set(plot_envirnment(9),'xdata',X_0(1),'ydata',X_0(2),'UData',u,'VData',v);

             %update i for the while loop:
             i=i+1;

             % measurement is from which sensor?
             switch sensorID   
                
                case 1         %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
                %fprintf('Event (LiDAR), dt=[%.1fms], at t=[%.3f]\n',dt*1000,tNow);   % print some info, for debugging our code.
                
                % both LiDARs are sinchronized.
                scan1 = data.scans(:,index);  %get the col of scan1, each col is a scan (301 scans).
                scan2 = data.scans2(:,index);  
                
                %plot data in polar LiDaR_CF
                processLiDAR_plot_ploar(plot_envirnment(1:2), scan1);
                processLiDAR_plot_ploar(plot_envirnment(3:4), scan2);
        
        
                %find the OOIs from each scan
                %ooi_positions1 = [ooi_positions1; Detect_OOIs(scan1)];
                %ooi_positions2= [ooi_positions2; Detect_OOIs(scan2)];
                
                %partC Global pos of the OOIs
                global_pos_OOI1 = [global_pos_OOI1; local_to_globalk(scan1, X_0,MyKinematicModel(X_0,vw,dt),Lidar1Cfg)];
                global_pos_OOI2 = [global_pos_OOI2; local_to_globalk(scan2, X_0,MyKinematicModel(X_0,vw,dt),Lidar2Cfg)];
                    
                %plot scans in cartesian LiDar_CF  and cartesian global dynamicly
                processLiD_cartiesian_plot(plot_envirnment(5:6),plot_envirnment(10:16),scan1,1,X_0,MyKinematicModel(X_0,vw,dt),Lidar1Cfg, data.Context.Landmarks,detected_OOIs_center(global_pos_OOI1));
                processLiD_cartiesian_plot(plot_envirnment(7:8),plot_envirnment(17:23),scan2,2,X_0,MyKinematicModel(X_0,vw,dt),Lidar2Cfg, data.Context.Landmarks,detected_OOIs_center(global_pos_OOI1));
    
                %part F   
                % part F here . .. . .

               pause(0.005);
               continue;                       %"done, next event!"
                
                %...............................
                case 2                          %  It is speed encoder + gyro  (they are packed together, synchonized readings)
                
                vw   = data.vw(:,index);        % speed and gyroZ, last updated copy. keep the variable "vw" updated to the last reading of those sensors.
                %fprintf('Event (DR),dt=[%.1f ms]; v=[%.2f]m/s,w=[%.2f]deg/sec\n',dt*1000,vw.*[1;180/pi]);
                %continue;  %"next!" (go to process next even, inmmediately)
                %fprintf('unknown sensor, type[%d], at t=[%d]\n',sensorID, t);         
                % because I do not know what to do with this measurement, I ignore it.
                continue;
             end 
end

OOIs_center = detected_OOIs_center(global_pos_OOI1);

[matchedPoints1, matchedPoints2] = Data_assosiation(data.Context.Landmarks,OOIs_center);


%plot( data.verify.poseL(1,:), data.verify.poseL(2,:),'m+');
%part A
disp('Showing ground truth (your estimated trajectory should be close).)');
ShowVerification1(data,X_kin_mod,global_pos_OOI1,OOIs_center,matchedPoints1, matchedPoints2 );

%part E
plot_for_part_E_bias(data,X_kin_mod);

%part F
%disp(pos_buffer);             
%disp(pose_buffer(:, 2));
%plot_for_part_F(pose_buffer);



delete(btn);
delete(btn_end);

% elapsed_time = toc()*1000; % stop the timer and store elapsed time
% display(elapsed_time); %in sec
end  

% ---------------------------------------------------------------------------------------------------
function plot_envirnment = InitCertainPartOfMyProgram(data)

    % create some figure for your stuff.
    figure(4); clf();    % global CF.
    
    Landmarks=data.Context.Landmarks;
    % plot centres of landmarks. 
    plot(Landmarks(1,:),Landmarks(2,:),'ko','markersize',3)

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
    
    
    plot_envirnment = CreateFigureToShowScansInPolar();
end
% ---------------------------------------------------------------------------------
function ShowVerification1(data,X_kin_mod,global_pos1,OOIs_center,matchedPoints1, matchedPoints2 )
figure(4)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
%legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
hold on;
plot(X_kin_mod(1,:),X_kin_mod(2,:), 'k-'); %plot the data from the kinematic model
hold on;
%plot (global_pos1(:,1),global_pos1(:,2),'b+');
plot (OOIs_center(:,1),OOIs_center(:,2),'m+');
hold on;
%hold on;
for i = 1:size(matchedPoints1,1)
    plot([matchedPoints1(i,1), matchedPoints2(i,1)], [matchedPoints1(i,2), matchedPoints2(i,2)], 'b');
end

end
% ---------------------------------------------------------------------------------
function  processLiDAR_plot_ploar(figur_handles,scan)   %plot dots and OOIs in polar
tic(); % start the timer
mask1        = 16383;                           % 0011111111111111; 
mask2        = 49152;                           % 1100000000000000; 
ranges       = single(bitand(scan,mask1))*0.01; %vector of ranges in m
intensities  = bitand(scan,mask2);              % vector of bright points if > 0 so it is bright
color_index  = find(intensities>0);             % a vector of indexes for bright points
angles       = [-75:0.5:75]'; 
% update some graphic object, for animation.
set(figur_handles(1),'ydata',ranges);
% refresh those "colored" ones, in the figure.
set(figur_handles(2),'xdata',angles(color_index),'ydata',ranges(color_index));
%time:
elapsed_time = toc()*1000; % stop the timer and store elapsed time
%display(elapsed_time); %in sec
end
           
% ---------------------------------------------------------------------------------
function  processLiD_cartiesian_plot(figur_handles_local,figure_handle_global,scan,id,T,X_kin_mod, LidarCfg,Landmarks, OOIs_center) %plot scans and OOI in cartes. LiDaR_CF and global
  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
   %plot dynamicaly the ranges in cartesian in LiDaR_CF:
   [x,y] = Ranges_in_cartesian_LiDaR_CF(scan);
   set(figur_handles_local(1),'xdata',x, 'ydata',y);

  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
   tic();
    %Get OOIS and plot dynamicly in cartesian LiDaR_CF:
    [ooi_positions] = Detect_OOIs(scan);
    x1=[];
    y1=[];
    for i = 1:length(ooi_positions)
        for j = 1:length(ooi_positions(i))
            x1 = ooi_positions(j, 1);
            y1 = ooi_positions(j, 2); 
            
        end
    end
    set(figur_handles_local(2),'xdata',x1,'ydata',y1);

 elapsed_time_part_B = toc()*1000; % stop the timer and store elapsed time
% fprintf('Elapsed_time_part_B = %.3f ms. \n ', elapsed_time_part_B);

  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
   %plot dynamicaly the ranges in cartesian global_CF: (dots)
   [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg);
   set(figure_handle_global(1),'xdata',scans_global_pos(:,1), 'ydata',scans_global_pos(:,2));

  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
  %plot the DA dynamicly on GCF   part_D
  [matchedPoints1, matchedPoints2] = Data_assosiation(Landmarks,OOIs_center);  %matc1=Landmarks, match2=ooi_center

  for i = 1:size(matchedPoints1,1)-1
      if i> 3
        set(figure_handle_global(2), 'xdata',[matchedPoints1(i,1), matchedPoints2(i,1)],'ydata', [matchedPoints1(i,2), matchedPoints2(i,2)]);
        set(figure_handle_global(3), 'xdata',[matchedPoints1(i+1,1), matchedPoints2(i+1,1)],'ydata', [matchedPoints1(i+1,2), matchedPoints2(i+1,2)]);
        set(figure_handle_global(4), 'xdata',[matchedPoints1(i-3,1), matchedPoints2(i-3,1)],'ydata', [matchedPoints1(i-3,2), matchedPoints2(i-3,2)]);
        set(figure_handle_global(5), 'xdata',[matchedPoints1(i-1,1), matchedPoints2(i-1,1)],'ydata', [matchedPoints1(i-1,2), matchedPoints2(i-1,2)]);
        set(figure_handle_global(6), 'xdata',[matchedPoints1(i-2,1), matchedPoints2(i-2,1)],'ydata', [matchedPoints1(i-2,2), matchedPoints2(i-2,2)]);
      else
        set(figure_handle_global(2), 'xdata',[matchedPoints1(i,1), matchedPoints2(i,1)],'ydata', [matchedPoints1(i,2), matchedPoints2(i,2)]);
        set(figure_handle_global(3), 'xdata',[matchedPoints1(i+1,1), matchedPoints2(i+1,1)],'ydata', [matchedPoints1(i+1,2), matchedPoints2(i+1,2)]);
 
      end
  end
               
  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
         %plot the OOI_center dynamicly on GCF
         OOIs_center_x =  OOIs_center(:,1);
         OOIs_center_y =  OOIs_center(:,2);
         for i = 1:length(OOIs_center_x)
              set(figure_handle_global(7),'xdata',OOIs_center_x(1:i), 'ydata',OOIs_center_y(1:i));
         end
  %-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

end

% -----------------------------------------------------------------------------------------------------------
function [ooi_positions] = Detect_OOIs(scan) %OOIs in cartesian LiDaR-CF
    % Constants
    mask2 = 49152; % 1100000000000000
    intensities = bitand(scan, mask2);
    max_diameter = 0.20; % m
    max_distance = 0.20; %m
    
    % Get the ranges in cartesian LiDaR_CF:
    [x, y] = Ranges_in_cartesian_LiDaR_CF(scan); % func working with ranges and angles
    points = [x, y];
    
    % Initialize distance vector
    distances = zeros(size(points, 1)-1, 1); % 300x1 vector dist. between a point and all the others, in cartesian
    
    % Compute distances between consecutive points
    for i = 1:size(points, 1)-1 
        distances(i) = norm(points(i+1,:) - points(i,:));
    end
    
   
    index_array = [];
    prev_index = 0; % Initialize previous index
    
    % Loop through distances and find the indexes for where we have cut
    for i = 1:length(distances)
        if distances(i) > max_distance
            % Add previous indexes and current index to index_array
            index_array = [index_array, (prev_index+1):i];
        end
        % Update previous index
        prev_index = i;
    end
    
    % NOW find the diameter for each sequence of dots and find OOIs given criteria:
    prev = 1;
    ooi_x = [];
    ooi_y = [];
    for i = 1:length(index_array)
        j = index_array(i);
        diameter = norm(points(j, :) - points(prev+1, :)); % diameter
        if diameter < max_diameter
            for k = prev:j
                if intensities(k) > 0                 %check if color>0
                    ooi_x = [ooi_x; mean(x(prev:j))];
                    ooi_y = [ooi_y; mean(y(prev:j))];
                    break; % break out of the loop when a valid intensity is found
                end
            end
        end
        prev = j+1;
    end
    ooi_positions = [ooi_x, ooi_y];
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
function [OOI_global_pos] = local_to_globalk(scan, X_0,X_kin_mod,LidarCfg) 
    %the heading of the vehicle:
    theta = X_0(3);
    % lidar position in GCF (assuming scalar):
    x_origin = X_kin_mod(1); 
    y_origin = X_kin_mod(2); 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
    alpha = LidarCfg.Alpha;
    %the rotation matrix LiDar to UVG:
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];
    %the rotation matrix UVG to Global:
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];
    % now get the OOI in local LiDaR_CF using detect_ooi function
    [ooi_positions] = Detect_OOIs(scan);  %return OOI_pos in cartesian local LiDaR_CF 
    % loop over detected OOIs and find the these Points in UVG_CF
    OOI_p_local = zeros(size(ooi_positions, 1),2);
    for i = 1:size(ooi_positions, 1)
        OOI_p_local(i,:) = R_LiDaR * [ooi_positions(i, 1),ooi_positions(i, 2)]' +[lx;ly]; % x_v pos of ooi in vehicle CF
    end
     % Find the global position of the OOI
     OOI_global_pos = R * OOI_p_local' + [x_origin;y_origin ];
     OOI_global_pos = OOI_global_pos'; %//was wrong dim. global pos give n*1 while shoud be n*2
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
        tun = corrct_angle(); % -6;    % =5 for p020  and -6 for p2021
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
%Part D
function [OOIs_center] = detected_OOIs_center(OOI_global_pos) %in GCF
    
     %----First I need to rearrange the OOI_global_pos such that the closest points are in sequence:
    % Compute pairwise distances between points
    distances = pdist2(OOI_global_pos, OOI_global_pos);
    % Sort distances and get corresponding indices
    [sortedDistances, sortedIndices] = sortrows(distances);
    % Rearrange points according to sorted indices
    OOI_global_pos_arranged = OOI_global_pos(sortedIndices, :);

    %---find the oois that represent the same landmark by taking the distance
    %between them.
    distances1 = zeros(size(OOI_global_pos_arranged, 1)-1, 1); 
    for i = 1:size(OOI_global_pos_arranged, 1)-1 
        distances1(i) = norm(OOI_global_pos_arranged(i+1,:) - OOI_global_pos_arranged(i,:));
    end
 
    index_array = [];
    prev_index = 0; % Initialize previous index
    %--- Loop through distances and find the indexes for where we have cut
    for i = 1:length(distances1)
        if distances1(i) > 1.5
            % Add previous indexes and current index to index_array
            index_array = [index_array, (prev_index+1):i];
        end
        % Update previous index
        prev_index = i;
    end
    prev = 1;
    OOIs_center_x=[];
    OOIs_center_y=[];
    %--Update the OOIs_center coordinates:
    for i = 1:length(index_array)
        j = index_array(i);
        OOIs_center_x = [OOIs_center_x; mean(OOI_global_pos_arranged(prev:j,1))];
        OOIs_center_y = [OOIs_center_y; mean(OOI_global_pos_arranged(prev:j,2))];
        prev = j + 1;
    end
   OOIs_center = [OOIs_center_x,OOIs_center_y];
end  


% ---------------------------------------------------------------------------------
function [matchedPoints1, matchedPoints2] = Data_assosiation(Landmarks,OOIs_center)
%Here I will run through all OOIs_center and data.Context.Landmarks to
%assosiate landmarkes with their corresponding detected OOI.
% Compute pairwise distances between points from both sets
%disp(size(OOIs_center));
Landmarks = Landmarks';
distances = pdist2(Landmarks, OOIs_center);
% Set a threshold distance for matching points (between 0.5m and 1.5m)
threshold = 1;
% Find pairs of points whose distances are below the threshold
[indices1, indices2] = find(distances <= threshold);
% Extract the matched points from each set
matchedPoints1 = Landmarks(indices1, :);
matchedPoints2 = OOIs_center(indices2, :);


end


% ---------------------------------------------------------------------------------

function figur_handles = CreateFigureToShowScansInPolar()
   
     %figure for part E
    figure(1);
    hold on;
    movegui(gcf, [580 120]);
    title('Part E Bias');xlabel('X (m)');  ylabel('Y (m)'); axis([-5,20,-5,20]);grid on;


    figure(3); clf();
    hold on;
    movegui(gcf, [640 120]);
    aa = [-75: 0.5 : 75];  % LiDAR's FoV  ( -75 to +75 degrees), discretized by angular resolution (0.5 deg).
    r=aa*0;   % same number of ranges  ( i.e. 301 )
    
    % create figures, and graphic objects for showing, later, data dynamically.
    % Use subfigures ("subplot") 
    subplot(221);  
    h1 = plot(aa,r,'.b');
    title('LiDAR1(shown in Polar)');  xlabel('angle (degrees)');  ylabel('range (m)'); axis([-75,75,0,20]); grid on;
    hold on;  
    h1b = plot(0,0,'r+');
    legend({'opaque pixels','brilliant pixels'});
        
    subplot(222); 
    h3 = plot(0,0,'.g'); 
    title('LiDAR1(shown in Cartesian)');  xlabel('X (m)');  ylabel('Y (m)'); axis([-20,20,-20,20]); grid on;
    hold on;  
    h3b = plot(0,0,'r*','markersize',2);
    subplot(223);  
    h2 = plot(aa,r,'.b'); 
    title('LiDAR2(shown in Polar)');  xlabel('angle (degrees)');  ylabel('range (m)'); axis([-75,75,0,20]); grid on;
    hold on;  
    h2b = plot(0,0,'r+');

    subplot(224);  
    h4 = plot(0,0,'.g'); 
    title('LiDAR2(shown in Cartesian)');  xlabel('X (m)');  ylabel('Y (m)');axis([-20,20,-20,20]);grid on;
    hold on;  
    h4b = plot(0,0,'r*');
    

    %figure for part F
    figure(2);
    movegui(gcf, [100 120]);
    hold on;
    title('Part F, Estimated Posetion');xlabel('X (m)');  ylabel('Y (m)'); axis([-5,20,-5,20]);grid on;
    

    figure(4);
    hold on;
    movegui(gcf, [70 120]);
    
    h5 = quiver(0,0,'O','LineWidth', 2,'MaxHeadSize', 0.5,'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','k');
    %h5 = quiver(0,0,'s','LineWidth', 1,'MarkerFaceColor','r','MarkerEdgeColor','k');
    title('OOIs globl CF');  xlabel('X (m)');  ylabel('Y (m)'); axis([-5,21,-5,26]); grid on;
    hold on;  
    h5b = plot(0,0,'m.','MarkerSize',1);     %dots from lidar1             %10
    h5c1 = plot(0,0,'b');                   %DA from lidar1                %11              %%should have like 5 haldles here.
    h5c2 = plot(0,0,'b');
    h5c3 = plot(0,0,'b');
    h5c4 = plot(0,0,'b');
    h5c5 = plot(0,0,'b');

    h5d = plot(0,0,'r*','markersize',3);                   %ooi_center dynamicly lidar1 %16
    h5e = plot(0,0,'k.','MarkerSize',1);  %dots from lidar2              %13
    h5f1 = plot(0,0,'b-');                %DA from lidar2    %11          %14
    h5f2 = plot(0,0,'b-');
    h5f3 = plot(0,0,'b-');
    h5f4 = plot(0,0,'b-');
    h5f5 = plot(0,0,'b-');
    h5g = plot(0,0,'r*','markersize',3);                %ooi_center dynamicly lidar2     %15
     legend({'Landmarks','Walls (middle planes)','initial position','UGV','LiDaR1','LiDaR2'});
    %legend({'scans1','scans2'});


   
    %               [1 , 2,  3, 4, 5,  6 ,7,  8 ,9, 10,  11,  12,  13 , 14, 
    figur_handles = [h1,h1b,h2,h2b,h3,h3b,h4,h4b,h5,h5b,h5c1,h5c2,h5c3,h5c4,h5c5,h5d,h5e,h5f1,h5f2,h5f3,h5f4,h5f5,h5g]; %retrun hadnles to the figurs
end    
% ---------------------------------------------------------------------------------


function X = MyKinematicModel(X,vw,dt)

       tun = set_bias();

       vw_tun = vw + tun;  %take the gyroscope values and correct it, been used in X(3)

     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw_tun(2) * dt;
end  s
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

% Part E

% Adjust bias and angular error here:

function bias = set_bias()               %For Part E, set to 0 for dataset 002 ,, -0.007 for p020 ,, 0.007 for p2021
bias = 0;%0.007;
end

function angular_error = corrct_angle()   % used in [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg)
 angular_error = 0;%-6;
end
% ---------------------------------------------------------------------------------

%Part F

function [OOI_pos_carCF] = OOI_local_to_car(scan,LidarCfg) 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
    alpha = LidarCfg.Alpha;
    %the rotation matrix LiDar to UVG:
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];

    % now get the OOI in local LiDaR_CF using detect_ooi function
    [ooi_positions] = Detect_OOIs(scan);  %return OOI_pos in cartesian local LiDaR_CF 
    % loop over detected OOIs and find the these Points in UVG_CF
    OOI_p_local = zeros(size(ooi_positions, 1),2);
    for i = 1:size(ooi_positions, 1)
        OOI_p_local(i,:) = R_LiDaR * [ooi_positions(i, 1),ooi_positions(i, 2)]' +[lx;ly]; % x_v pos of ooi in vehicle CF
    end
    OOI_pos_carCF = OOI_p_local;

end 


% Define the function that calculates the error between the predicted and actual landmark positions in the GCF
function error = calc_error(landmarks,scan,X_0,X_kin_mod,LidarCfg)
    % Convert the positions of the OOIs in the car's CF to the GCF using the pose estimate
    OOI_global_pos = local_to_globalk(scan, X_0,X_kin_mod,LidarCfg);
    
    % Calculate the predicted positions of the landmarks in the GCF using the positions of the OOIs in the GCF
    landmarks_predicted = detected_OOIs_center(OOI_global_pos);
   landmarks = landmarks';
    % ... add more landmarks as needed

    % Calculate the error between the predicted and actual landmark positions in the GCF
    for i =1 : length(landmarks_predicted)
        error = norm(landmarks_predicted(i) - landmarks(i));% + norm(landmark2_predicted - landmark2) + norm(landmark3_predicted - landmark3) + ... % add more landmarks as needed
    end
end


% Define the function that minimizes the error function to obtain the pose estimate
function pose_estimate = estimate_pose(landmarks,scan,X_0,X_kin_mod,LidarCfg)
    % Initialize the pose estimate to the identity matrix
    pose_estimate = eye(4);
    
    % Use an optimization algorithm to minimize the error function
    options = optimset('MaxIter', 1000, 'TolFun', 1e-6);
    pose_estimate = fminsearch(@(pose_estimate) calc_error(landmarks,scan,X_0,X_kin_mod,LidarCfg), pose_estimate, options);
end

function plot_for_part_F(pose_buffer)
    disp(pose_buffer);
    figure(2)
    hold on;
    plot(pose_buffer(:, 1), pose_buffer(:, 2),'b-');

   % plot(X_kin_mod(1,:),X_kin_mod(2,:), 'k-'); %plot the data from the kinematic model
    %legend({'ground Truth','kinematic model'});
 end

% ---------------------------------------------------------------------------------

%Buttons callback functions
function MyCallbackBye(a,b)
   global MyFlags;
   disp('Bye!');
   MyFlags(1)=0; 
end

function MyCallbackPauseContinue(a,b)
   global MyFlags;
   MyFlags(2) = ~MyFlags(2) ; 
end






% ********** List of functions implemented in this program: **************
%{

-function ExploreData(data)
-function plot_envirnment = InitCertainPartOfMyProgram(data)
-function ShowVerification1(data,X_kin_mod,global_pos1)
-function processLiDAR_plot_ploar(figur_handles,scan)   ->plot dots and OOIs in polar 
-function [ooi_positions] = Detect_OOIs(scan)            ->return OOI_pos in cartesian LiDaR_CF
-function processLiD_cartiesian_plot(figur_handles_local,figure_handle_global,scan,T,X_kin_mod, LidarCfg) ->plot scans and OOI in cartes. LiDaR_CF and global
-function [x,y] = Ranges_in_cartesian_LiDaR_CF(scan)     ->used in two fuctions, returns pos of dots in cartesan local CF
-function [OOI_global_pos] = local_to_globalk(scan, X_0,X_kin_mod,LidarCfg) 
-function [scans_global_pos] = scan_local_to_global(scan, T,X_kin_mod,LidarCfg) ->Retrun the all dots in Global_CF
-function figur_handles = CreateFigureToShowScansInPolar()
-function X = MyKinematicModel(X,vw,dt)
etc..
%}
