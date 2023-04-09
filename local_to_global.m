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
ooi_positions1=[];
ooi_positions2=[];
j = 1;
k =1;

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

    %X(t+dt) = discrete time model,  F(X(t),u(t))
    % at this line, the program variable X_0  contains the predicted state at the current time (in variable tNow)
    
end
disp(X_kin_mod(1,:))

function X = MyKinematicModel(X,vw,dt)
     % Here, you implement your discrete time model; e.g. using Euler's approach, etc.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt;
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw(2) * dt;


end  




 x_local=[];
    y_local=[];


    
    for i = 1:length(ooi_positions)
        for j = 1:length(ooi_positions(i))
            x_local = ooi_positions(j, 1);  %x_l
            y_local = ooi_positions(j, 2);  %y_l
        end
    end
    p_local = zeros(2,1);
 
    %(x_v, y_v)
    p_local(1) = x_local+0.4;
    p_local(2) = y_local;


 
   
    disp(size(p_local));