%------------------------------------------------------------------------------
%Least Squares Sensor Pose Calibration
%------------------------------------------------------------------------------

clear;clc;

%initial guess of the state vector which is made of the
%sensor pose [Xk;Yk;Zk;Rk;Pk;Yk]
x = [0.42; 0.2875; 0.52; -1.7705; 0.0176; -1.6170];
%x = [0.102259;-0.0115757;0.4;-1.5617;0.0120;-1.5606];

%measurement covariance
Omega = eye(6);

%linear system matrices for solving Least Squares
H=zeros(6);
b=zeros(6,1);

%default increment for computing numeric Jacobian
eps = 1.e-5;
deltaX=zeros(6,1);

count = 0;
iterate=2;
th = 0.0002;
max_iterations = 100;
l = 1000;

disp(' ')
disp('#----------------------------------------------------------------------#')
disp('Least Squares Sensor Pose Calibration')
disp('#----------------------------------------------------------------------#')
disp(' ')

%load data file
load output.txt
[m, n] = size(output);

if m ~= 0
    fprintf('[INFO]: output.txt file has %d lines\n',m);
    disp('Press any key to continue...');
    pause
end

while (iterate > 1)
    
    err_norm = 0;
    count = count + 1;
    disp('Iteration: ')
    disp(count)
    for i = 1:m
        
        %acquire data from file
        measurement = transpose(output(i,1:6));
        odometry = transpose(output(i,7:n));
        X = v2t(x);
        M = v2t(measurement);
        O = v2t(odometry);
        
        %predict sensor pose from robot pose and transform
        P = prediction(O,X);
        
        %compute difference between prediction and "ground truth"
        e = error_function(P,M);
        err_norm = err_norm + norm(e);
        
        %Compute numeric Jacobian
        J = zeros(6,6);
        x_up = x;
        
        for j = 1:6
            x_up(j) = x_up(j) + eps;
            perturbedP = prediction(O,v2t(x_up));
            e_up = error_function(perturbedP,M);
            J(:,j) = (e_up - e)/eps;
            %J(:,j) = error_function(perturbedP,P)/eps;
            x_up(j) = x(j);
        end
        
        %update the linear system matrices H and b
        H = H + (J')*Omega*J;
        b = b + (J')*Omega*e;
        
    end
    
    err_norm
    H = H + l * eye(6);
    deltaX = -H\b ;
    DeltaX = v2t(deltaX);
    x = t2v(X*DeltaX);
    current_x = x'
    deltaX_norm = norm(deltaX)
    disp(' ')
    disp('-------------------------------------------------------------------->>')
    disp(' ')
    
    %pause
    
    if ( deltaX_norm < th || count > max_iterations )
        iterate = 0;
        x
        disp('Stop iterating!!!')
    end
    
end
%[ 0.42 0.2875 0.52 -1.7705 0.0176 -1.6170]

