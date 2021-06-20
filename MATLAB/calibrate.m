function [Afinal,Bfinal] = calibrate(x,y,z)

debug = 0; % Turns on plots
data = [x; y; z];
nPoints = length(data);
k = 1;
vars = [1,1,1,0,0,0,0,0,0]'; % a11, a22, a33, a12, a13, a23, b1 ,b2, b3

maxIter = 1000;
tol = 1e-5;

Afinal = diag([1,1,1]);
Bfinal = [0;0;0];
objOld = 1;
objNew = 0;

if debug == 1
    plotColor = jet(maxIter);
    figure;
    hold on;
end

while (abs(objOld-objNew) > tol && k < maxIter)
        
    if debug == 1
        options = optimset('Display','iter');
    else
        options = optimset('Display','off');
    end
    [vars, resnorm] = lsqnonlin('sumSquares',vars,[],[],options,data);
    
    objOld = objNew;
    objNew = resnorm;
    
    clc
    disp(['Residual : ',num2str(resnorm)]);
    
    A = [vars(1),vars(4),vars(5);0,vars(2),vars(6);0,0,vars(3)];
    B = [vars(7);vars(8);vars(9)];
    data = A*data + B*ones(1,nPoints);
    
    Afinal = A*Afinal;
    Bfinal = A*Bfinal + B;
    
    k = k + 1;   
    
    if debug == 1
        hold on
        disp(resnorm);
        plot3(data(1,:),data(2,:),data(3,:),'.','Color',plotColor(k,:))
        xlabel('x')
        ylabel('y')
        zlabel('z')
        axis equal
        grid on
        view(3)
        title(k)
        pause(0.01)
    end
end

if k >= maxIter
    disp('Maximum iterations reached, invalid calibration!')
else
    disp([num2str(k),' iterations required']);
end

end