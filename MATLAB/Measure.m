%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
% EKF/UKF measurement function. Computes the expected measurements  %
% given a state.                                                    %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
% By Geoffrey Bower                                                 %
% (C) 2011                                                          %
% Last Updated 5-25-2011                                            %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
% 6 measurement quantities:                                         %
% x,y,z accelerations                                               %
% x,y,z magnetic field strength                                     %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z] = Measure(x,xyz)
z = zeros(6,1);

%Compute direction cosine matrix from quaternions
Tb2i = [1-2*x(3)*x(3)-2*x(4)*x(4)  2*x(2)*x(3)-2*x(1)*x(4)     2*x(2)*x(4)+2*x(1)*x(3)
    2*x(2)*x(3)+2*x(1)*x(4)     1-2*x(2)*x(2)-2*x(4)*x(4)  2*x(3)*x(4)-2*x(1)*x(2)
    2*x(2)*x(4)-2*x(1)*x(3)     2*x(3)*x(4)+2*x(1)*x(2)     1-2*x(2)*x(2)-2*x(3)*x(3)];

%Compute expected accelerations in body axes
z(1:3) = Tb2i'*[0,0,-1]';

%Compute expected field strength in body axes
z(4:6) = Tb2i'*xyz; %xyz is expected magnetic field in NED coordinates

end