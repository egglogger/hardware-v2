%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
% Function to estimate the initial heading from the magnetic field  %
% data using non-linear least squares.                              %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
% By Geoffrey Bower                                                 %
% Last Updated 9-21-2010                                            %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [F] = findHeading(x,theta0,phi0,xMag,yMag,zMag,xyz)

psi = x;
F(1) = cos(theta0)*cos(psi)*xMag + (sin(theta0)*sin(phi0)*cos(psi)-sin(psi)*cos(phi0))*yMag + (sin(theta0)*cos(phi0)*cos(psi)+sin(phi0)*sin(psi))*zMag - xyz(1);
F(2) = cos(theta0)*sin(psi)*xMag + (sin(theta0)*sin(phi0)*sin(psi)+cos(phi0)*cos(psi))*yMag + (sin(theta0)*cos(phi0)*sin(psi)-sin(phi0)*cos(psi))*zMag - xyz(2);
F(3) = -sin(theta0)*xMag + cos(theta0)*sin(phi0)*yMag + cos(theta0)*cos(phi0)*zMag - xyz(3);

end