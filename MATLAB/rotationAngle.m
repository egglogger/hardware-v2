% rotationAngle
%
% [angle, vector] = rotationAngle(roll_1, pitch_1, yaw_1, roll_2, pitch_2,
% yaw_2) computes the incremental rotation angle between two poses
% described by the Euler angles (roll, pitch, and yaw). The vector
% indicates the body axis about which the rotation occurs (per Euler's
% axis-angle theorem). The input and output angles are in radians. The
% output vector is a unit vector.
%
% angle = rotationAngle(roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2)
% computes the same as above, but does not return the vector. The input and
% output angles are in radians.
%

function [angle, vector] = rotationAngle(roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2)

% Check the size of all the inputs and make sure they are the same length
n = length(roll_1);

% Loop over the inputs
angle = zeros(n,1);
vector = zeros(n,3);
for i = 1:n
    
    % Check if there is no rotation, if so choose arbitrary vector
    if (yaw_1(i) == yaw_2(i) && pitch_1(i) == pitch_2(i) && roll_1(i) == roll_2(i))
        angle(i) = 0;
        vector(i,:) = [1,0,0];
    else
        % Compute the rotation matrix for the first orientation (body to
        % inertial rotation matrix)
        R1 = [cos(yaw_1(i)) * cos(pitch_1(i)), -cos(roll_1(i)) * sin(yaw_1(i)) + sin(roll_1(i)) * sin(pitch_1(i)) * cos(yaw_1(i)), sin(roll_1(i)) * sin(yaw_1(i)) + cos(roll_1(i)) * sin(pitch_1(i)) * cos(yaw_1(i));
            cos(pitch_1(i)) * sin(yaw_1(i)), cos(roll_1(i)) * cos(yaw_1(i)) + sin(roll_1(i)) * sin(pitch_1(i)) * sin(yaw_1(i)), -sin(roll_1(i)) * cos(yaw_1(i)) + cos(roll_1(i)) * sin(pitch_1(i)) * sin(yaw_1(i));
            -sin(pitch_1(i)), sin(roll_1(i)) * cos(pitch_1(i)), cos(roll_1(i)) * cos(pitch_1(i))];
        
        % Compute the rotation matrix for the second orientation (body to
        % inertial rotation matrix)
        R2 = [cos(yaw_2(i)) * cos(pitch_2(i)), -cos(roll_2(i)) * sin(yaw_2(i)) + sin(roll_2(i)) * sin(pitch_2(i)) * cos(yaw_2(i)), sin(roll_2(i)) * sin(yaw_2(i)) + cos(roll_2(i)) * sin(pitch_2(i)) * cos(yaw_2(i));
            cos(pitch_2(i)) * sin(yaw_2(i)), cos(roll_2(i)) * cos(yaw_2(i)) + sin(roll_2(i)) * sin(pitch_2(i)) * sin(yaw_2(i)), -sin(roll_2(i)) * cos(yaw_2(i)) + cos(roll_2(i)) * sin(pitch_2(i)) * sin(yaw_2(i));
            -sin(pitch_2(i)), sin(roll_2(i)) * cos(pitch_2(i)), cos(roll_2(i)) * cos(pitch_2(i))];
        
        % Compute the incremental rotation matrix
        Rinc = R1\R2;
        
        % Compute the rotation vector
        angle(i) = acos(0.5 * (Rinc(1,1) + Rinc(2,2) + Rinc(3,3) - 1));
        vector(i,1) = (Rinc(3,2) - Rinc(2,3))/(2*sin(angle(i)));
        vector(i,2) = (Rinc(1,3) - Rinc(3,1))/(2*sin(angle(i)));
        vector(i,3) = (Rinc(2,1) - Rinc(1,2))/(2*sin(angle(i)));
    end
end

end