function delta_theta = angle_difference(theta1, theta2)
    delta_theta = theta2 - theta1;
    delta_theta = mod(delta_theta + pi, 2*pi) - pi;
end