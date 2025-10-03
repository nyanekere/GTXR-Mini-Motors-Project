% Constants
grain_outer_diameter = 3.239;
% Constraints
max_grain_len = 7; % in
min_grain_len = 3;
max_no_grains = 11;
min_no_grains = 7;
max_inner_diameter = 1.4;
min_inner_diameter = 0.9;
max_throat_diameter = 1.2;
min_throat_diameter = 0.8;
min_port_to_throat = 1.5;
min_MEOP = 300; %psi
max_MEOP = 800; %psi
max_motor_length = 52; %TBD
min_motor_length = 47; % TBD
% Initial Values
no_grains = linspace(min_no_grains, max_no_grains, 5);
grain_inner= linspace(min_inner_diameter, max_inner_diameter, 20);
grain_len = linspace(min_grain_len, max_grain_len, 20);
throat_diameter = linspace(min_throat_diameter, max_throat_diameter, 25);
% no_grains = [5];
% grain_inner= [2.1805];
% grain_len = [10];
% throat_dia = [1.46599];
counter = 1;
output_data = [];
for grain_num = no_grains
for grain_in = grain_inner
for len_grain = grain_len
for throat_d = throat_diameter
[MEOP, specific, impulse, T_avg, T_max, exit_area, Mach, time, max_flow_vel, port_to_throat, motor_class, max_mass_flux] = RideMyRocket(grain_num, grain_in, len_grain, throat_d);
real_output = checkConstraints(port_to_throat, MEOP, len_grain, motor_class, grain_num, max_motor_length, min_motor_length);
if real_output == true
A = [MEOP, specific, impulse, T_avg, T_max, exit_area, Mach, time, max_flow_vel, port_to_throat, motor_class, max_mass_flux, grain_num, grain_in, len_grain, throat_d];
output_data = [output_data; A];
counter = counter + 1;
end
end
end
end
end

% Display Top 5 Impulse Motors
[impulse1, impulse2, impulse3, impulse4, impulse5] = bestImpulse(output_data)

function [real_output] = checkConstraints(port_to_throat, MEOP, len_grain, motor_class, grain_num, max_motor_length, min_motor_length)
if((MEOP >= 300) && (MEOP <= 670) && ...
(port_to_throat > 1.5) && ...
(len_grain * grain_num < max_motor_length) && (len_grain * grain_num > min_motor_length)) 
real_output = true;
else
real_output = false;
end
end

function [impulse1, impulse2, impulse3, impulse4, impulse5] = bestImpulse(arr)
impColumn = arr(:,3);
[sortedImp, impInds] = sort(impColumn);
maxes = sortedImp(end:-1:end-4);
impulse1 = maxes(1);
rowImpulse1 = impInds(end)
impulse2 = maxes(2);
rowImpulse2 = impInds(end-1)
impulse3 = maxes(3);
rowImpulse3 = impInds(end-2)
impulse4 = maxes(4);
rowImpulse4 = impInds(end-3)
impulse5 = maxes(5);
rowImpulse5 = impInds(end-4)

fprintf ('')

end

%&& (strcmp(motor_class, "N")))