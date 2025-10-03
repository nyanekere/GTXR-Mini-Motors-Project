
function [MEOP, specific, impulse, T_avg, T_max, exit_area, Mach, time, max_flow_vel, port_to_throat, motor_class, max_mass_flux]  = RideMyRocket(no_grains, grain_inner, grain_len, throat_dia)

    time = 0;
    % N-Class Motor Simulation
    % Inputs
    no_grains = 9;
    grain_inner= 1.3737;
    grain_len = 5.792;
    throat_dia = 1.1167;
    %Constants
    grain_outer = 3.239; % Grain Outer Diameter
    A = pi * (throat_dia/2)^2; % Throat Area
    a = 0.0270; % Burn Rate Coefficient
    n = 0.3; % Burn Rate Exponent
    gamma = 1.2; % Specific Heat Ratio
    grain_density = 0.0018;
    c_star = 4890; % Characteristic Velocity
    R = 2000; % Gas Constant
    T_0 = 4700; %
    time_step = 0.01;
    port_to_throat = (grain_inner^2/throat_dia^2); %Port Area/Throat Area
    N_class = false;
    % Initial Propellant Calculations
    propellant_amt = (((pi*(grain_outer/2)^2)*grain_len)-((pi*(grain_inner/2)^2)*grain_len)) * grain_density; % Propellant Amount = mass
    area_top = (pi*((grain_outer/2)^2)) - (pi*((grain_inner/2)^2));% intial are of the top
    inner_area = (pi * grain_inner) * grain_len;
    A_b = inner_area + (2 * area_top);
    propellant_weight = 32.174 * propellant_amt * no_grains;
    Chamber_pressure = [];
    m_dot_vec = [];
    thrust_vec =[];
    flow_velocity_vec = [];
    m_flux_vec = [];

    fprintf('Number of Grains: %f \n', no_grains)
    fprintf('Grain Dimensions: %f length, %f inner diameter, %f outer diameter \n', grain_len, grain_inner, grain_outer)
    fprintf('Throat Area: %f in^2 \n', A)
    fprintf('Propellant Weight: %f lbm \n', propellant_weight)
    
    while propellant_amt>0
        total_area_burn = no_grains*A_b; % Calculate total area burned
        K_n = total_area_burn/A; %Calculate ratio between total area burned and throat area
        P_c = (K_n*c_star*a*grain_density)^(1/(1-n)); %Calculate chamber pressure
        r_dot = a*(P_c)^n; %Calculate burn rate (in/s)
        %Call grain burn function
        [m_dot, A_b, propellant_amt, grain_inner, grain_len] = grain_burn(propellant_amt, grain_len, grain_inner, grain_outer, time_step, r_dot, no_grains, grain_density);
        Chamber_pressure = [Chamber_pressure, P_c];
        m_dot_vec = [m_dot_vec m_dot];
        m_flux = m_dot/(pi*grain_inner^2/4);
        m_flux_vec = [m_flux_vec,m_flux];
        time = time + time_step;
    end
    
    MEOP = max(Chamber_pressure);
    OptimalPc = mean(Chamber_pressure); %This is the chamber pressure for which the motor is optimised
    outside_pressure = 14.7;
    P_ratio = outside_pressure/OptimalPc;
    
    if (P_ratio>=1)
        
        MEOP = 0;
        specific = 0;
        impulse = 0;
        T_avg = 0;
        T_max = 0;
        exit_area = 0;
        Mach = 0;
        time = 0;
        max_flow_vel = 0;
        port_to_throat = 0;
        motor_class = 'A';
        max_mass_flux = 0;

        return

    else
    
    [Mach_max,T_ratio, P_ratio, rho_ratio, Area_ratio] = flowisentropic(gamma,P_ratio,'pres');
    
    end
    
    for i = 1:length(Chamber_pressure)
        exit_pressure = P_ratio * Chamber_pressure(i);
        exit_area = A * Area_ratio;
        s_Sound = sqrt(R * gamma * (T_0*T_ratio));
        m_dot_FL = m_dot_vec(i);
        Mach = Mach_max;
        [inst_thrust, flow_velocity] = thrust(m_dot_FL, Mach, exit_area , exit_pressure, outside_pressure, s_Sound);
        %instant thrust for the loop
        % mach vector is edited
        thrust_vec = [thrust_vec, inst_thrust];
        flow_velocity_vec = [flow_velocity_vec, flow_velocity];
    end
    
    
    T_max = max(thrust_vec);
    max_flow_vel = max(flow_velocity_vec);
    T_avg = mean(thrust_vec);
    
    impulse = T_avg * time;
    specific = impulse/ propellant_weight;
    propellant_weight;
    propellant_amt;
    max_m_dot = max(m_dot_vec);
    max_mass_flux = 32.174*max(m_flux_vec);
    Temp_max = T_ratio*T_0;
    motor_class = check_motor(impulse);


    
    fprintf('Max Chamber Pressure: %f psi \n', MEOP)
    fprintf('Max Mass Flux: %f lbm/second\n', max_mass_flux)
    fprintf('Burn Time: %f \n', time)
    fprintf('Design Pressure Ratio: %f \n', P_ratio)
    fprintf('Max Exit Mach Number %f \n', Mach)
    fprintf('Max Exit Temperature: %f R \n', Temp_max)
    fprintf('Max Velocity: %f ft/s\n', max_flow_vel)
    fprintf('Max Thrust: %f lbf\n', T_max)
    fprintf('Avg Thrust: %f lbf\n', T_avg)
    fprintf('Optimum Exit Area: %f in^2\n', exit_area)
    fprintf('Total Impulse: %f lbf s\n', impulse)
    fprintf('Specific Impulse: %f s\n', specific)
    fprintf('Port to Throat Ratio: %f \n', port_to_throat)
    fprintf('Motor Class: %s \n', motor_class)
    % 
    
    % Plots
    % Thrust vs. Time
    % Mdot vs. Time	
    % Chamber Pressure vs. Time
    % figure
    % time_vec = 0:time_step:(length(thrust_vec) - 1) * time_step;
    % plot(time_vec, thrust_vec)
    %     title('Thrust vs. Time');
    %     xlabel('Time (s)');
    %     ylabel('Thrust (lbs)')
    % figure
    % time_vec = 0:time_step:(length(m_dot_vec) - 1) * time_step;
    % plot(time_vec, m_dot_vec)
    % title('Mass Flux vs. Time')
    % xlabel('Time (s)')
    % ylabel('Mass Flux (slugs/s)')
    % figure
    % time_vec = 0:time_step:(length(Chamber_pressure) - 1) * time_step;
    % plot(time_vec, Chamber_pressure)
    % title('Chamber Pressure vs. Time')
    % xlabel('Time (s)')
    % ylabel('Chamber Pressure (psi)')
    % overall function ends
    % function to get instant thrust vector
    % Ae is outer throat diameter
    
    function [inst_thrust, flow_velocity] = thrust(m_dot,Mach, Ae, exit_pressure, outside_pressure, s_Sound);
        flow_velocity = Mach * s_Sound;
        inst_thrust = (flow_velocity*m_dot)+ Ae*(exit_pressure - outside_pressure);
    end
    
    %Helper grain burn function for debugging purposes
    function [m_dot,A_b,new_prop_amt, new_ID, new_grain_len] = grain_burn(prop_amt,grain_len,ID,OD,time_step,r_dot, no_grains, grain_density)
        %This function gets the new grain geometry after every time step passes
        Burn_depth = r_dot *time_step; % the burn depth is length
        new_grain_len = grain_len-(2* Burn_depth);% subtracts the change in grain length from both sides from the orginal length
        new_ID = ID + (2 * Burn_depth);% new inner diameter
        area_top = (pi*((OD/2)^2)) - (pi*((ID/2)^2));% intial are of the top
        inner_area = (pi * ID) * grain_len;
        A_b = inner_area + (2 * area_top);
        burn_vol_top = area_top * Burn_depth;
        burn_area_top = pi * (((new_ID^2)/4)-((ID^2)/4));
        burn_vol_side = burn_area_top * new_grain_len;
        tot_burn_vol = (burn_vol_top*2)+burn_vol_side;
        delta_mass = tot_burn_vol * grain_density; % gets change in mass from volume burned
        m_dot = (delta_mass/time_step) * no_grains; % m dot is change in mass over time
        new_prop_amt = prop_amt - delta_mass; % gets new prop amount
    end

    function [motor_class] = check_motor(impulse)
        if(impulse <= 4604 & impulse >= 2302.01)
            motor_class = "N";
        elseif(impulse <= 18400 & impulse >= 9210)
            motor_class = "P";
        else
            motor_class = "Error";
        end

    end


end

