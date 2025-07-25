function J = CostFunction(x, u, e, data)

    ref = data.References;


    % Cost initialization
    
    N = data.PredictionHorizon;
    
    Qp = diag([10e5 10e5 10e6]);
    Qeta = diag([10e-4 10e-4 10e2]);
    Qvel = diag([10e-2 10e-2 10e-2]);
    Qw = diag([10e-2 10e-2 10e-2]);
    Qv = 0.01*diag([1 1 1 1 1 1]);
    
    w_e = 1e4;                % peso slack (≈10^4)

  
    
    %global ref_matrix
    
    J = 0;  % inizializza costo
    
    for k = 1:N
      
        pos = x(k,1:3)';
        eta = x(k,6:-1:4)';
        vel = x(k,7:9)';
        omega = x(k,10:12)';

        pos_des = ref(k,1:3)';
        eta_des = ref(k,6:-1:4)';
        vel_des = ref(k,7:9)';
        omega_des = ref(k,10:12)';

        Rb = eul2rotm(eta','ZYX');
        Rb_des = eul2rotm(eta_des','ZYX');

        err_pos = pos - pos_des ;       
        err_vel = vel - vel_des;
        eR = 0.5*vee(Rb_des'*Rb - Rb'*Rb_des);
        ew = omega - Rb'*Rb_des*omega_des;

        u_k = u(k, :)';      % prendi il comando k-esimo [6x1]
        
    
        J = J + err_pos'*Qp*err_pos + err_vel'*Qvel*err_vel + eR'*Qeta*eR + ew'*Qw*ew + u_k'*Qv*u_k + w_e*e^2;
    end


end

function v = vee(S)
    v = [S(3,2); S(1,3); S(2,1)];
end


