function dx = droneDynamics(x, v, ~, ~)

    
    pos = x(1:3);       % x, y, z
    eul = x(6:-1:4);       % phi, theta, psi
    lin_vel = x(7:9);   % u, v, w
    ang_vel = x(10:12); % p, q, r

    % Parametri
    m = 1.32;
    g = [0; 0; 9.81];
    J = diag([0.0154 0.0154 0.0263]);
    AT = diag([0.9 0.9 0.75]);
    AR = diag([0.6 0.6 0.6]);

    % Estrazione da v
    fP = v(1:3);
    tauP = v(4:6);


    % Matrici di rotazione
    R = eul2rotm(eul', 'ZYX');

    % Dinamica traslazionale
    dot_lin_vel = g + (1/m)*(R*(fP - AT*lin_vel));
   

    % Dinamica rotazionale
    omega = ang_vel;
    dot_omega = inv(J)*(-cross(omega, J*omega) + tauP - AR * omega);


    % Uscita finale
    dx = [lin_vel;
          ang_vel;
          dot_lin_vel;
          dot_omega];

end
