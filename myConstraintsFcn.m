function ineq = myConstraintsFcn(x, u, e, data)

    cf = 1.65776*10^(-5);
    cm = 2.1792*10^(-7);
    l=0.45;

    B = [1 0 1 0 0 0 0 0;
         0 0 0 0 1 0 1 0;
         0 1 0 1 0 1 0 1;
         0 l 0 -l 0 0 0 0;
         0 0 0 0 0 -l 0 l;
         -l -cm/cf l -cm/cf l cm/cf -l cm/cf];

    u_real = pinv(B)*u;

    fp1 = [u_real(1) 0 u_real(2)]';
    fp2 = [u_real(3) 0 u_real(4)]';
    fp3 = [0 u_real(5) u_real(6)]';
    fp4 = [0 u_real(7) u_real(8)]';

    omega1 = (30/pi)*sqrt(norm(fp1)/cf);
    omega2 = (30/pi)*sqrt(norm(fp2)/cf);
    omega3 = -(30/pi)*sqrt(norm(fp3)/cf);
    omega4 = -(30/pi)*sqrt(norm(fp4)/cf);

    alfa1 = atan2(u_real(1),u_real(2));
    alfa2 = atan2(u_real(3),u_real(4));
    alfa3 = -atan2(u_real(5),u_real(6));
    alfa4 = -atan2(u_real(7),u_real(8));

    Omega = [omega1 omega2 omega3 omega4]';
    alfa = [alfa1 alfa2 alfa3 alfa4]';

    % Vincoli:
    Omega_min = -10000*10000;
    Omega_max = 10000*10000;
    alfa_min = -3*pi/2;
    alfa_max =  3*pi/2;

    ineq = [Omega-Omega_max;-Omega+Omega_min;alfa-alfa_max;-alfa+alfa_min];

   
end
