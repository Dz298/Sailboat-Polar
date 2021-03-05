%{
% Optimize Sail Area by max L/D ratio
    argmax_{SA_sail}(L/D) 
    constraints: 
        v_wind = 10 m/s;
        alpha_sail = 5 degrees;
        m_sail <= 5 kg
%}
[p,~]=setBoatParam;
w_spd = 10;
alpha_sail = deg2rad(5);
[c_lift,c_drag]=C_LD(alpha_sail,p);
p.m_sail = 0.5;
SA_sail = p.m_sail/p.rho_sail;
p.SA_sail = SA_sail;
v_sail = [0,-w_spd];
while (p.m_sail<=5)
    %lift and drag on sail
    L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
        c_lift*[-v_sail(2),v_sail(1)];
    D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
        c_drag*(-v_sail);
    plot(SA_sail,norm(L_sail)/norm(D_sail),'k.');
    hold on
    pause(.1)
    SA_sail = SA_sail+0.01;
    p.SA_sail = SA_sail;
    p.m_sail = p.SA_sail*p.rho_sail;
end

