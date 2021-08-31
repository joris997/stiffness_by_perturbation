% grf = importdata("data/GRFs.mot")
% 
% figure
% subplot(2,3,1)
% hold on
% plot(grf.data(:,1),grf.data(:,2))
% plot(grf.data(:,1),grf.data(:,3))
% plot(grf.data(:,1),grf.data(:,4))
% 
% states = importdata("data/scaled_states.sto")
% initialstate = states.data(1,:);
% 
% headers1 = states.colheaders{78}
% headers2 = states.colheaders{76}

syms k m g y dy ddy Fadd t L0 real
q = y; dq = dy; ddq = ddy;

T = 1/2*m*dq^2;
V = m*g*y + 1/2*k*(y-L0)^2;

%% lagrange
dT_ddq = jacobian(T,dq).';
ddT_ddqdt = jacobian(dT_ddq,t) + ...
            jacobian(dT_ddq,q)*dq + ...
            jacobian(dT_ddq,dq)*ddq;

dT_dq = jacobian(T,q).';

dV_dq = jacobian(V,q).';

Q = Fadd;

EoM = simplify(ddT_ddqdt - dT_dq + dV_dq - Q);

[A,b] = equationsToMatrix(EoM,ddq);

%% Parameters
m = 20;
g = 9.81;
k = 1234;
L0 = 0;

%% 0 N
tEnd = 1;
q0 = [-0.2 0];

Fadd0 = 0;
[t0,x0] = ode45(@(t,x)evalEoM(t,x,m,g,k,L0,Fadd0),[0 tEnd],q0);

%% 100 N
Fadd100 = 100;
[t100,x100] = ode45(@(t,x)evalEoM(t,x,m,g,k,L0,Fadd100),[0 tEnd],q0);

%% postprocess
figure; hold on; grid on;
plot(t0,x0(:,1))
% plot(t100,x100(:,1))

% x1 = x0(end,1);
% x2 = x100(end,1);
% v1 = x0(end,2);
% v2 = x100(end,2);
% a1 = evalEoM(0,x0(end,:),m,g,k,L0,Fadd0); a1 = a1(2);
% a2 = evalEoM(0,x100(end,:),m,g,k,L0,Fadd100); a2 = a2(2);

% K = (Fadd100-Fadd0)/(v2 - v1)*tEnd
% K = (Fadd100-Fadd0)/(x2 - x1)

% y0 = x0(1,1);
% y1 = x0(end,1);
% dy0 = x0(1,2);
% dy1 = x0(end,2);
% ddy0 = evalEoM(0,x0(1,:),m,g,k,L0,Fadd0); ddy0 = ddy0(2);
% ddy1 = evalEoM(0,x0(end,:),m,g,k,L0,Fadd100); ddy1 = ddy1(2);
% 
% syms m F real
% msol = solve((F*y0)/((y0-y1)*m) + g - ddy1 == 0,m)
% 
% Fsol = solve((F*y1)/((y0-y1)*msol) + g - ddy1 == 0,F)

function ddq = evalEoM(~,q,m,g,k,L0,Fadd)
    y = q(1); dy = q(2);
    ddy = (Fadd - g*m + (k*(2*L0 - 2*y))/2)/m;
    
    ddq = [dy; ddy];
end