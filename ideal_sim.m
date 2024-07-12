clear all
close all
clc

animacion = 0;

% kuka modelo
wTp = @(p) [cos(p(3)) -sin(p(3)) 0 p(1); sin(p(3)) cos(p(3)) 0 p(2); 0 0 1 0; 0 0 0 1];
pTb = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.1; 0 0 0 1];

T01 = @(theta_1) [cos(theta_1),   0,      sin(theta_1), 0.033*cos(theta_1);
       sin(theta_1),   0, -1.0*cos(theta_1), 0.033*sin(theta_1);
                  0, 1.0,                 0,             0.1459;
                  0,   0,                 0,                1.0];
 
T12 = @(theta_2) [cos(theta_2 + 1.571), -1.0*sin(theta_2 + 1.571),   0, 0.155*cos(theta_2 + 1.571);
       sin(theta_2 + 1.571),      cos(theta_2 + 1.571),   0, 0.155*sin(theta_2 + 1.571);
                          0,                         0, 1.0,                          0;
                          0,                         0,   0,                        1.0];
 
T23 = @(theta_3) [cos(theta_3), -1.0*sin(theta_3),   0, 0.1348*cos(theta_3);
       sin(theta_3),      cos(theta_3),   0, 0.1348*sin(theta_3);
                  0,                 0, 1.0,                   0;
                  0,                 0,   0,                 1.0];
 
T34 = @(theta_4) [cos(theta_4 + 1.571),   0,      sin(theta_4 + 1.571),   0;
       sin(theta_4 + 1.571),   0, -1.0*cos(theta_4 + 1.571),   0;
                          0, 1.0,                         0,   0;
                          0,   0,                         0, 1.0];
 
T45 = @(theta_5) [cos(theta_5), -1.0*sin(theta_5),   0,      0;
       sin(theta_5),      cos(theta_5),   0,      0;
                  0,                 0, 1.0, 0.1936;
                  0,                 0,   0,    1.0];

% pose camara
eTc = [0 -1 0 0.05; 1 0 0 0; 0 0 1 -0.08; 0 0 0 1];

% calibracion camara
K = [600 0 320; 0 600 240; 0 0 1];

% pose objeto visto desde w
test = 1;
wto = [0.7 0.1 0.50]'; % [0.6 0.0 0.55]'
wRo = rotz(30)*rotx(20);

wTo = [[wRo wto]; 0 0 0 1];
op1 = [0.0 0.025 0.025 1]';
op2 = [0.0 -0.025 0.025 1]';
op3 = [0.0 -0.025 -0.025 1]';
op4 = [0.0 0.025 -0.025 1]';

%%
S = 40;
dt = 0.05;
N = S/dt;

q = [0 0 0 0 0 -pi/4 -pi/4 0]';
qo = q;

x1_d = [241.8608  150.4895]';
x2_d = [398.1392  150.4895]';
x3_d = [398.1475  306.7751]';
x4_d = [241.8525  306.7751]';

s1_d = Proyection_2D(x1_d,K);
s2_d = Proyection_2D(x2_d,K);
s3_d = Proyection_2D(x3_d,K);
s4_d = Proyection_2D(x4_d,K);
sd = [s1_d' s2_d' s3_d' s4_d']';

M_plot = [];
vc_plot = [];
s_plot = [];
sd_plot = [];
qp_plot = [];
q_plot = [];
e_plot = [];
px_plot =[];
pxd_plot =[];

for i=1:N
    Tb = wTp(q(1:3));
    T0 = Tb*pTb;
    T1 = T0*T01(q(4));
    T2 = T1*T12(q(5));
    T3 = T2*T23(q(6));
    T4 = T3*T34(q(7));
    T5 = T4*T45(q(8));

    wTe = T5;
    wTc = wTe*eTc;
    cTw = inv(wTc);

    t = cTw(1:3,4);
    R = cTw(1:3,1:3);

    St = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
    V = [R St*R; zeros(3,3) R];

    J = [Jacob(q); [0 0 0]' [0 0 0]' [0 0 1]' [0 0 1]' T1(1:3,3) T2(1:3,3) T3(1:3,3) T4(1:3,3)];

    % 3D proyection
    cTo = inv(wTc)*wTo;
    [x1,Z1] = Proyection_3D(K,cTo,op1);
    [x2,Z2] = Proyection_3D(K,cTo,op2);
    [x3,Z3] = Proyection_3D(K,cTo,op3);
    [x4,Z4] = Proyection_3D(K,cTo,op4);

    % 2D proyection
    s1 = Proyection_2D(x1,K);
    s2 = Proyection_2D(x2,K);
    s3 = Proyection_2D(x3,K);
    s4 = Proyection_2D(x4,K);
    s = [s1' s2' s3' s4']';

    L1 = Interaction_Matrix(s1,Z1);
    L2 = Interaction_Matrix(s2,Z2);
    L3 = Interaction_Matrix(s3,Z3);
    L4 = Interaction_Matrix(s4,Z4);
    L = [L1; L2; L3; L4];

    e = s-sd;

    vc = -1.5*pinv(L*V)*e;

    Ji = J'*inv(J*J' + 0.2*eye(6));
    q0 = Get_q0(q);

    kc = 2.6;
    qp = Ji*vc + (eye(8,8) - pinv(J)*J)*(kc*q0);

    q = q + qp*dt;


    M_plot = [M_plot sqrt(det(J*J'))];
    s_plot = [s_plot s];
    vc_plot = [vc_plot vc];
    qp_plot = [qp_plot qp];
    q_plot = [q_plot q];
    e_plot = [e_plot e];

    sd_plot = [sd_plot sd];
    px_plot = [px_plot [x1' x2' x3' x4']'];
    pxd_plot = [pxd_plot [x1_d' x2_d' x3_d' x4_d']'];


    if animacion
        cla
        hold on
        grid on
    
        Dibujar_Sistema_Referencia_3D(eye(4,4))
        Dibujar_MM(Tb,T0,T1,T2,T3,T4,T5)
        Dibujar_Camara(wTc)
        Dibujar_Objeto_3D(wTo,op1,op2,op3,op4)
    
        axis equal
        view([-60 50])
        drawnow
    end

    % % % Plotting
    % 
    % subplot(2,2,1)
    % hold on
    % I = 255*ones(480,640);
    % 
    % I = insertShape(I,"FilledCircle",[x1' 10],"Color","red","Opacity",1);
    % I = insertShape(I,"FilledCircle",[x2' 10],"Color","red","Opacity",1);
    % I = insertShape(I,"FilledCircle",[x3' 10],"Color","red","Opacity",1);
    % I = insertShape(I,"FilledCircle",[x4' 10],"Color","red","Opacity",1);
    % 
    % I = insertShape(I,"circle",[x1_d' 10],"Color","black","Opacity",1,LineWidth=5);
    % I = insertShape(I,"circle",[x2_d' 10],"Color","black","Opacity",1,LineWidth=5);
    % I = insertShape(I,"circle",[x3_d' 10],"Color","black","Opacity",1,LineWidth=5);
    % I = insertShape(I,"circle",[x4_d' 10],"Color","black","Opacity",1,LineWidth=5);
    % 
    % imshow(I)
    % 
    % subplot(2,2,2)
    % cla
    % hold on
    % grid on
    % 
    % plot(s1_d(1),s1_d(2),'rx','MarkerSize',10,'LineWidth',2)
    % plot(s2_d(1),s2_d(2),'rx','MarkerSize',10,'LineWidth',2)
    % plot(s3_d(1),s3_d(2),'rx','MarkerSize',10,'LineWidth',2)
    % plot(s4_d(1),s4_d(2),'rx','MarkerSize',10,'LineWidth',2)
    % 
    % plot(s1(1),s1(2),'ko','MarkerSize',10,'LineWidth',2)
    % plot(s2(1),s2(2),'ko','MarkerSize',10,'LineWidth',2)
    % plot(s3(1),s3(2),'ko','MarkerSize',10,'LineWidth',2)
    % plot(s4(1),s4(2),'ko','MarkerSize',10,'LineWidth',2)
    % 
    % drawnow
end

%%
t_plot = dt:dt:S;

%%
f = figure;
hold on
grid on
title('Manipulability measure','FontSize',14)
set(gca,'FontSize',14)
xlabel('time (s)','FontSize',14)
ylabel('$w(\mathbf{q})$','interpreter','latex','FontSize',16)
plot(t_plot,M_plot,'LineWidth',1.5)
Save_Figure(f,['manipulability_' num2str(test)])

%%
f = figure;
grid on
hold on
title('Joint positions','FontSize',14)
set(gca,'FontSize',14)
plot(t_plot,q_plot','LineWidth',1.5)
legend('$x_b$','$y_b$','$\theta_b$','$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','interpreter','latex','FontSize',14)
xlabel('time (s)','FontSize',14)
ylabel('joint values (m, rad)','FontSize',14)
Save_Figure(f,['JointPositions_' num2str(test)])

%%
f = figure;
grid on
hold on
title('Joint velocities','FontSize',14)
set(gca,'FontSize',14)
plot(t_plot,qp_plot','LineWidth',1.5)
legend('$\dot{x}_b$','$\dot{y}_b$','$\dot{\theta}_b$','$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$','$\dot{\theta}_4$','$\dot{\theta}_5$','interpreter','latex','FontSize',14)
xlabel('time (s)','FontSize',14)
ylabel('joint values (m/s, rad/s)','FontSize',14)
Save_Figure(f,['JointVelocities_' num2str(test)])

%%
f = figure;
hold on
title('Image points trajectories','FontSize',14)
set(gca,'FontSize',14)
set(gca, 'YDir','reverse')

line([px_plot(1,1) px_plot(3,1)],[px_plot(2,1) px_plot(4,1)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle',':','HandleVisibility','off')
line([px_plot(1,1) px_plot(7,1)],[px_plot(2,1) px_plot(8,1)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle',':','HandleVisibility','off')
line([px_plot(3,1) px_plot(5,1)],[px_plot(4,1) px_plot(6,1)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle',':','HandleVisibility','off')
line([px_plot(5,1) px_plot(7,1)],[px_plot(6,1) px_plot(8,1)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle',':','HandleVisibility','off')

line([px_plot(1,end) px_plot(3,end)],[px_plot(2,end) px_plot(4,end)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle','-','HandleVisibility','off')
line([px_plot(1,end) px_plot(7,end)],[px_plot(2,end) px_plot(8,end)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle','-','HandleVisibility','off')
line([px_plot(3,end) px_plot(5,end)],[px_plot(4,end) px_plot(6,end)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle','-','HandleVisibility','off')
line([px_plot(5,end) px_plot(7,end)],[px_plot(6,end) px_plot(8,end)],'LineWidth',1.5,'MarkerSize',8,'color',[0 0 0],'LineStyle','-','HandleVisibility','off')

plot(px_plot(1,:)',px_plot(2,:)','-.','LineWidth',1.5)
plot(px_plot(3,:)',px_plot(4,:)','-.','LineWidth',1.5)
plot(px_plot(5,:)',px_plot(6,:)','-.','LineWidth',1.5)
plot(px_plot(7,:)',px_plot(8,:)','-.','LineWidth',1.5)

xlabel('$u$ (pixels)','FontSize',16,'interpreter','latex')
ylabel('$v$ (pixels)','FontSize',16,'interpreter','latex')

legend('$m_1$','$m_2$','$m_3$','$m_4$','interpreter','latex','FontSize',14)

axis([1 640 1 480])

Save_Figure(f,['ImagePointsTrajectories_' num2str(test)])

%%
f = figure;
t = tiledlayout(2,2);

nexttile
grid on
hold on
title('Image feature $s_1$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,sd_plot(1:2,:)','LineWidth',1.0)
plot(t_plot,s_plot(1:2,:)','--','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('m','FontSize',12)

nexttile
grid on
hold on
title('Image feature $s_2$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,sd_plot(3:4,:)','LineWidth',1.0)
plot(t_plot,s_plot(3:4,:)','--','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('m','FontSize',12)
legend('$x^d_i$','$y^d_i$','$x_i$','$y_i$','interpreter','latex','FontSize',12)

nexttile
grid on
hold on
title('Image feature $s_3$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,sd_plot(5:6,:)','LineWidth',1.0)
plot(t_plot,s_plot(5:6,:)','--','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('m','FontSize',12)

nexttile
grid on
hold on
title('Image feature $s_4$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,sd_plot(7:8,:)','LineWidth',1.0)
plot(t_plot,s_plot(7:8,:)','--','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('m','FontSize',12)

t.Padding = 'none';
Save_Figure(f,['ImageFeatures_' num2str(test)])

%%
f = figure;
t = tiledlayout(2,2);

nexttile
grid on
hold on
title('Feature error $e_1$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,e_plot(1:2,:)','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('error (m)','FontSize',12)

nexttile
grid on
hold on
title('Feature error $e_2$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,e_plot(3:4,:)','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('error (m)','FontSize',12)
legend('$e^x_i$','$e^y_i$','interpreter','latex','FontSize',12)

nexttile
grid on
hold on
title('Feature error $e_3$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,e_plot(5:6,:)','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('error (m)','FontSize',12)

nexttile
grid on
hold on
title('Feature error $e_4$','FontSize',14,'interpreter','latex')
set(gca,'FontSize',12)
plot(t_plot,e_plot(7:8,:)','LineWidth',1.0)
xlabel('time (s)','FontSize',12)
ylabel('error (m)','FontSize',12)

t.Padding = 'none';
Save_Figure(f,['FeaturesErrors_' num2str(test)])

%%
figure
grid on
hold on
title('Camera velocity','FontSize',14)
set(gca,'FontSize',14)
plot(t_plot,vc_plot','LineWidth',1.5)

legend('$v_c^x$','$v_c^y$','$v_c^z$','$\omega_c^x$','$\omega_c^y$','$\omega_c^z$','FontSize',14,'interpreter','latex')

xlabel('time (s)','FontSize',14)
ylabel('m/s, rad/s','FontSize',14)

Save_Figure(f,['CameraVelocity_' num2str(test)])


%%

figure
hold on
grid on

Tb = wTp(qo(1:3));
T0 = Tb*pTb;
T1 = T0*T01(qo(4));
T2 = T1*T12(qo(5));
T3 = T2*T23(qo(6));
T4 = T3*T34(qo(7));
T5 = T4*T45(qo(8));
wTe = T5;
wTc = wTe*eTc;

Dibujar_Sistema_Referencia_3D(eye(4,4))
Dibujar_MM(Tb,T0,T1,T2,T3,T4,T5)
Dibujar_Camara(wTc)
Dibujar_Objeto_3D(wTo,op1,op2,op3,op4)

axis equal
view([-60 50])


figure
hold on
grid on

Tb = wTp(q(1:3));
T0 = Tb*pTb;
T1 = T0*T01(q(4));
T2 = T1*T12(q(5));
T3 = T2*T23(q(6));
T4 = T3*T34(q(7));
T5 = T4*T45(q(8));
wTe = T5;
wTc = wTe*eTc;

Dibujar_Sistema_Referencia_3D(eye(4,4))
Dibujar_MM(Tb,T0,T1,T2,T3,T4,T5)
Dibujar_Camara(wTc)
Dibujar_Objeto_3D(wTo,op1,op2,op3,op4)

axis equal
view([-60 50])



return
















%%
function [x,Z] = Proyection_3D (K,cTo,op)
    cRo = cTo(1:3,1:3);
    cto = cTo(1:3,4);
    M = [cRo cto];
    P = K*M;

    X = P*op;
    x = X(1:2)/X(3);

    cp = cTo*op;
    Z = cp(3);
end

function s = Proyection_2D (u,K)
    uc = K(1:2,3);
    f = diag(K(1:2,1:2));
    
    s = (u-uc)./f;
end

function L = Interaction_Matrix(s,Z)
    x = s(1);
    y = s(2);

    L = [-1/Z 0 x/Z x*y -(1+x^2) y;
         0 -1/Z y/Z 1+y^2 -x*y -x];
end

function Dibujar_MM (Tb,T0,T1,T2,T3,T4,T5)
    L_base = 0.471/2;
    l_base = 0.3/2;

    Dibujar_Base(Tb,L_base,l_base)
    Dibujar_Rotacional(T0)
    line([T0(1,4) T1(1,4)],[T0(2,4) T1(2,4)],[T0(3,4) T1(3,4)],'color',[0 0 0],'LineWidth',1,'LineStyle','--')
    Dibujar_Rotacional(T1)
    line([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'color',[0 0 0],'LineWidth',1,'LineStyle','--')
    Dibujar_Rotacional(T2)
    line([T2(1,4) T3(1,4)],[T2(2,4) T3(2,4)],[T2(3,4) T3(3,4)],'color',[0 0 0],'LineWidth',1,'LineStyle','--')
    Dibujar_Rotacional(T3)
    line([T3(1,4) T4(1,4)],[T3(2,4) T4(2,4)],[T3(3,4) T4(3,4)],'color',[0 0 0],'LineWidth',1,'LineStyle','--')
    Dibujar_Rotacional(T4)
    line([T4(1,4) T5(1,4)],[T4(2,4) T5(2,4)],[T4(3,4) T5(3,4)],'color',[0 0 0],'LineWidth',1,'LineStyle','--')
    Dibujar_Sistema_Referencia_3D(T5)
end


function Dibujar_Base(T,L,l)
    x = T(1,4);
    y = T(2,4);
    theta = atan2(T(2,1),T(1,1));

	Lo = L*0.3;
    lo = L*0.2;
    Tob = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
    Tblf = [1 0 L; 0 1 l; 0 0 1];
    Tbrf = [1 0 L; 0 1 -l; 0 0 1];
    Tblb = [1 0 -L; 0 1 l; 0 0 1];
    Tbrb = [1 0 -L; 0 1 -l; 0 0 1];
    
    Tolf = Tob*Tblf;
    Torf = Tob*Tbrf;
    Tolb = Tob*Tblb;
    Torb = Tob*Tbrb;
    
    p1 = Tob*[+L+Lo -l+lo 1]';
    p2 = Tob*[-L-Lo -l+lo 1]';
    p3 = Tob*[+L+Lo +l-lo 1]';
    p4 = Tob*[-L-Lo +l-lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])

    p1 = Tolf*[+Lo -lo 1]';
    p2 = Tolf*[-Lo -lo 1]';
    p3 = Tolf*[+Lo +lo 1]';
    p4 = Tolf*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])

    p1 = Torf*[+Lo -lo 1]';
    p2 = Torf*[-Lo -lo 1]';
    p3 = Torf*[+Lo +lo 1]';
    p4 = Torf*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])

    p1 = Torb*[+Lo -lo 1]';
    p2 = Torb*[-Lo -lo 1]';
    p3 = Torb*[+Lo +lo 1]';
    p4 = Torb*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    
    p1 = Tolb*[+Lo -lo 1]';
    p2 = Tolb*[-Lo -lo 1]';
    p3 = Tolb*[+Lo +lo 1]';
    p4 = Tolb*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',1,'MarkerSize',8,'color',[0 0 0])

    Dibujar_Sistema_Referencia_3D(T)
end

function Dibujar_Rotacional(T)
    S = 0.1;
    
    n = 50;
    alpha = linspace(-pi,pi,n);
    
    r = S*0.5;
    Xl = r*cos(alpha)';
    Yl = r*sin(alpha)';
    Zl = 0.5*S*ones(n,1);
    
    Xu = Xl;
    Yu = Yl;
    Zu = -Zl;
    
    AUX = [Xl Yl Zl ones(n,1)]*T';
    Xl = AUX(:,1);
    Yl = AUX(:,2);
    Zl = AUX(:,3);
    
    AUX = [Xu Yu Zu ones(n,1)]*T';
    Xu = AUX(:,1);
    Yu = AUX(:,2);
    Zu = AUX(:,3);
    
    fill3(Xl,Yl,Zl,'blue','FaceAlpha',0.0,'LineWidth',1);
    fill3(Xu,Yu,Zu,'blue','FaceAlpha',0.0,'LineWidth',1);
    
    line([Xl(13) Xu(13)],[Yl(13) Yu(13)],[Zl(13) Zu(13)],'color',[0 0 0],'linewidth',1)
    line([Xl(38) Xu(38)],[Yl(38) Yu(38)],[Zl(38) Zu(38)],'color',[0 0 0],'linewidth',1)
    
    % Dibujar_Sistema_Referencia_3D(T)
end

function Dibujar_Camara (wTc)
    l = 0.03;

    pc1 = wTc*[-l l l 1]';
    pc2 = wTc*[l l l 1]';
    pc3 = wTc*[-l -l l 1]';
    pc4 = wTc*[l -l l 1]';

    pc5 = wTc*[-l l -l 1]';
    pc6 = wTc*[l l -l 1]';
    pc7 = wTc*[-l -l -l 1]';
    pc8 = wTc*[l -l -l 1]';

    line([pc1(1) pc2(1)],[pc1(2) pc2(2)],[pc1(3) pc2(3)],'LineWidth',1,'color',[0 0 0])
    line([pc1(1) pc3(1)],[pc1(2) pc3(2)],[pc1(3) pc3(3)],'LineWidth',1,'color',[0 0 0])
    line([pc2(1) pc4(1)],[pc2(2) pc4(2)],[pc2(3) pc4(3)],'LineWidth',1,'color',[0 0 0])
    line([pc3(1) pc4(1)],[pc3(2) pc4(2)],[pc3(3) pc4(3)],'LineWidth',1,'color',[0 0 0])

    line([pc5(1) pc6(1)],[pc5(2) pc6(2)],[pc5(3) pc6(3)],'LineWidth',1,'color',[0 0 0])
    line([pc5(1) pc7(1)],[pc5(2) pc7(2)],[pc5(3) pc7(3)],'LineWidth',1,'color',[0 0 0])
    line([pc6(1) pc8(1)],[pc6(2) pc8(2)],[pc6(3) pc8(3)],'LineWidth',1,'color',[0 0 0])
    line([pc7(1) pc8(1)],[pc7(2) pc8(2)],[pc7(3) pc8(3)],'LineWidth',1,'color',[0 0 0])

    line([pc1(1) pc5(1)],[pc1(2) pc5(2)],[pc1(3) pc5(3)],'LineWidth',1,'color',[0 0 0])
    line([pc2(1) pc6(1)],[pc2(2) pc6(2)],[pc2(3) pc6(3)],'LineWidth',1,'color',[0 0 0])
    line([pc3(1) pc7(1)],[pc3(2) pc7(2)],[pc3(3) pc7(3)],'LineWidth',1,'color',[0 0 0])
    line([pc4(1) pc8(1)],[pc4(2) pc8(2)],[pc4(3) pc8(3)],'LineWidth',1,'color',[0 0 0])

    pcc1 = wTc*[-l-0.5*l l+0.5*l l+0.5*l 1]';
    pcc2 = wTc*[l+0.5*l l+0.5*l l+0.5*l 1]';
    pcc3 = wTc*[-l-0.5*l -l-0.5*l l+0.5*l 1]';
    pcc4 = wTc*[l+0.5*l -l-0.5*l l+0.5*l 1]';
    
    line([pc1(1) pcc1(1)],[pc1(2) pcc1(2)],[pc1(3) pcc1(3)],'LineWidth',1,'color',[0 0 0])
    line([pc2(1) pcc2(1)],[pc2(2) pcc2(2)],[pc2(3) pcc2(3)],'LineWidth',1,'color',[0 0 0])
    line([pc3(1) pcc3(1)],[pc3(2) pcc3(2)],[pc3(3) pcc3(3)],'LineWidth',1,'color',[0 0 0])
    line([pc4(1) pcc4(1)],[pc4(2) pcc4(2)],[pc4(3) pcc4(3)],'LineWidth',1,'color',[0 0 0])
    
    line([pcc1(1) pcc2(1)],[pcc1(2) pcc2(2)],[pcc1(3) pcc2(3)],'LineWidth',1,'color',[0 0 0])
    line([pcc1(1) pcc3(1)],[pcc1(2) pcc3(2)],[pcc1(3) pcc3(3)],'LineWidth',1,'color',[0 0 0])
    line([pcc2(1) pcc4(1)],[pcc2(2) pcc4(2)],[pcc2(3) pcc4(3)],'LineWidth',1,'color',[0 0 0])
    line([pcc3(1) pcc4(1)],[pcc3(2) pcc4(2)],[pcc3(3) pcc4(3)],'LineWidth',1,'color',[0 0 0])

    Dibujar_Sistema_Referencia_3D(wTc)
end

function Dibujar_Objeto_3D (wTo,op1,op2,op3,op4)
    wp1 = wTo*op1;
    wp2 = wTo*op2;
    wp3 = wTo*op3;
    wp4 = wTo*op4;
    plot3(wp1(1),wp1(2),wp1(3),'om','LineWidth',1,'MarkerSize',4)
    plot3(wp2(1),wp2(2),wp2(3),'om','LineWidth',1,'MarkerSize',4)
    plot3(wp3(1),wp3(2),wp3(3),'om','LineWidth',1,'MarkerSize',4)
    plot3(wp4(1),wp4(2),wp4(3),'om','LineWidth',1,'MarkerSize',4)

    Dibujar_Sistema_Referencia_3D(wTo)
end

function Dibujar_Sistema_Referencia_3D (T)
    apx = T*[0.06 0 0 1]';
    apy = T*[0 0.06 0 1]';
    apz = T*[0 0 0.06 1]';
    atb = T(1:3,4);
    
    line([atb(1) apx(1)],[atb(2) apx(2)],[atb(3) apx(3)],'color',[1 0 0],'LineWidth',2)
    line([atb(1) apy(1)],[atb(2) apy(2)],[atb(3) apy(3)],'color',[0 1 0],'LineWidth',2)
    line([atb(1) apz(1)],[atb(2) apz(2)],[atb(3) apz(3)],'color',[0 0 1],'LineWidth',2)
end

function Save_Figure (f,name)
    saveas(f,['Images/' name '.eps'],'epsc')
    saveas(f,['Images/' name '.png'])
end