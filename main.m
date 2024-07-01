clear all
close all
clc

L_base = 0.471/2;
l_base = 0.3/2;

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

bTe = @(q) T01(q(1))*T12(q(2))*T23(q(3))*T34(q(4))*T45(q(5));


bot = Bot_youBot();

% Como si fuera una calibracion
focalLength = [320 320]; 
principalPoint = [240 320];
imageSize = [480 640];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

tagSize = 0.3;
X = [-tagSize/2 tagSize/2 0 1; tagSize/2 tagSize/2 0 1; tagSize/2 -tagSize/2 0 1; -tagSize/2 -tagSize/2 0 1];

%%
qd = [0 0 -pi/4 -pi/4 0]';
qa = [0 0 0 0 0]';

while norm(qd-qa)>0.01
    qa = bot.Get_Arm_Position();
    qp = 5.0*(qd-qa);

    bot.Set_Joint_Velocity(qp);
end


I = bot.Get_Image();
[id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);

Z = X*pose.T;

u = loc;
Z = Z(:,3);

% 2D proyection
s1 = Proyection_2D(u(1,:),intrinsics);
s2 = Proyection_2D(u(2,:),intrinsics);
s3 = Proyection_2D(u(3,:),intrinsics);
s4 = Proyection_2D(u(4,:),intrinsics);
s = [s1 s2 s3 s4]';

sd = s;
pixeld = reshape(u',8,1);


%%
S = 20;

% pixeld = [421.7461  247.8589  423.8410   99.9276  275.8566  100.1249  275.9145  247.9473]';
% sd =  [0.3150    0.0578    0.3197   -0.3908   -0.1286   -0.3911   -0.1285    0.0558]';

% od = rad2deg(double(orientation));
% R = (rotz(od(3))*roty(od(2))*rotx(od(1)))';

kc = 1.4;
lambda = 3;
qd = [0 0 0 0 0 -pi/4 -pi/4 0]';

M_plot = [];
vc_plot = [];
s_plot = [];
sd_plot = [];
qp_plot = [];
e_plot = [];

px_plot =[];
pxd_plot =[];

e_plot = [e_plot zeros(8,1)];
qp = zeros(8,1);

tic;
while toc<=S
    I = bot.Get_Image();

    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    
    if ~isempty(id)
        pb = bot.Get_Platform_Pose();
        qa = bot.Get_Arm_Position();
    
        q = [pb; qa];

        T1 = wTp(q(1:3))*pTb*T01(q(4));
        T2 = T1*T12(q(5));
        T3 = T2*T23(q(6));
        T4 = T3*T34(q(7));
        T5 = T4*T45(q(8));
        
        J = [Jacob(q); [0 0 0]' [0 0 0]' [0 0 1]' [0 0 1]' T1(1:3,3) T2(1:3,3) T3(1:3,3) T4(1:3,3)];


        wTe = T5;
        % eTc = [0 -1 0 0.05; 1 0 0 0; 0 0 1 -0.08; 0 0 0 1];
        eTc = [0 -1 0 0.05; 1 0 0 0; 0 0 1 -0.08; 0 0 0 1];
        wTc = wTe*eTc;
        cTw = inv(wTc);

        t = cTw(1:3,4);
        R = cTw(1:3,1:3);

        St = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
        V = [R St*R; zeros(3,3) R];
        %V = [R -R*skew(-R'*t); zeros(3,3) R];


        Z = X*pose.T;

        u = loc;
        Z = Z(:,3);

        % 2D proyection
        s1 = Proyection_2D(u(1,:),intrinsics);
        s2 = Proyection_2D(u(2,:),intrinsics);
        s3 = Proyection_2D(u(3,:),intrinsics);
        s4 = Proyection_2D(u(4,:),intrinsics);
        s = [s1 s2 s3 s4]';

        L1 = Interaction_Matrix(s1,Z(1));
        L2 = Interaction_Matrix(s2,Z(2));
        L3 = Interaction_Matrix(s3,Z(3));
        L4 = Interaction_Matrix(s4,Z(4));
        L = [L1; L2; L3; L4];

        e = s-sd;
        
        q0 = Get_q0(q);
        
        % M = L*V;
        % vc = -lambda*(inv(M'*M)*M')*e;

        g = 9.81;
        m1 = 1.3;
        m2 = 1.3;
        m3 = 2.0;
        l1 = 0.155;
        l2 = 0.1348;
        l3 = 0.1936;
    
        q5 = q(5)+pi/2;
        q6 = q(6);
        q7 = q(7);

        qg = [0 0 0 0 g*(m3*(l2*cos(q5 + q6) + l1*cos(q5) + (l3*cos(q5 + q6 + q7))/2) + m2*((l2*cos(q5 + q6))/2 + l1*cos(q5)) + (l1*m1*cos(q5))/2) g*(m3*(l2*cos(q5 + q6) + (l3*cos(q5 + q6 + q7))/2) + (l2*m2*cos(q5 + q6))/2) (g*l3*m3*cos(q5 + q6 + q7))/2 0]';
        
        %vc = -lambda*pinv(L*V)*e - .2*pinv(L*V)*(sum(e_plot,2)) - .25 * pinv(L*V)*(10*(e - e_plot(:,end)));
        vc = -lambda*pinv(L*V)*e - .25 * pinv(L*V)*(10*(e - e_plot(:,end)));

        Ji = J'*inv(J*J' + .2*eye(6));
        
        qp = Ji* vc + (eye(8,8) - pinv(J)*J)*(kc*q0) + .00945*qg;

        % % qp = J'*diag([3.0 3.0 3.0 1.5 1.5 1.5])*vc  + (eye(8,8) - Ji*J)*(kc*(qd-q));
        % % qp = J'*diag([3.0 3.0 3.0 1.5 1.5 1.5])*vc  + (eye(8,8) - Ji*J)*(kc*q0);

        M_plot = [M_plot sqrt(det(J*J'))];
        s_plot = [s_plot s];
        vc_plot = [vc_plot vc];
        qp_plot = [qp_plot qp];
        e_plot = [e_plot e];

        sd_plot = [sd_plot sd];
        px_plot = [px_plot reshape(u',8,1)];
        pxd_plot = [pxd_plot pixeld];

%         figure(1)
%         subplot(2,2,1)
%         PD = reshape(pixeld,2,4);
%         P = reshape(u',2,4);
%         cla
%         for i=1:4
%             hold on
%             grid on
%             plot(PD(1,i),PD(2,i),'b*')
%             plot(P(1,i),P(2,i),'r*')
%             axis([250,450,100,320])
%         end
% 
%         subplot(2,2,2)
%         plot(M_plot)
%         subplot(2,2,3)
%         plot(e_plot')
%         subplot(2,2,4)
%         plot(vc_plot')
% 
%         drawnow

        % lambda = 0.1;
        % qp = -lambda*(L*V*J)'*e  + (eye(8,8) - Ji*J)*(kc*(qd-q))



        %% base
	    u_base = qp(1:3);
    
	    alpha = pb(3) + pi/4;
	    v_base = [sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) -(L_base+l_base); ...
             sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) (L_base+l_base); ...
             sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) -(L_base+l_base); ...
             sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) (L_base+l_base)]*u_base;
    
        %% arm
        qp_arm = qp(4:end);
    
        bot.Set_Joint_Velocity(qp_arm);
        bot.Set_Platform_Velocity(v_base);


        % % Just drawings
        % I = insertShape(I,"FilledCircle",[loc(1,:) 5],"Color","red","Opacity",1);
        % I = insertShape(I,"FilledCircle",[loc(2,:) 5],"Color","green","Opacity",1);
        % I = insertShape(I,"FilledCircle",[loc(3,:) 5],"Color","blue","Opacity",1);
        % I = insertShape(I,"FilledCircle",[loc(4,:) 5],"Color","cyan","Opacity",1);
    end

    % cla
    % imshow(I)
    % drawnow
end

%%

close all

figure
grid on
hold on
title('Manipulability','FontSize',14, 'interpreter','latex')
plot(M_plot)
xlabel('time (s)','FontSize',14,'interpreter','latex')
ylabel('Manipulability','FontSize',14,'interpreter','latex')
xlim([0 20])

% figure
% grid on
% hold on
% title('features servoing')
% plot(s_plot')
% plot(sd_plot')

count = 1;
for i=1:2:8
    figure
    grid on
    hold on
    title(['features servoing s ' num2str(count)])
    plot(s_plot(i:i+1,:)','LineWidth',3)
    plot(sd_plot(i:i+1,:)')
    
    legend('x','y','xd','yd')
    count = count + 1;
end



%%
figure
SD = reshape(sd,2,4);
S = reshape(s,2,4);

for i=1:4
    hold on
    grid on
    plot(SD(1,i),SD(2,i),'*')
    plot(S(1,i),S(2,i),'*')
end

title('plano')
legend('des1','act1','des2','act2','des3','act3','des4','act4')

figure
PD = reshape(pixeld,2,4);
P = reshape(u',2,4);

for i=1:4
    hold on
    grid on
    plot(PD(1,i),PD(2,i),'*')
    plot(P(1,i),P(2,i),'*')
end

title('imagen')
legend('des1','act1','des2','act2','des3','act3','des4','act4')
%%

% figure
% grid on
% hold on
% title('features servoing u')
% plot(px_plot','LineWidth',3)
% plot(pxd_plot')
% 
% legend('x1','x2','x3','x4',...
%        'xd1','xd2','xd3','xd4')



count = 1;

for i=1:2:8
    %figure
    subplot(2,2,count)
    grid on
    hold on
    title(['Image feature $s_', num2str(count),'$'],'FontSize',14,'interpreter','latex')
    plot(sd_plot(i:i+1,:)','LineWidth',1.0)
    plot(s_plot(i:i+1,:)','--','LineWidth',1.0)
    xlim([0 20])
    xlabel('time (s)','FontSize',14,'interpreter','latex')
    ylabel('m','FontSize',14,'interpreter','latex')
    legend('x','y','xd','yd','FontSize',8,'interpreter','latex')
    count = count + 1;
end





figure
grid on
hold on
title('e features servoing')
plot(e_plot')

figure
grid on
hold on
title('vc servoing')
plot(vc_plot')

figure
grid on
hold on
title('qp mobile manipulator')
plot(qp_plot')
legend('q1','q2','q3','q4','q5','q6','q7','q8')

%%
bot.Stop_Simulation();


function s = Proyection_2D (u,intrinsics)
    uc = intrinsics.PrincipalPoint;
    f = intrinsics.FocalLength;
    
    s = (u-uc)./f;


    % s = [0 0];
    % s(1) = (u(1)-uc(1));
    % s(2) = (u(2)-uc(2));
end

function L = Interaction_Matrix(s,Z)
    x = s(1);
    y = s(2);

    L = [-1/Z 0 x/Z x*y -(1+x^2) y;
         0 -1/Z y/Z 1+y^2 -x*y -x];
end
