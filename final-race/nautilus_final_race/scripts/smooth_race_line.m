clc
clear
close all
%%
c1 = [12.06 -10.42; 14.75 -10.5];
c2 = [0.82 -10.65; 1.97, -13.09];
c3 = [-19.69 -27.6; -17.94, -29.6];
c4 = [-23.33 -22.10; -24.71, -20.17];
c5 = [-16.73, -6.24; -16.91, -3.62];
start = [0.0, -1.25; 0.0, 1.25];

M = csvread('race.csv');
M = [M ; 0 0 0];
x = M(:,1);
y = M(:,2);
theta = M(:,3);
p = [x,y];

K_A = f_K_A(4009,203,p,390);

% K_O = f(4009,4195,p);
% O_A = f(1,203,p);

K_O = K_A(1:187,:);
O_A = K_A(188:end,:);

A_B = p(204:401,:);

% %B_C = f(402,917,p);
B_C = p(402:917,:);
C_D = p(918:1372,:);

B_C1 = p(402:660,:);
C1_M = f(661,989,p);
M_D =  p(990:1372,:);

D_E = f(1373,1450,p);
E_F = p(1451:1663,:);
F_G = f(1664,2361,p);
G_H = p(2362:2855,:);
H_I = f(2856,3234,p);
I_J = p(3235:3549,:);
J_K = f(3550,4008,p);


% new = [O_A;A_B;B_C;C_D;D_E;E_F;F_G;G_H;H_I;I_J;J_K;K_O];
new = [O_A;A_B;B_C1;C1_M;M_D;D_E;E_F;F_G;G_H;H_I;I_J;J_K;K_O];
new_point = [new theta];
csvwrite('new_race_3.csv',new_point);
%%
d1 = cal_dist(F_G)
%%
SMx = smooth(new(:,1),101);
SMy = smooth(new(:,2),101);
SMnew = [SMx,SMy,theta];

csvwrite('new_race_smooth_100.csv',SMnew);

% figure 
% plot(SMx,SMy,'r','linewidth',3)
% hold on
% plot(new(:,1),new(:,2),'linewidth',3)
%%
figure
plot(SMnew(:,1),SMnew(:,2),'r','linewidth',3)
grid on
hold on 
plot(x,y,'linewidth',3);

hold on 
plot(c1(:,1),c1(:,2),'k','linewidth',2)
text(c1(2,1),c1(2,2),' C1')
hold on 
plot(c2(:,1),c2(:,2),'k','linewidth',2)
text(c2(2,1),c2(2,2),' C2')
hold on 
plot(c3(:,1),c3(:,2),'k','linewidth',2)
text(c3(2,1),c3(2,2),' C3')
hold on 
plot(c4(:,1),c4(:,2),'k','linewidth',2)
text(c4(2,1),c4(2,2),' C4')
hold on 
plot(c5(:,1),c5(:,2),'k','linewidth',2)
text(c5(2,1),c5(2,2),' C5')
hold on 
plot(start(:,1),start(:,2),'k','linewidth',2)
text(start(2,1),start(2,2),' START')

hold on
plot(O_A(1,1),O_A(1,2),'k*')
hold on
plot(A_B(1,1),A_B(1,2),'k*')
text(A_B(1,1),A_B(1,2)+1,'A203')
hold on
plot(B_C(1,1),B_C(1,2),'k*')
text(B_C(1,1)+1,B_C(1,2),'B402')
hold on
plot(M_D(1,1),M_D(1,2),'k*')
text(M_D(1,1)+1,M_D(1,2),'M989')
%text(C_D(1,1)+1,C_D(1,2),'C')
hold on
plot(D_E(1,1),D_E(1,2),'k*')
text(D_E(1,1)+1,D_E(1,2),'D1373')
hold on
plot(E_F(1,1),E_F(1,2),'k*')
text(E_F(1,1)+1,E_F(1,2),'E1450')
hold on
plot(F_G(1,1),F_G(1,2),'k*')
text(F_G(1,1),F_G(1,2)+1,'F1664')
hold on
plot(G_H(1,1),G_H(1,2),'k*')
text(G_H(1,1),G_H(1,2)-1,'G2361')
hold on
plot(H_I(1,1),H_I(1,2),'k*')
text(H_I(1,1)+1,H_I(1,2),'H2856')
hold on
plot(I_J(1,1),I_J(1,2),'k*')
text(I_J(1,1)+1,I_J(1,2),'I3234')
hold on
plot(J_K(1,1),J_K(1,2),'k*')
text(J_K(1,1)+1,J_K(1,2),'J3550')
hold on
plot(K_A(1,1),K_A(1,2),'k*')
text(K_A(1,1),K_A(1,2)+1,'K4008')

legend('Smoothed Strighten Raceline','Original Raceline','Location','northwest')
%%
function p_new = f(a,b,p)
p_a = p(a,:);
p_b = p(b,:);
x = linspace(p_a(1),p_b(1),b-a+1);
y = linspace(p_a(2),p_b(2),b-a+1);
p_new = [x;y]';
end
function K_A = f_K_A(a,b,p,num)
p_a = p(a,:);
p_b = p(b,:);
x = linspace(p_a(1),p_b(1),num);
y = linspace(p_a(2),p_b(2),num);
K_A = [x;y]';
end
function distance = cal_dist(section)
distance = 0;
for i = 1:length(section)-1
    d = sqrt( (section(i,1)-section(i+1,1))^2 + (section(i,2)-section(i+1,2))^2 );
    distance = distance + d;
end
end


