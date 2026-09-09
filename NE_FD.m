function state_dot = NE_FD(state, tau)
%#codegen
% Body-frame Newton-Euler forward dynamics -- 3-DOF RRR arm
% Parameters are HARDCODED here to match Simscape blocks exactly.
q=state(1:3); dq=state(4:6);

l1=0.50;l2=0.40;l3=0.30;
lc1=l1/2;lc2=l2/2;lc3=l3/2;
m1=2.0;m2=1.5;m3=1.0;
r1=0.060;r2=0.05;r3=0.040;
g_acc=9.81;%9.81
zcap=[0;0;1];
P_10=[0;0;0];P_21=[l1;0;0];P_32=[l2;0;0];
Pc_1=[lc1;0;0];Pc_2=[lc2;0;0];Pc_3=[lc3;0;0];
Ic1=diag([m1*r1^2/2,m1*(3*r1^2+l1^2)/12,m1*(3*r1^2+l1^2)/12]);
Ic2=diag([m2*r2^2/2,m2*(3*r2^2+l2^2)/12,m2*(3*r2^2+l2^2)/12]);
Ic3=diag([m3*r3^2/2,m3*(3*r3^2+l3^2)/12,m3*(3*r3^2+l3^2)/12]);
% Ic1 = diag([0.00360000, 0.04346667, 0.04346667]);
% Ic2 = diag([0.00187500, 0.02093750, 0.02093750]);
% Ic3 = diag([0.00080000, 0.00790000, 0.00790000]);

% 4 NE calls: bias b, then M columns (gravity off, dq=0)
b=ne_inv(q,dq,[0;0;0],g_acc,m1,m2,m3,Ic1,Ic2,Ic3,P_10,P_21,P_32,Pc_1,Pc_2,Pc_3,zcap);
dq0=[0;0;0];
M1=ne_inv(q,dq0,[1;0;0],0,m1,m2,m3,Ic1,Ic2,Ic3,P_10,P_21,P_32,Pc_1,Pc_2,Pc_3,zcap);
M2=ne_inv(q,dq0,[0;1;0],0,m1,m2,m3,Ic1,Ic2,Ic3,P_10,P_21,P_32,Pc_1,Pc_2,Pc_3,zcap);
M3=ne_inv(q,dq0,[0;0;1],0,m1,m2,m3,Ic1,Ic2,Ic3,P_10,P_21,P_32,Pc_1,Pc_2,Pc_3,zcap);
M=[M1,M2,M3];
ddq=M\(tau-b);
state_dot=[dq;ddq];
end

function tau_out=ne_inv(q,dq,ddq,g_acc,m1,m2,m3,Ic1,Ic2,Ic3,P_10,P_21,P_32,Pc_1,Pc_2,Pc_3,zcap)
% Full outward pass (links 1,2,3) then inward pass (links 3,2,1)
th1=q(1);th2=q(2);th3=q(3);
d1=dq(1);d2=dq(2);d3=dq(3);
dd1=ddq(1);dd2=ddq(2);dd3=ddq(3);
om0=[0;0;0];omd0=[0;0;0];v0=[0;0;0];vd0=[0;g_acc;0];
% --- Outward: Link 1 ---
R01=[cos(th1),-sin(th1),0;sin(th1),cos(th1),0;0,0,1];R10=R01.';
om1=R10*om0+d1*zcap;
omd1=R10*omd0+R10*tld(om0)*d1*zcap+dd1*zcap;
v1=R10*(v0+tld(om0)*P_10);
vd1=R10*(vd0+tld(omd0)*P_10+tld(om0)*(tld(om0)*P_10));
vcd1=vd1+tld(omd1)*Pc_1+tld(om1)*(tld(om1)*Pc_1);
F1=m1*vcd1;T1=Ic1*omd1+cross(om1,Ic1*om1);
% --- Outward: Link 2 ---
R12=[cos(th2),-sin(th2),0;sin(th2),cos(th2),0;0,0,1];R21=R12.';
om2=R21*om1+d2*zcap;
% omd2=R21*omd1+R21*tld(om1)*d2*zcap+dd2*zcap;
omd2 = R21*omd1 + tld(R21*om1)*(d2*zcap) + dd2*zcap;
v2=R21*(v1+tld(om1)*P_21);
vd2=R21*(vd1+tld(omd1)*P_21+tld(om1)*(tld(om1)*P_21));
vcd2=vd2+tld(omd2)*Pc_2+tld(om2)*(tld(om2)*Pc_2);
F2=m2*vcd2;T2=Ic2*omd2+cross(om2,Ic2*om2);
% --- Outward: Link 3 ---
R23=[cos(th3),-sin(th3),0;sin(th3),cos(th3),0;0,0,1];R32=R23.';
om3=R32*om2+d3*zcap;
% omd3=R32*omd2+R32*tld(om2)*d3*zcap+dd3*zcap;
omd3=R32*omd2+tld(R32*om2)*d3*zcap+dd3*zcap;
vd3=R32*(vd2+tld(omd2)*P_32+tld(om2)*(tld(om2)*P_32));
vcd3=vd3+tld(omd3)*Pc_3+tld(om3)*(tld(om3)*Pc_3);
F3=m3*vcd3;T3=Ic3*omd3+cross(om3,Ic3*om3);
% --- Inward: 3→2→1 ---
f3=F3;t3=T3+tld(Pc_3)*F3;tau3=t3(3);
f2=F2+R23*f3;t2=T2+R23*t3+tld(Pc_2)*F2+tld(P_32)*R23*f3;tau2=t2(3);
f1=F1+R12*f2;t1=T1+R12*t2+tld(Pc_1)*F1+tld(P_21)*R12*f2;tau1=t1(3);
tau_out=[tau1;tau2;tau3];
end

function S=tld(v)
S=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
end