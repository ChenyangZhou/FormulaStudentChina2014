clear;
%for front and rear suspension rate calculation%
%Unit:lb,inch,Hz%
%parameters define%
Wf=3.2;%front suspension Frequency
Wr=2.9;%rear suspension Frequency

Wsf=286;%front spring mass
Wsr=385;%rear spring mass
Ws=671;%total spring mass
Wuf=26.18;%front unspring mass
Wur=28.82;%rear unspring mass
Wt=726;%total mass
Kt=857;%tire rate
h=274.6;%CG height

LRf=0.72;%front linkage ratio
LRr=0.74;%rear linkage ratio

Zf=3.03;%front roll center height
Zr=2.36;%rear roll center height

RLf=8.7;%tire static loaded radius;
RLr=8.7;

Fltd=0.50;%front load transfer distribution

Tf=48.03;%Track;
Tr=46.46;
Ts=34.38;%spring track;

RG=1.6;% desired roll gradient

Krf=4*pi^2*Wf^2*(Wsf/2)/386.4;% ride rate
Krr=4*pi^2*Wr^2*(Wsr/2)/386.4;

Kwf=Krf*Kt/(Kt-Krf);%wheel rate
Kwr=Krr*Kt/(Kt-Krr);

Ksf=Kwf/LRf^2;% spring rate
Ksr=Kwr/LRr^2;
%Ksf=225;
%Ksr=350;

hs=(Wt*h-Wuf*RLf-Wur*RLr)/Ws;%spring mass CG height
as=Wsf/Ws;%spring mass weight distribution

Hrm=hs-(Zf+(Zr-Zf)*(1-as));%rolling moment lever arm
MAy=Hrm*Ws/12;%rolling moment per g lateral acceleration

Kp=MAy/RG;%required roll rate

Kpsf=Krf*Tf^2/1375;%front srping roll rate
Kpsr=Krr*Tr^2/1375;%rear spring roll rate

Kps=Kpsf+Kpsr;% total available roll rate
Kpb=Kp-Kps;%stabilizer bars provide;

TLTAy=Wt*h/((Tf+Tr)/2);%total load transfer
FLTAy=Fltd*TLTAy;%front load transfer

Kpf=(FLTAy-Wsf*Zf/Tf-Wuf*RLf/Tf)*Tf/(12*RG);%front roll stiffness
Kpbf=Kpf-Kpsf;%front bar provide
Kpbr=Kp-Kpf-Kpsr;%rear bar provide 

%Kbf=Kpbf/(LBf^2);
%Kbf=Kpfr/(LBr^2);

ideal_front_spring=Ksf%*175/1000
ideal_rear_srping=Ksr%*175/1000
