clear;
%for front and rear suspension rate calculation%
%Unit:lb,inch,Hz%
%parameters define%
Wf=1.9;%front suspension Frequency
Wr=1.7;%rear suspension Frequency

Wsf=979.5;%front spring mass
Wsr=1163;%rear spring mass
Ws=2142.5;%total spring mass
Wuf=170;%front unspring mass
Wur=250;%rear unspring mass
Wt=2562.5;%total mass
Kt=2000;%tire rate
h=15;%CG height

LRf=0.728;%front linkage ratio
LRr=0.9615;%rear linkage ratio

Zf=3.1;%front roll center height
Zr=6.8;%rear roll center height

RLf=11.25;%tire static loaded radius;
RLr=12.75;

Fltd=0.4985;%front load transfer distribution

Tf=65.58;%Track;
Tr=64.5;
Ts=47.1;%spring track;

RG=1.6;% desired roll gradient

Krf=4*pi^2*Wf^2*(Wsf/2)/386.4;% ride rate
Krr=4*pi^2*Wr^2*(Wsr/2)/386.4;

Kwf=Krf*Kt/(Kt-Krf);%wheel rate
Kwr=Krr*Kt/(Kt-Krr);

Ksf=Kwf/LRf^2;% spring rate
Ksr=Kwr/LRr^2;

hs=(Wt*h-Wuf*RLf-Wur*RLr)/Ws;%spring mass CG height
as=Wsf/Ws;%spring mass weight distribution

Hrm=hs-(Zf+(Zr-Zf)*(1-as));%rolling moment lever arm
MAy=Hrm*Ws/12;%rolling moment per g lateral acceleration

Kp=MAy/RG;%required roll rate

Kpsf=Krf*Tf^2/1375;%front srping roll rate
Kpsr=(Kwr*Ts^2)*(Kt*Tr^2)/(1375*(Kwr*Ts^2+Kt*Tr^2));%rear spring roll rate

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
