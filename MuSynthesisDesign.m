
clear
%----------------Uncertainty weighting function----------------
W1=;
W2=;
delta1 = ultidyn('delta1',[1 1]);
delta2 = ultidyn('delta2',[1 1]);
unDelta=append(1+W1*delta1,1+W2*delta2);
%----------------Actuator----------------
delta1 = ultidyn('delta3',[1 1]);
delta2 = ultidyn('delta4',[1 1]);
W3=;%Uncertainty weighting function of actuator 
Wactuator1=tf(1,[0.5 1])*(1+W3*delta1);
Wactuator2=tf(1,[0.5 1])*(1+W3*delta2);
Wa=append(Wactuator1,Wactuator2);
%----------------Given disturbance weighting function--------------------
Wdshape=;
%---------------Given noise weighting function--------------
Wnoise=;
%---------------Given weighting functions---------------
Wp=;%Performance weighting function
Wu=;%Control weighting function
%---------------Given a reference model-------------------
Wmod=;
%%
%Given the controlled system
Ap=;
Bp=;
Ep=;
Cp=;
Dp=;

Wssr=[1 0;0 ss(1)];%Ignore this item

[RowD,ColD]=size(Dp);
[~,ColE]=size(Ep);
G_Add=unDelta*ss(Ap,[Bp Ep],Cp,zeros(RowD,ColD+ColE));

inputs={'cmd','disti','noise','control'};
outputs={'e_Wp','control_Wu','cmd','ymeasure'};
Wdshape.Inputname='disti';Wdshape.Outputname={'disto'};
G_Add.Inputname={'Vp(1)';'Vp(2)';'disto'};G_Add.Outputname='yout';
Wa.Inputname='control';Wa.Outputname='Vp';
Wssr.Inputname='yout';Wssr.Outputname='yout_delay';
Wnoise.Inputname='noise';Wnoise.Outputname='yout_noise';
Wp.Inputname='eclean';Wp.Outputname='e_Wp';
Wu.Inputname='control';Wu.Outputname='control_Wu';
Wmod.Inputname='cmd';Wmod.Outputname='ref';
sum1=sumblk('%ymeasure=yout_delay+yout_noise',{'ymeasure(1)','ymeasure(2)'});
sum3=sumblk('eclean=yout-%ref',{'ref(1)','ref(2)'});
Pmodel=connect(G_Add,Wmod,Wssr,Wnoise,Wdshape,Wa,Wp,Wu,sum1,sum3,inputs,outputs);
%%
%μ-synthesis
ncontrol=;%number of control variables
nmeasure=;%number of output variables
[kmu,bnd] = musyn(Pmodel,nmeasure,ncontrol);
%kmu is the designed controller.
%To get the controller, you can use : kmu.A  kmu.B  kmu.C  kmu.D

