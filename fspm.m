function [ recievedPwr ] = fspm(globl,carAttr,dis)
%Free space propagation model

recievedPwr = 10*log10(((10^(carAttr.trasnsPwr/10))*(10^(carAttr.trnsGain/10))*...
    (10^(carAttr.recpGain/10))).*(power((globl.c./((4*pi*carAttr.f).*dis)),2))); 
recievedPwr(recievedPwr==Inf)=-50;
end

