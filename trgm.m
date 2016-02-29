function [ rcvPwr ] = trgm( par,carAttr,dis )
%% Two Ray Ground model uses the direct path and the reflected path between the cars.

% The received signal in two?ray ground model consists of two components: 
% the signal propagating through free space and the signal reflected off the ground.

 %%  References :
         % 1 A Channel Model for VANET Simulation System
         % 2 https://en.wikipedia.org/wiki/Two-ray_ground-reflection_model
         % 3 On the Applicability of Two-Ray Path Loss Models for Vehicular 
         %   Network Simulation
         % 4 http://www.hindawi.com/journals/ace/2010/416190/tab1/
         % 5 https://en.wikipedia.org/wiki/Autobahn

 %% Two Ray Ground Model  
 
los = sqrt(power((carAttr.height-carAttr.height),2)+dis); % line of sight
reflectedPath = sqrt(power((carAttr.height+...
                        carAttr.height),2)+dis); % reflected path 
deltaDis = abs(reflectedPath - los); % path difference
dRef = 0.75.*reflectedPath; % path of x, the part of wave before reflection from grnd
phaseDiff = exp(-1i.*(((2*pi)/carAttr.wavelength).*deltaDis));      
R = ((2*carAttr.height)./dRef) - ... % reflectivity
                ((sqrt(par.dielectricRoad-power((dis./dRef),2)))./...
                (((2*carAttr.height)./dRef) - ...
                (sqrt(par.dielectricRoad-power((dis./dRef),2)))));
t1 = sqrt(carAttr.trnsGain).*los; %LOS term
t2 = ((sqrt(carAttr.trnsGain).*R)./reflectedPath).*phaseDiff;% Reflected term
rcvPwr = -20*log((carAttr.trasnsPwr*(carAttr.wavelength/(4*pi))).*(abs(t1+t2)));
rcvPwr(isnan(rcvPwr))=0;
rcvPwr(rcvPwr>14)=-50;

end

