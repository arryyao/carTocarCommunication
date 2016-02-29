
clear all;
clc ;

%%  Constant parameters common for all the objects 

globl.c = 300000000; % speed of light 
parameters.v0Lane1 =  80 / 3.6; % meters per second, low-speed lane
parameters.v0Lane2 = 120 / 3.6; % meters per second, medium-speed lane
parameters.v0Lane3 = 180 / 3.6; % meters per second, high-speed lane
parameters.start = 15;    % Starting position
parameters.gap = 50;  % maximum gap 50 for a leading vehicle
parameters.dt = 0.3;  % time interval
parameters.s0 = 2; % minimum bumper to bumper distance to the front vehicle
parameters.delta = 4.0; % acceleration exponent
parameters.Lanes = 3;   % Number of Lanes
parameters.LengthofCar = 5.0;
parameters.dielectricRoad = 4; %(4-8) road made of aliphatic concrete

% car attributes
attrOfCar.txnPwr = 33; %in dBm scale
attrOfCar.txnGain = 10.3; %in dB scale
attrOfCar.rcvrGain = 10.3; %in dB scale
attrOfCar.f = 5900000000; % 5.9 Ghz
attrOfCar.B = 10000000;    % Bandwidth of the channel
attrOfCar.recvThreshold = -90; % in dBm scale
attrOfCar.txnRange = 1000; % communication range in metres
attrOfCar.velocity = 120/3.6;
attrOfCar.lengthOfCar = 5.0;
attrOfCar.wavelength = 0.051; % in meters
attrOfCar.height = 1.895; % in metres

noOfLanes = 3; 
noOfCars = 30;
Roadlength = 2500;    % total Road length
simTime = 10;  % Standard value : 10

% Road side unit attributes
rsuAttr.firstDis = 2100;
rsuAttr.numberOfRsu = round(Roadlength/(Roadlength-rsuAttr.firstDis)); % number of road side units
rsuAttr.txnPwr = 33; % Transmission power dBm
rsuAttr.txnGain = 20.3; %  Transmission gain dB
rsuAttr.rcvrGain = 20.3; %  Receiver gain dB
rsuAttr.recvThreshold = -40; % Minimum threshold of received power in dBm
rsuAttr.f = 5900000000; % frequency of transmission 5.9 Ghz

cars(noOfLanes,noOfCars) = car; % neighbours for car 
carsbi1(noOfLanes,noOfCars) = car; 
cars2(noOfLanes,noOfCars) = car;
carsbi2(noOfLanes,noOfCars) = car; 
% number of car objects created  
% noOfLanes x noOfCars is created 
% The carID, Simulation number and lane are assigned to every cell in this matrix 
rsus(rsuAttr.numberOfRsu) = rsu;  %% attributes of RSU for 1 x 2 matrix assigned 

postn = zeros(noOfLanes,noOfCars); %% initialising the car postns
leadingCarPost = zeros(noOfLanes,noOfCars); 
% initialising the leading car postns
dynDisBwCars = zeros(noOfLanes,noOfCars); 
% dynamic distance between cars 
velocity = zeros(noOfLanes,noOfCars); 
% initializing the velocity of cars 
changeInVelocity = zeros(noOfLanes,noOfCars); 
% initializing the change in velocity of cars 



%% Initialize the cars on the road

% Initialize velocity
velocity(1,:)=parameters.v0Lane1;
velocity(2,:)=parameters.v0Lane2;
velocity(3,:)=parameters.v0Lane3;

% Initialize the postns of cars

postn = [50 100 150 200 250 300 350 400 450 500 550 600 650 700 750 800 850 ...
            900 950 1000 1050 1100 1150 1200 1250 1300 1350 1400 1450 1500  ;
            50 100 150 200 250 300 350 400 450 500 550 600 650 700 750 800 850 ...
            900 950 1000 1050 1100 1150 1200 1250 1300 1350 1400 1450 1500  ;
            50 100 150 200 250 300 350 400 450 500 550 600 650 700 750 800 850 ...
            900 950 1000 1050 1100 1150 1200 1250 1300 1350 1400 1450 1500];
  
 
clearvars indx0;
%% Find constant values for Communication model (cars + rsu)


%% Simulation of the cars on Road based on Communication model

for indx = 1:simTime
    
    for laneIndx = 1 : noOfLanes
     
     for indx2 = 1:noOfCars
         
         %% Antenna
         
         dis = postn - postn(laneIndx,indx2); % uni-directional antenna
         dis(dis<0)=0;
         dis2 = abs(postn -postn(laneIndx,indx2)); %bi-directional antennas
         dis(dis>attrOfCar.txnRange)=0;
         dis2(dis2>attrOfCar.txnRange)=0;
         
        %% Free space propagation model

         % Call the fspm function to get the received power.
         
       %  For uni-directional antenna
	   
         [ngbrLane,ngbrId] = find((fspm(globl,attrOfCar,dis)>=...
             attrOfCar.recvThreshold)==1); 
         ngbrId = ngbrId(ngbrId~=indx2);
         ngbrLane = ngbrLane(1:length(ngbrId));
         
         for indx3 = 1:length(ngbrId)
             cars(laneIndx,indx2).lane(end+1) = ngbrLane(indx3);
             cars(laneIndx,indx2).carID(end+1) = ngbrId(indx3);
             cars(laneIndx,indx2).simulationNbr(end+1) = indx;
         end      
         cars(laneIndx,indx2).dist(end+1) = postn(laneIndx,indx2);
         clearvars ngbrLane; clearvars ngbrId;
         
       %  For bi-directional antenna
	   
         [ngbrLane,ngbrId] = find((fspm(globl,attrOfCar,dis2)>=...
             attrOfCar.recvThreshold)==1); 
         ngbrId = ngbrId(ngbrId~=indx2);
         ngbrLane = ngbrLane(1:length(ngbrId));
         
         for indx3 = 1:length(ngbrId)
             carsbi1(laneIndx,indx2).lane(end+1) = ngbrLane(indx3);
             carsbi1(laneIndx,indx2).carID(end+1) = ngbrId(indx3);
             carsbi1(laneIndx,indx2).simulationNbr(end+1) = indx;
         end       
         carsbi1(laneIndx,indx2).dist(end+1) = postn(laneIndx,indx2);
         clearvars ngbrLane; clearvars ngbrId;
         
         %% Two ray Ground Model
          
         [ngbrLane,ngbrId] = find(((trgm(parameters,attrOfCar,dis))>=attrOfCar.recvThreshold)==1); 
         ngbrId = ngbrId(ngbrId~=indx2);
         ngbrLane = ngbrLane(1:length(ngbrId));
         
         % uni-directional antenna
		 
         for indx3 = 1:length(ngbrId)
             cars2(laneIndx,indx2).lane(end+1) = ngbrLane(indx3);
             cars2(laneIndx,indx2).carID(end+1) = ngbrId(indx3);
             cars2(laneIndx,indx2).simulationNbr(end+1) = indx;
         end      
         cars2(laneIndx,indx2).dist(end+1) = postn(laneIndx,indx2);
         clearvars ngbrLane; clearvars ngbrId;
         
         % bi-directional antenna
		 
         [ngbrLane,ngbrId] = find(((trgm(parameters,attrOfCar,dis2))...
             >=attrOfCar.recvThreshold)==1); 
         ngbrId = ngbrId(ngbrId~=indx2);
         ngbrLane = ngbrLane(1:length(ngbrId));
         
         for indx3 = 1:length(ngbrId)
             carsbi2(laneIndx,indx2).lane(end+1) = ngbrLane(indx3);
             carsbi2(laneIndx,indx2).carID(end+1) = ngbrId(indx3);
             carsbi2(laneIndx,indx2).simulationNbr(end+1) = indx;
         end    
         carsbi2(laneIndx,indx2).dist(end+1) = postn(laneIndx,indx2);
         clearvars ngbrLane; clearvars ngbrId;
         
         %% Leading car postns
         
         postDis = postn(1,indx2);
         diff = postn - postDis;
         
         if(diff(laneIndx,1) == 0) % update for first car
             minVal = min(diff(diff~=0));
             if(minVal>2 && minVal<50)
                 [u,v] = find(minVal==diff); %%% u is lane , v is postn of the leading car
                 leadingCarPost(:,indx2) = postn(:,v(1)); 
             end
             
         else if(diff(noOfLanes,noOfCars) == 0) %update for last car
                 % No leading car for last car on road
                 clearvars diff; clearvars u, clearvars v, clearvars minVal;
                 clearvars postDis;
             else
                 minVal = min(diff(diff>0));
                 if(minVal~=0)
                     if(minVal>2 && minVal<50)
                     [u,v] = find(minVal==diff);
                     leadingCarPost(:,indx2) = postn(:,v(1));
                    end 
                 end
                               
             end
         end
         
         %% Dynamic distance between following and leading cars

          curVel = velocity(laneIndx,indx2);
          deltaV = curVel - curVel;
          leadPos = leadingCarPost(1,indx2);
          if(leadPos~=0)
              [m,n] = find(leadPos==postn);
              if(n~=0)
                  leadingCarVel = velocity(laneIndx,n(1));
                  deltaV = abs(curVel - leadingCarVel); %% maybe abs
              end
          end
          
          T = randomNumberGern(0.8,2);
          
          if T >= 1.5
               a = 1;
               b = 1;
          else
              a = randomNumberGern(1.5,2);
              b = randomNumberGern(1.5,2);
          end
          
          dynDisBwCars(laneIndx,indx2) = (parameters.s0 +...
             max(0,((curVel *T)+(curVel*deltaV / 2*sqrt(a*b)))));
         
          %% Update value of Gap
         
            sAlpha = leadingCarPost(laneIndx,indx2) - postn(laneIndx,indx2)...
                - attrOfCar.lengthOfCar;
            if(sAlpha<0)
                sAlpha = leadingCarPost(laneIndx,indx2) - attrOfCar.lengthOfCar...
                    - postn(laneIndx,indx2) + Roadlength;
            end
            
            %% Change in velocity
            
            if (laneIndx==1)
                changeInVelocity(laneIndx,indx2) = randomNumberGern(1,2) ...
                    * (1 - (velocity(laneIndx,indx2)/parameters.v0Lane1)^(parameters.delta) ...
                    -(dynDisBwCars(1,indx2)/sAlpha)^2);
            elseif (laneIndx==2)
                changeInVelocity(laneIndx,indx2) = randomNumberGern(1,2) ...
                    * (1 - (velocity(laneIndx,indx2)/parameters.v0Lane2)^(parameters.delta) ...
                    -(dynDisBwCars(1,indx2)/sAlpha)^2);
            else
                changeInVelocity(laneIndx,indx2) = randomNumberGern(1,2) ...
                    * (1 - (velocity(laneIndx,indx2)/parameters.v0Lane3)^(parameters.delta) ...
                    -(dynDisBwCars(1,indx2)/sAlpha)^2);
            end
            
            
            %% Update velocity
            
            tempV = velocity(laneIndx,indx2) + (changeInVelocity(laneIndx,indx2)...
                * parameters.dt);
            velocity(laneIndx,indx2) = tempV;
            
            %% Update postn
            
            temp_Pos = postn(laneIndx,indx2) + velocity(laneIndx,indx2)...
                *parameters.dt + (0.5 * changeInVelocity(laneIndx,indx2) * power(parameters.dt,2));
%             if abs(temp_Pos) >= Roadlength
%                 temp_Pos = abs(temp_Pos - Roadlength) ; 
%             end
            spl_cs_chk = ((-0.5*(velocity(laneIndx,indx2)^2))/...
                changeInVelocity(laneIndx,indx2));
            
            if (postn(laneIndx,indx2)<(abs(temp_Pos)+5))
                postn(laneIndx,indx2) = abs(temp_Pos);
            end
            
            
            %% Special Case
            
            if(spl_cs_chk>0 && tempV<0)
            spl_case_vel = velocity(laneIndx,indx2) + (changeInVelocity...
                (laneIndx,indx2)*parameters.dt);
                if(spl_case_vel<0)
                    velocity(laneIndx,indx2) = 0;
                    temp_Pos = abs(postn(laneIndx,indx2) - ...
                        ((0.5*(velocity(laneIndx,indx2)^2))/...
                        changeInVelocity(1,indx2)));
                    if(abs(temp_Pos)>Roadlength)
                        temp_Pos = abs(temp_Pos-Roadlength);
                        postn(laneIndx,indx2) = abs(temp_Pos);
                    end
                end
            end  
     end
    end
        
         
     %% Draw Cars
            figure(1)
         
            clf;
            % Plot the cars on each lane
                for j = 1:length(noOfCars)
                    for i = 1:parameters.Lanes
                    x1 = postn(i,:);
                    y = 0;
                        plot(0,y,'');    hold on;
                        plot(100,i+11,''); hold on;
                        set(gca,'Color',[0.8 0.8 0.8]);
                        if (i==1)
                            plot(0:Roadlength,i+1,'-ks');  hold on; 
                            plot(x1,i+3,'b>','MarkerFaceColor','b',...
                            'MarkerSize',8); hold on;
                        elseif (i==2)
                            plot(0:Roadlength,i+4,'-ks');  hold on; 
                            plot(x1,i+6,'m>','MarkerFaceColor','m',...
                            'MarkerSize',8);  hold on
                        else
                            plot(0:Roadlength,i+7,'-ks');  hold on; 
                            plot(x1,i+9,'k>','MarkerFaceColor','k',...
                            'MarkerSize',8);  hold on
                        end
                        
                        xlim([0,Roadlength]);
                    end
                    hold on;
                end
            pause(0.1);
            title('Car travel simulation on road');
end
hold off;
disp('running analysis');
%% Plotting the simulations

figure(2)
plot(sum(neighbourPlot(simTime,cars,1),1)/simTime,...
    '--ro','LineWidth',1,...
                       'MarkerEdgeColor','r',...
                       'MarkerFaceColor','r',...
                       'MarkerSize',6);
                        hold on;
plot(sum(neighbourPlot(simTime,cars2,1),1)/simTime,...
    ':kd','LineWidth',1,...
                       'MarkerEdgeColor','m',...
                       'MarkerFaceColor','m',...
                       'MarkerSize',6);
                        hold on;
plot(sum(neighbourPlot(simTime,carsbi1,1),1)/simTime,...
    ':g*','LineWidth',1,...
                       'MarkerEdgeColor','g',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',6); 
                        hold on;
plot(sum(neighbourPlot(simTime,carsbi2,1),1)/simTime,...
    ':cs','LineWidth',1,...
                       'MarkerEdgeColor','c',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',6); 
                        legend('Uni Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - FSPM',...
                               'Bi Directional antenna - TRGM');
                        hold off;                   
xlabel('Cars');
ylabel('Average Neighbour distribution');
title('Cars average neighbour distribution for a single lane- 2Ray vs Freespace');

xLim = 1:simTime;

%% Node Degree

figure(3)
plot(xLim,nodeDegree(cars,simTime,1,1),'--ro','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi1,simTime,1,1),':md','LineWidth',1); hold on;
plot(xLim,nodeDegree(cars2,simTime,1,1),':g*','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi2,simTime,1,1),':cs','LineWidth',1);
%legend('Uni Directional antenna - TRGM');
legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('Neighbour distribution');
title('Node Degree of a Car 1 in lane 1');


figure(4)
plot(xLim,nodeDegree(cars,simTime,2,10),'--ro','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi1,simTime,2,10),':md','LineWidth',1); hold on;
plot(xLim,nodeDegree(cars2,simTime,2,10),':g*','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi2,simTime,2,10),':cs','LineWidth',1); hold on;
%legend('Uni Directional antenna - TRGM');
 legend('Uni Directional antenna - FSPM',...
                                'Bi Directional antenna - FSPM',...
                                'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('Neighbour distribution');
title('Node Degree of a Car 10 in lane 2');

figure(5)
plot(xLim,nodeDegree(cars,simTime,3,10),'--ro','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi1,simTime,3,10),':md','LineWidth',1); hold on;
plot(xLim,nodeDegree(cars2,simTime,3,10),':g*','LineWidth',1); hold on;
plot(xLim,nodeDegree(carsbi2,simTime,3,10),':cs','LineWidth',1); 

legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('Neighbour distribution');
title('Node Degree of a Car 10 in lane 3');



%% Clustering coefficient

figure(6)
plot(xLim,clusteringCoefficient(cars,simTime,3,10),'--ro','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi1,simTime,3,10),':md','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(cars2,simTime,3,10),':g*','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi2,simTime,3,10),':cs','LineWidth',1);

legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('clusteringCoefficient distribution');
title('clustering Coefficient of a Car 10 in lane 3');


figure(7)
plot(xLim,clusteringCoefficient(cars,simTime,2,10),'--ro','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi1,simTime,2,10),':md','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(cars2,simTime,2,10),':g*','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi2,simTime,2,10),':ks','LineWidth',1); hold on;

legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('clustering Coefficient distribution');
title('clustering Coefficient of  Car 10 in lane 2');

figure(8)
plot(xLim,clusteringCoefficient(cars,simTime,1,1),'--ro','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi1,simTime,1,1),':md','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(cars2,simTime,1,1),':g*','LineWidth',1); hold on;
plot(xLim,clusteringCoefficient(carsbi2,simTime,1,1),':cs','LineWidth',1); 

legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('clustering Coefficient distribution');
title('clustering Coefficient of  Car 1 in lane 1');

%% Link Duration
% cars distribution, anyLane ,carID1, anyOtherLane, carID2, number of simulations
figure(9)
plot(xLim,linkDuration(cars,1,1,2,10,simTime),'--ro','LineWidth',1); hold on;
plot(xLim,linkDuration(carsbi1,1,1,2,10,simTime),':md','LineWidth',2); hold on;
plot(xLim,linkDuration(cars2,1,1,2,10,simTime),':g*','LineWidth',1); hold on;
plot(xLim,linkDuration(carsbi2,1,1,2,10,simTime),':cs','LineWidth',2); 

legend('Uni Directional antenna - FSPM',...
                               'Bi Directional antenna - FSPM',...
                               'Uni Directional antenna - TRGM',...
                               'Bi Directional antenna - TRGM');
hold off;
xlabel('time');
ylabel('link duration');
title('Link duration between cars between any two cars concerned');









