function [ carPlot ] = neighbourPlot( simTime,cars,lane )

carPlot = zeros(simTime, length(cars));

for indx = 1: length(cars)
    car = cars(lane,indx);
    simArray = car.simNo;
    for indx2 = 1: length(simArray)
        carPlot(simArray(indx2),indx) = ...
            carPlot(simArray(indx2),indx) + 1;
    end
end

end

