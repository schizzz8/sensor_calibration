function e = error_function(prediction,measurement)

%This function computes the difference between the predicted pose of the sensor
%and the measured pose

	e = t2v(measurement\prediction);
end

