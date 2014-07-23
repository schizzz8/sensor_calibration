function v = t2v(T)

	[m n] = size(T);

	if ( m == 4 && n == 4 )
		v = zeros(6,1);
		v(1:3) = T(1:3,4);

		if (T(2,1) > 0.998)
			heading = atan2(T(1,3),T(3,3));
			attitude = pi/2;
			bank = 0;
		end

		if (T(2,1) < -0.998) 
			heading = atan2(T(1,3),T(3,3));
			attitude = pi/2;
			bank = 0;
		end

		heading = atan2(-T(3,1),T(1,1));
		bank = atan2(-T(2,3),T(2,2));
		attitude = asin(T(2,1));

		v(4:6) = [bank;heading;attitude];
	end
end
