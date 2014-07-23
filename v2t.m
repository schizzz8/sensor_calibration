function T = v2t(v)

	[m, n] = size(v);

	if ( m == 6 && n == 1 )
		T = eye(4);
		T(1:3,4) = v(1:3);

		cb = cos(euler(1));
		ch = cos(euler(2));
		ca = cos(euler(3));
		sb = sin(euler(1));
		sh = sin(euler(2));
		sa = sin(euler(3));

T(1:3,1:3) = [ch*ca;-ch*sa*cb + sh*sb;ch*sa*sb + sh*cb;
	      sa;ca*cb;-ca*sb;
	      -sh*ca;sh*sa*cb + ch*sb;-sh*sa*sb + ch*cb];
	end
end
