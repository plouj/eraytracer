-module(raytracer).
-compile(export_all).

-record(vector, {x, y, z}).
-record(colour, {r, g, b}).
-record(ray, {origin, direction}).
-record(screen, {width, height}). % screen dimensions in the 3D world
-record(camera, {location, rotation, fov, screen}).
-record(sphere, {radius, center, colour}).
%-record(axis_aligned_cube, {size, location}).


raytraced_pixel_list(0, 0, _) ->
    done;
raytraced_pixel_list(Width, Height, Scene) when Width > 0, Height > 0 ->
    lists:flatmap(
      fun(Y) ->
	      lists:map(
		fun(X) ->
			% coordinates passed as a percentage
			trace_ray_from_pixel({X/Width, Y/Height}, Scene) end,
		lists:seq(0, Width - 1)) end,
      lists:seq(0, Height - 1)).

trace_ray_from_pixel({X, Y}, [Camera|Rest_of_scene]) ->
    Ray = ray_through_pixel(X, Y, Camera),
    case nearest_object_intersecting_ray(Ray, Rest_of_scene) of
	{Nearest_object, _Distance} ->
	    object_colour(Nearest_object);
	none ->
	    {255, 255, 255};
	_Else ->
	    {255, 0, 0}
    end.

nearest_object_intersecting_ray(Ray, Scene) ->
    nearest_object_intersecting_ray(Ray, none, infinity, Scene).
nearest_object_intersecting_ray(_Ray, NearestObj, Distance, []) ->
%    io:format("intersecting ~w at ~w~n", [NearestObj, Distance]),
    NearestObj;
nearest_object_intersecting_ray(Ray,
				NearestObj,
				Distance,
				[Object|Rest_of_scene]) ->
    case Object of
	#sphere{} ->
	    NewDistance = ray_sphere_intersect(Ray, Object),
	    if NewDistance /= none ->
%		    io:format("found new intersection at ~w~n", [Distance]),
		    nearest_object_intersecting_ray(
		      Ray,
		      Object,
		      NewDistance,
		      Rest_of_scene);
	       true ->
		    nearest_object_intersecting_ray(Ray,
						    NearestObj,
						    Distance,
						    Rest_of_scene)
	    end;
	_Else ->
	    nearest_object_intersecting_ray(Ray,
					    NearestObj,
					    Distance,
					    Rest_of_scene)
    end.

ray_sphere_intersect(
  #ray{origin=#vector{
	 x=X0, y=Y0, z=Z0},
       direction=#vector{
	 x=Xd, y=Yd, z=Zd}},
  #sphere{radius=Radius, center=#vector{
			   x=Xc, y=Yc, z=Zc}}) ->
    A = Xd*Xd + Yd*Yd + Zd*Zd,
    B = 2 * (Xd*(X0-Xc) + Yd*(Y0-Yc) + Zd*(Z0-Zc)),
    C = (X0-Xc)*(X0-Xc) + (Y0-Yc)*(Y0-Yc) + (Z0-Zc)*(Z0-Zc) - Radius*Radius,
    Discriminant = B*B - 4*A*C,
    %io:format("A=~w B=~w C=~w discriminant=~w~n",
%	      [A, B, C, Discriminant]),
    if Discriminant >= 0 ->
	    T0 = (-B + math:sqrt(Discriminant))/2,
	    T1 = (-B - math:sqrt(Discriminant))/2,
	    %io:format("T0=~w T1=~w~n", [T0, T1]),
	    lists:min([T0, T1]);
       true ->
	    none
    end.

	   

focal_length(Angle, Dimension) ->
    Dimension/(2*math:tan(Angle*(math:pi()/180)/2)).

point_on_screen(X, Y, Camera) ->
    %TODO: implement rotation (using quaternions)
    Screen_width = (Camera#camera.screen)#screen.width,
    Screen_height = (Camera#camera.screen)#screen.height,
    lists:foldl(fun(Vect, Sum) -> vector_add(Vect, Sum) end,
		Camera#camera.location,
		[vector_scalar_mult(
		   #vector{x=0, y=0, z=1},
		   focal_length(
		     Camera#camera.fov,
		     Screen_width)),
		 #vector{x = (X-0.5) * Screen_width,
			 y=0,
			 z=0},
		 #vector{x=0,
			 y= (Y-0.5) * Screen_height,
			 z=0}
		]).
    

shoot_ray(From, Through) ->
    #ray{origin=From, direction=vector_sub(Through, From)}.

% assume that X and Y are percentages of the 3D world screen dimensions
ray_through_pixel(X, Y, Camera) ->
    shoot_ray(Camera#camera.location, point_on_screen(X, Y, Camera)).

vectors_equal(V1, V2) ->
    vectors_equal(V1, V2, 0.0001).
vectors_equal(V1, V2, Epsilon) ->
    (V1#vector.x + Epsilon >= V2#vector.x)
	and (V1#vector.x - Epsilon =<V2#vector.x)
	and (V1#vector.y + Epsilon >= V2#vector.y)
	and (V1#vector.y - Epsilon =<V2#vector.y)
    	and (V1#vector.z + Epsilon >= V2#vector.z)
	and (V1#vector.z - Epsilon =<V2#vector.z).


vector_add(V1, V2) ->
    #vector{x = V1#vector.x + V2#vector.x,
	    y = V1#vector.y + V2#vector.y,
	    z = V1#vector.z + V2#vector.z}.

vector_sub(V1, V2) ->
        #vector{x = V1#vector.x - V2#vector.x,
	    y = V1#vector.y - V2#vector.y,
	    z = V1#vector.z - V2#vector.z}.

vector_square_mag(#vector{x=X, y=Y, z=Z}) ->
    X*X + Y*Y + Z*Z.

vector_mag(V) ->
    math:sqrt(vector_square_mag(V)).

vector_scalar_mult(#vector{x=X, y=Y, z=Z}, Scalar) ->
    #vector{x=X*Scalar, y=Y*Scalar, z=Z*Scalar}.

vector_dot_product(#vector{x=A1, y=A2, z=A3}, #vector{x=B1, y=B2, z=B3}) ->
    A1*B1 + A2*B2 + A3*B3.

vector_cross_product(#vector{x=A1, y=A2, z=A3}, #vector{x=B1, y=B2, z=B3}) ->
    #vector{x = A2*B3 - A3*B2,
	    y = A3*B1 - A1*B3,
	    z = A1*B2 - A2*B1}.

vector_normalize(V) ->
    Mag = vector_mag(V),
    if Mag == 0 ->
	    #vector{x=0, y=0, z=0};
       true ->
	    vector_scalar_mult(V, 1/vector_mag(V))
    end.

vector_neg(#vector{x=X, y=Y, z=Z}) ->
    #vector{x=-X, y=-Y, z=-Z}.

vector_rotate(V1, _V2) ->
    %TODO: implement using quaternions
    V1.

object_colour(#sphere{ colour=C}) ->
    {C#colour.r, C#colour.g, C#colour.b};
object_colour(_Unknown) ->
    {0, 0, 0}.

% returns a list of objects in the scene
% camera is assumed to be the first element in the scene
scene() ->
    [#camera{location=#vector{x=0, y=0, z=0},
	     rotation=#vector{x=0, y=0, z=0},
	     fov=90,
	     screen=#screen{width=4, height=3}},
     #sphere{radius=4,
	     center=#vector{x=0, y=0, z=7},
	     colour=#colour{r=0, g=128, b=255}}
    ].


write_pixels_to_ppm(Width, Height, MaxValue, Pixels, Filename) ->
    case file:open(Filename, write) of
	{ok, IoDevice} ->
	    io:format("file opened~n", []),
	    io:format(IoDevice, "P3~n", []),
	    io:format(IoDevice, "~p ~p~n", [Width, Height]),
	    io:format(IoDevice, "~p~n", [MaxValue]),
	    lists:foreach(
	      fun({R, G, B}) ->
		      io:format(IoDevice, "~p ~p ~p ",
				[R, G, B]) end,
	      Pixels),
	    file:close(IoDevice),
	    io:format("done~n", []);
	error ->
	    io:format("error opening file~n", [])
    end.

go() ->
    go(10, 10, "/tmp/traced.ppm").
go(Width, Height, Filename) ->
    write_pixels_to_ppm(Width,
			Height,
			255,
			raytraced_pixel_list(Width,
					     Height,
					     scene()),
			Filename).

% testing
scene_test() ->
    io:format("testing the scene function", []),
    case scene() of
	[{camera,
	  {vector, 0, 0, 0},
	  {vector, 0, 0, 0},
	  90,
	  {screen, 4, 3}},
	 {sphere,
	  4,
	  {vector, 0, 0, 7},
	  {colour, 0, 128, 255}}] ->
	    true;
_Else ->
	    false
    end.

failing_test() ->
    io:format("this test always fails", []),
    false.

passing_test() ->
    io:format("this test always passes", []),
    true.

run_tests() ->
    Tests = [fun scene_test/0,
	     fun passing_test/0,
	     fun vector_equality_test/0,
	     fun vector_addition_test/0,
	     fun vector_subtraction_test/0,
	     fun vector_square_mag_test/0,
	     fun vector_mag_test/0,
	     fun vector_scalar_multiplication_test/0,
	     fun vector_dot_product_test/0,
	     fun vector_cross_product_test/0,
	     fun vector_normalization_test/0,
	     fun vector_negation_test/0,
	     fun ray_through_pixel_test/0,
	     fun ray_shooting_test/0,
	     fun point_on_screen_test/0,
	     fun nearest_object_intersecting_ray_test/0,
	     fun focal_length_test/0,
	     fun vector_rotation_test/0
	    ],
    run_tests(Tests, 1, true).

run_tests([], _Num, Success) ->
    case Success of
	true ->
	    io:format("Success!~n", []),
	    ok;
	_Else ->
	    io:format("some tests failed~n", []),
	    failed
    end;

run_tests([First_test|Rest_of_tests], Num, Success_so_far) ->
    io:format("test #~p: ", [Num]),
    Current_success = First_test(),
    case Current_success of
	true ->
	    io:format(" - OK~n", []);
	_Else ->
	    io:format(" - FAILED~n", [])
    end,
    run_tests(Rest_of_tests, Num + 1, Current_success and Success_so_far).

vector_equality_test() ->
    io:format("vector equality"),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1234, y=-234, z=0},
    Vector3 = #vector{x=0.0983, y=0.0214, z=0.12342},
    Vector4 = #vector{x=0.0984, y=0.0213, z=0.12341},
    Vector5 = #vector{x=10/3, y=-10/6, z=8/7},
    Vector6 = #vector{x=3.3, y=-1.6, z=1.1},
    
    Subtest1 = vectors_equal(Vector1, Vector1)
	and vectors_equal(Vector2, Vector2)
	and not (vectors_equal(Vector1, Vector2))
	and not (vectors_equal(Vector2, Vector1)),
    Subtest2 = vectors_equal(Vector3, Vector4, 0.0001),
    Subtest3 = vectors_equal(Vector5, Vector6, 0.1),

    Subtest1 and Subtest2 and Subtest3.
    
    
vector_addition_test() ->
    io:format("vector addition", []),
    Vector0 = vector_add(
		#vector{x=3, y=7, z=-3},
		#vector{x=0, y=-24, z=123}),	       
    Subtest1 = (Vector0#vector.x == 3)
	and (Vector0#vector.y == -17)
	and (Vector0#vector.z == 120),

    Vector1 = #vector{x=5, y=0, z=984},
    Vector2 = vector_add(Vector1, Vector1),
    Subtest2 = (Vector2#vector.x == Vector1#vector.x*2)
	and (Vector2#vector.y == Vector1#vector.y*2)
	and (Vector2#vector.z == Vector1#vector.z*2),

    Vector3 = #vector{x=908, y=-098, z=234},
    Vector4 = vector_add(Vector3, #vector{x=0, y=0, z=0}),
    Subtest3 = vectors_equal(Vector3, Vector4),

    Subtest1 and Subtest2 and Subtest3.

vector_subtraction_test() ->
    io:format("vector subtraction", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=8390, y=-2098, z=939},
    Vector3 = #vector{x=1, y=1, z=1},
    Vector4 = #vector{x=-1, y=-1, z=-1},

    Subtest1 = vectors_equal(Vector1, vector_sub(Vector1, Vector1)),
    Subtest2 = vectors_equal(Vector3, vector_sub(Vector3, Vector1)),
    Subtest3 = not vectors_equal(Vector3, vector_sub(Vector1, Vector3)),
    Subtest4 = vectors_equal(Vector4, vector_sub(Vector4, Vector1)),
    Subtest5 = not vectors_equal(Vector4, vector_sub(Vector1, Vector4)),
    Subtest5 = vectors_equal(vector_add(Vector2, Vector4),
			     vector_sub(Vector2, Vector3)),

    Subtest1 and Subtest2 and Subtest3 and Subtest4 and Subtest5.

vector_square_mag_test() ->
    io:format("vector square magnitude test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=1, z=1},
    Vector3 = #vector{x=3, y=-4, z=0},

    Subtest1 = (0 == vector_square_mag(Vector1)),
    Subtest2 = (3 == vector_square_mag(Vector2)),
    Subtest3 = (25 == vector_square_mag(Vector3)),

    Subtest1 and Subtest2 and Subtest3.

vector_mag_test() ->
    io:format("vector magnitude test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=1, z=1},
    Vector3 = #vector{x=3, y=-4, z=0},

    Subtest1 = (0 == vector_mag(Vector1)),
    Subtest2 = (math:sqrt(3) == vector_mag(Vector2)),
    Subtest3 = (5 == vector_mag(Vector3)),

    Subtest1 and Subtest2 and Subtest3.

vector_scalar_multiplication_test() ->
    io:format("scalar multiplication test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=1, z=1},
    Vector3 = #vector{x=3, y=-4, z=0},

    Subtest1 = vectors_equal(Vector1, vector_scalar_mult(Vector1, 45)),
    Subtest2 = vectors_equal(Vector1, vector_scalar_mult(Vector1, -13)),
    Subtest3 = vectors_equal(Vector1, vector_scalar_mult(Vector3, 0)),
    Subtest4 = vectors_equal(#vector{x=4, y=4, z=4},
			     vector_scalar_mult(Vector2, 4)),
    Subtest5 = vectors_equal(Vector3, vector_scalar_mult(Vector3, 1)),
    Subtest6 = not vectors_equal(Vector3, vector_scalar_mult(Vector3, -3)),
    
    Subtest1 and Subtest2 and Subtest3 and Subtest4 and Subtest5 and Subtest6.

vector_dot_product_test() ->
    io:format("dot product test", []),
    Vector1 = #vector{x=1, y=3, z=-5},
    Vector2 = #vector{x=4, y=-2, z=-1},
    Vector3 = #vector{x=0, y=0, z=0},
    Vector4 = #vector{x=1, y=0, z=0},
    Vector5 = #vector{x=0, y=1, z=0},
    
    Subtest1 = 3 == vector_dot_product(Vector1, Vector2),
    Subtest2 = vector_dot_product(Vector2, Vector2)
	== vector_square_mag(Vector2),
    Subtest3 = 0 == vector_dot_product(Vector3, Vector1),
    Subtest4 = 0 == vector_dot_product(Vector4, Vector5),
    
    Subtest1 and Subtest2 and Subtest3 and Subtest4.

vector_cross_product_test() ->
    io:format("cross product test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=0, z=0},
    Vector3 = #vector{x=0, y=1, z=0},
    Vector4 = #vector{x=0, y=0, z=1},
    Vector5 = #vector{x=1, y=2, z=3},
    Vector6 = #vector{x=4, y=5, z=6},
    Vector7 = #vector{x=-3, y=6, z=-3},
    Vector8 = #vector{x=-1, y=0, z=0},
    Vector9 = #vector{x=-9, y=8, z=433},
    
    Subtest1 = vectors_equal(Vector1, vector_cross_product(Vector2, Vector2)),
    Subtest2 = vectors_equal(Vector1, vector_cross_product(Vector2, Vector8)),
    Subtest3 = vectors_equal(Vector2, vector_cross_product(Vector3, Vector4)),
    Subtest4 = vectors_equal(Vector7, vector_cross_product(Vector5, Vector6)),
    Subtest5 = vectors_equal(
		 vector_cross_product(Vector7,
				      vector_add(Vector8, Vector9)),
		 vector_add(
		   vector_cross_product(Vector7, Vector8),
		   vector_cross_product(Vector7, Vector9))),
    Subtest6 = vectors_equal(Vector1,
			     vector_add(
			       vector_add(
				 vector_cross_product(
				   Vector7,
				   vector_cross_product(Vector8, Vector9)),
				 vector_cross_product(
				   Vector8,
				   vector_cross_product(Vector9, Vector7))),
			       vector_cross_product(
				 Vector9,
				 vector_cross_product(Vector7, Vector8)))),
			     
    Subtest1 and Subtest2 and Subtest3 and Subtest4 and Subtest5 and Subtest6.

vector_normalization_test() ->
    io:format("normalization test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=0, z=0},
    Vector3 = #vector{x=5, y=0, z=0},

    Subtest1 = vectors_equal(Vector1, vector_normalize(Vector1)),
    Subtest2 = vectors_equal(Vector2, vector_normalize(Vector2)),
    Subtest3 = vectors_equal(Vector2, vector_normalize(Vector3)),
    Subtest4 = vectors_equal(Vector2, vector_normalize(
					vector_scalar_mult(Vector2, 324))),

    Subtest1 and Subtest2 and Subtest3 and Subtest4.

vector_negation_test() ->
    io:format("vector negation test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=4, y=-5, z=6},

    Subtest1 = vectors_equal(Vector1, vector_neg(Vector1)),
    Subtest2 = vectors_equal(Vector2, vector_neg(vector_neg(Vector2))),
    
    Subtest1 and Subtest2.

ray_through_pixel_test() ->
    io:format("ray through pixel test", []),
    false.

ray_shooting_test() ->
    io:format("ray shooting test"),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=0, z=0},
    
    Subtest1 = vectors_equal(
		 (shoot_ray(Vector1, Vector2))#ray.direction,
		 Vector2),
    
    Subtest1.

ray_sphere_intersection_test() ->
    Sphere = #sphere{
      radius=3,
      center=#vector{x = 0, y=0, z=10},
      colour=#colour{r=111, g=111, b=111}},
    Ray1 = #ray{
      origin=#vector{x=0, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    Ray2 = #ray{
      origin=#vector{x=3, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    Ray3 = #ray{
      origin=#vector{x=4, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    io:format("ray/sphere intersection=~w~n", [ray_sphere_intersect(Ray1, Sphere)]),
    Subtest1 = ray_sphere_intersect(Ray1, Sphere) == 7.0,
    Subtest2 = ray_sphere_intersect(Ray2, Sphere) == 10.0,
    Subtest3 = ray_sphere_intersect(Ray3, Sphere) == none,
    io:format("ray/sphere intersection=~w~n", [ray_sphere_intersect(Ray2, Sphere)]),
    io:format("ray/sphere intersection=~w~n", [ray_sphere_intersect(Ray3, Sphere)]),
    Subtest1 and Subtest2 and Subtest3.

point_on_screen_test() ->
    io:format("point on screen test", []),
    Camera1 = #camera{location=#vector{x=0, y=0, z=0},
		      rotation=#vector{x=0, y=0, z=0},
		      fov=90,
		      screen=#screen{width=1, height=1}},
    Camera2 = #camera{location=#vector{x=0, y=0, z=0},
		      rotation=#vector{x=0, y=0, z=0},
		      fov=90,
		      screen=#screen{width=640, height=480}},

    Subtest1 = vectors_equal(
		 #vector{x=0, y=0, z=0.5},
		 point_on_screen(0.5, 0.5, Camera1)),
    Subtest2 = vectors_equal(
		 #vector{x=-0.5, y=-0.5, z=0.5},
		 point_on_screen(0, 0, Camera1)),
    Subtest3 = vectors_equal(
		 #vector{x=0.5, y=0.5, z=0.5},
		 point_on_screen(1, 1, Camera1)),
    Subtest4 = vectors_equal(
		 point_on_screen(0, 0, Camera2),
		 #vector{x=-320, y=-240, z=320}),
    Subtest5 = vectors_equal(
		 point_on_screen(1, 1, Camera2),
		 #vector{x=320, y=240, z=320}),
    Subtest6 = vectors_equal(
		point_on_screen(0.5, 0.5, Camera2),
		#vector{x=0, y=0, z=320}),

    Subtest1 and Subtest2 and Subtest3 and Subtest4 and Subtest5 and Subtest6.

nearest_object_intersecting_ray_test() ->
    io:format("nearest object intersecting ray test", []),
    false.

focal_length_test() ->
    Epsilon = 0.1,
    Size = 36,
    io:format("focal length test", []),
    lists:foldl(
      fun({Focal_length, Dimension}, Matches) ->
	      %Result = focal_length(Dimension, Size),
	      %io:format("comparing ~w ~w ~w ~w~n", [Focal_length, Dimension, Result, Matches]),
     Matches
		  and ((Focal_length + Epsilon >= focal_length(
						    Dimension, Size))
		       and (Focal_length - Epsilon =< focal_length(
						       Dimension, Size)))
      end, true,
      [{13, 108}, {15, 100.4}, {18, 90}, {21, 81.2}]).

vector_rotation_test() ->
    io:format("vector rotation test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=0, y=1, z=0},
    Vector3 = #vector{x=90, y=0, z=0},
    Vector4 = #vector{x=45, y=0, z=0},
    Vector5 = #vector{x=30.11, y=-988.2, z=92.231},
    Vector6 = #vector{x=0, y=0, z=1},

    Subtest1 = vectors_equal(
		 vector_rotate(Vector1, Vector1),
		 Vector1),
    Subtest2 = vectors_equal(
		 vector_rotate(Vector5, Vector1),
		 Vector5),
    Subtest3 = vectors_equal(
		 vector_rotate(
		   vector_rotate(Vector5, Vector4),
		   Vector4),
		 vector_rotate(Vector5, Vector3)),
    Subtest4 = vectors_equal(
		 Vector6,
		 vector_rotate(Vector2, Vector3)),
    
    Subtest1 and Subtest2 and Subtest3 and Subtest4.
