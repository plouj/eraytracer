-module(raytracer).
-compile(export_all).

-record(vector, {x, y, z}).
-record(colour, {r, g, b}).
%-record(ray, {origin, direction}).
-record(screen, {width, height}). % screen dimensions in the 3D world
-record(camera, {location, rotation, fov, screen}).
-record(sphere, {radius, center, colour}).
%-record(axis_aligned_cube, {size, location}).

raytraced_pixel_list(0, 0, _) ->
    done;
raytraced_pixel_list(Width, Height, Scene) when Width > 0, Height > 0 ->
    lists:flatmap(
      fun(X) ->
	      lists:map(
		fun(Y) ->
			% coordinates passed as a percentage
			trace_ray_from_pixel({X/Width, Y/Height}, Scene) end,
		lists:seq(0, Height - 1)) end,
      lists:seq(0, Width - 1)).

trace_ray_from_pixel({X, Y}, [Camera|Rest_of_scene]) ->
    Ray = ray_through_pixel(X, Y, Camera),
    {_Nearest_object, _Distance} = nearest_object_intersecting_ray(Ray, Rest_of_scene),
    % return the Nearest_object's colour
    {random:uniform(256)-1, random:uniform(256)-1, random:uniform(256)-1}.

nearest_object_intersecting_ray(_Ray, _Scene) ->
    none.

point_on_screen(_X, _Y, _Camera) ->
    none.

shoot_ray(_From, _Through) ->
    none.

% assume that X and Y are percentages of the 3D world screen dimensions
ray_through_pixel(X, Y, Camera) ->
    shoot_ray(Camera#camera.location, point_on_screen(X, Y, Camera)).

vectors_equal(V1, V2) ->
    (V1#vector.x == V2#vector.x)
	and (V1#vector.y == V2#vector.y)
	and (V1#vector.z == V2#vector.z).

add_vectors(V1, V2) ->
    #vector{x = V1#vector.x + V2#vector.x,
	    y = V1#vector.y + V2#vector.y,
	    z = V1#vector.z + V2#vector.z}.

subtract_vectors(V1, V2) ->
        #vector{x = V1#vector.x - V2#vector.x,
	    y = V1#vector.y - V2#vector.y,
	    z = V1#vector.z - V2#vector.z}.

vector_square_mag(#vector{x=X, y=Y, z=Z}) ->
    X*X + Y*Y + Z*Z.

vector_magnitude(V) ->
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
    Mag = vector_magnitude(V),
    if Mag == 0 ->
	    #vector{x=0, y=0, z=0};
       true ->
	    vector_scalar_mult(V, 1/vector_magnitude(V))
    end.

vector_neg(#vector{x=X, y=Y, z=Z}) ->
    #vector{x=-X, y=-Y, z=-Z}.

% returns a list of objects in the scene
% camera is assumed to be the first element in the scene
scene() ->
    [#camera{location=#vector{x=0, y=0, z=0},
	     rotation=#vector{x=1, y=0, z=0},
	     fov=90,
	     screen=#screen{width=1, height=1}},
     #sphere{radius=2,
	     center=#vector{x=5, y=0, z=0},
	     colour=#colour{r=0, g=128, b=255}}
    ].


write_pixels_to_ppm(Width, Height, Pixels, MaxValue, Filename) ->
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
    go(1, 1, "/tmp/traced.ppm").
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
	  {vector, 1, 0, 0},
	  90,
	  {screen, 1, 1}},
	 {sphere,
	  2,
	  {vector, 5, 0, 0},
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
	     fun vector_magnitude_test/0,
	     fun vector_scalar_multiplication_test/0,
	     fun vector_dot_product_test/0,
	     fun vector_cross_product_test/0,
	     fun vector_normalization_test/0,
	     fun vector_negation_test/0,
	     fun ray_through_pixel_test/0,
	     fun ray_shooting_test/0,
	     fun point_on_screen_test/0,
	     fun nearest_object_intersecting_ray_test/0
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
    
    vectors_equal(Vector1, Vector1)
	and vectors_equal(Vector2, Vector2)
	and not (vectors_equal(Vector1, Vector2))
	and not (vectors_equal(Vector2, Vector1)).
    
    
vector_addition_test() ->
    io:format("vector addition", []),
    Vector0 = add_vectors(
		#vector{x=3, y=7, z=-3},
		#vector{x=0, y=-24, z=123}),	       
    Subtest1 = (Vector0#vector.x == 3)
	and (Vector0#vector.y == -17)
	and (Vector0#vector.z == 120),

    Vector1 = #vector{x=5, y=0, z=984},
    Vector2 = add_vectors(Vector1, Vector1),
    Subtest2 = (Vector2#vector.x == Vector1#vector.x*2)
	and (Vector2#vector.y == Vector1#vector.y*2)
	and (Vector2#vector.z == Vector1#vector.z*2),

    Vector3 = #vector{x=908, y=-098, z=234},
    Vector4 = add_vectors(Vector3, #vector{x=0, y=0, z=0}),
    Subtest3 = vectors_equal(Vector3, Vector4),

    Subtest1 and Subtest2 and Subtest3.

vector_subtraction_test() ->
    io:format("vector subtraction", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=8390, y=-2098, z=939},
    Vector3 = #vector{x=1, y=1, z=1},
    Vector4 = #vector{x=-1, y=-1, z=-1},

    Subtest1 = vectors_equal(Vector1, subtract_vectors(Vector1, Vector1)),
    Subtest2 = vectors_equal(Vector3, subtract_vectors(Vector3, Vector1)),
    Subtest3 = not vectors_equal(Vector3, subtract_vectors(Vector1, Vector3)),
    Subtest4 = vectors_equal(Vector4, subtract_vectors(Vector4, Vector1)),
    Subtest5 = not vectors_equal(Vector4, subtract_vectors(Vector1, Vector4)),
    Subtest5 = vectors_equal(add_vectors(Vector2, Vector4),
			     subtract_vectors(Vector2, Vector3)),

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

vector_magnitude_test() ->
    io:format("vector magnitude test", []),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=1, z=1},
    Vector3 = #vector{x=3, y=-4, z=0},

    Subtest1 = (0 == vector_magnitude(Vector1)),
    Subtest2 = (math:sqrt(3) == vector_magnitude(Vector2)),
    Subtest3 = (5 == vector_magnitude(Vector3)),

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
				      add_vectors(Vector8, Vector9)),
		 add_vectors(
		   vector_cross_product(Vector7, Vector8),
		   vector_cross_product(Vector7, Vector9))),
    Subtest6 = vectors_equal(Vector1,
			     add_vectors(
			       add_vectors(
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
    io:format("ray shooting test", []),
    false.

point_on_screen_test() ->
    io:format("point on screen test", []),
    false.

nearest_object_intersecting_ray_test() ->
    io:format("nearest object intersecting ray test", []),
    false.
