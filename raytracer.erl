-module(raytracer).
-compile(export_all).

-record(vector, {x, y, z}).
-record(ray, {start, direction}).
-record(camera, {location, rotation, fov}).
-record(sphere, {radius, location}).
-record(axis_aligned_cube, {size, location}).

raytraced_pixel_list(0, 0, _) ->
    done;
raytraced_pixel_list(Width, Height, Scene) when Width > 0, Height > 0 ->
    lists:flatmap(
      fun(X) ->
	      lists:map(
		fun(Y) ->
			trace_ray_from_pixel({X, Y}, Scene) end,
		lists:seq(0, Height - 1)) end,
      lists:seq(0, Width - 1)).

trace_ray_from_pixel({X, Y}, _Scene) ->
    {random:uniform(256)-1, random:uniform(256)-1, random:uniform(256)-1}.

% returns a list of objects in the scene
scene() ->
    [#camera{location=#vector{x=0, y=0, z=0},
	     rotation=#vector{x=1, y=0, z=0},
	     fov=90},
     #sphere{radius=2, location=#vector{x=5, y=0, z=0}}
    ].


write_pixels_to_ppm(Width, Height, Pixels, Filename) ->
    case file:open(Filename, write) of
	{ok, IoDevice} ->
	    io:format("file opened~n", []),
	    io:format(IoDevice, "P3~n", []),
	    io:format(IoDevice, "~p ~p~n", [Width, Height]),
	    io:format(IoDevice, "255~n", []),
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
			raytraced_pixel_list(Width,
					     Height,
					     scene()),
			Filename).
