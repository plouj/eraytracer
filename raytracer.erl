
%%  raytracer.erl

%%  a simple raytracer written in Erlang

%%  Copyright (c) 2008 Michael Ploujnikov

%%      This program is free software: you can redistribute it and/or modify
%%      it under the terms of the GNU General Public License as published by
%%      the Free Software Foundation, either version 2 of the License, or
%%      (at your option) any later version.

%%      This program is distributed in the hope that it will be useful,
%%      but WITHOUT ANY WARRANTY; without even the implied warranty of
%%      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%%      GNU General Public License for more details.

%%      You should have received a copy of the GNU General Public License
%%      along with this program.  If not, see <http://www.gnu.org/licenses/>.

%%  Features:
%%  * three object types:
%%   * spheres
%%   * planes
%%   * triangles (not done)
%%  * point lights
%%  * shadows
%%  * lighting based on local illumination models
%%   * ambient (not done)
%%   * diffuse
%%   * specular
%%   * attenuation (not done)
%%  * reflections to a fixed depth
%%  * PPM output file format
%%  * randomly generated scene (not done)
%%  * useful test suite (working but not very friendly when fails)
%%  * concurrent
%%   * specify how many pixels to give to each process
%%  * distributed (across multiple computers)

%% Instructions
%% The simplest way to run this raytracer is through an Erlang shell with the go() function.
%% At the very least, the go function expects one parameter,
%%  which tells it how to run the raytracer:
%% To execute in a simple, serial manner use:
%% > raytracer:go(simple)
%% To execute in a parallel manner (taking advantage of multiple CPUs available to the system) use:
%% > raytracer:go(concurrent)
%% To execute in a distributed manner use:
%% > raytracer:go(distributed)
%% For the last command to work you need to do some preparation of the nodes (computers) that will be running the raytracer. To same myself some time I'll assume that you know how to do that or can figure out how to do that by reading http://erlang.org/doc/getting_started/part_frame.html and then http://www.erlang.org/doc/man/pool.html .
%% Read the rest of the code for additional parameters that the go() function takes
%% See the example *.sh files for examples of standalone invocation


-module(raytracer).
-export([go/1,
	 go/5,
	 raytrace/1,
	 raytrace/5,
	 run_tests/0,
	 master/2,
	 worker/5,
	 distributed_worker/6,
	 standalone/1,
	 standalone/5,
	 raytraced_pixel_list_simple/4,
	 raytraced_pixel_list_concurrent/4,
	 raytraced_pixel_list_distributed/4
	]).

-record(vector, {x, y, z}).
-record(colour, {r, g, b}).
-record(ray, {origin, direction}).
-record(screen, {width, height}). % screen dimensions in the 3D world
-record(camera, {location, rotation, fov, screen}).
-record(material, {colour, specular_power, shininess, reflectivity}).
-record(sphere, {radius, center, material}).
-record(triangle, {v1, v2, v3, material}).
-record(plane, {normal, distance, material}).
-record(point_light, {diffuse_colour, location, specular_colour}).
-define(BACKGROUND_COLOUR, #colour{r=0, g=0, b=0}).
-define(UNKNOWN_COLOUR, #colour{r=0, g=1, b=0}).
-define(FOG_DISTANCE, 40).

raytraced_pixel_list_simple(0, 0, _, _) ->
    done;
raytraced_pixel_list_simple(Width, Height, Scene, Recursion_depth)
  when Width > 0, Height > 0 ->
    lists:flatmap(
      fun(Y) ->
	      lists:map(
		fun(X) ->
			% coordinates passed as a percentage
			{1, colour_to_pixel(
			    trace_ray_through_pixel(
			      {X/Width, Y/Height}, Scene, Recursion_depth))} end,
		lists:seq(0, Width - 1)) end,
      lists:seq(0, Height - 1)).

raytraced_pixel_list_concurrent(0, 0, _, _) ->
    done;
raytraced_pixel_list_concurrent(Width, Height, Scene, Recursion_depth)
  when Width > 0, Height > 0 ->
    Master_PID = spawn(raytracer, master, [self(), Width*Height]),
    lists:flatmap(
      fun(Y) ->
	      lists:map(
		fun(X) ->
			% coordinates passed as a percentage
			spawn(raytracer, worker,
			  [Master_PID, X+Y*Width, {X/Width, Y/Height}, Scene, Recursion_depth]) end,
		lists:seq(0, Width - 1)) end,
      lists:seq(0, Height - 1)),
    io:format("all workers have been spawned~n", []),
    receive
	Final_pixel_list ->
	    Final_pixel_list
    end.

raytraced_pixel_list_distributed(0, 0, _, _) ->
    done;
raytraced_pixel_list_distributed(Width, Height, Scene, Recursion_depth)
  when Width > 0, Height > 0 ->
    io:format("distributed tracing~n", []),
    Pool_master = pool:start(renderslave),
    io:format("Pool master is ~p~n", [Pool_master]),
    io:format("Nodes are ~p~n", [pool:get_nodes()]),
    Master_PID = pool:pspawn(raytracer, master, [self(), Width*Height]),
    Pixels = [{X, Y} || X <- lists:seq(0, Width-1), Y <- lists:seq(0, Height-1)],
    distribute_work(Pixels, trunc(Width*Height/64), Master_PID, Width, Height, Scene,
		    Recursion_depth),
    io:format("all workers have been spawned~n", []),
    receive
	Final_pixel_list ->
	    Final_pixel_list
    end.

distribute_work(Pixels, Pixels_per_worker, Master_PID, Width, Height, Scene,
		Recursion_depth) when length(Pixels) > Pixels_per_worker ->
    {To_work_on, The_rest} = lists:split(Pixels_per_worker, Pixels),
    pool:pspawn(raytracer, distributed_worker,
		[Master_PID, To_work_on, Width, Height, Scene, Recursion_depth]),
    distribute_work(The_rest, Pixels_per_worker, Master_PID,
			    Width, Height, Scene, Recursion_depth);
distribute_work(Pixels, _Pixels_per_worker, Master_PID, Width, Height, Scene,
		Recursion_depth) ->
    pool:pspawn(raytracer, distributed_worker,
		[Master_PID, Pixels, Width, Height, Scene, Recursion_depth]).

master(Program_PID, Pixel_count) ->
    master(Program_PID, Pixel_count, []).
master(Program_PID, 0, Pixel_list) ->
    io:format("master is done~n", []),
    Program_PID ! lists:keysort(1, Pixel_list);
% assumes all workers eventually return a good value
master(Program_PID, Pixel_count, Pixel_list) ->
    receive
	Pixel_tuple ->
	    master(Program_PID, Pixel_count-1, [Pixel_tuple|Pixel_list])
    end.
    

% assumes X and Y are percentages of the screen dimensions
worker(Master_PID, Pixel_num, {X, Y}, Scene, Recursion_depth) ->
    Master_PID ! {Pixel_num,
		  colour_to_pixel(trace_ray_through_pixel({X, Y}, Scene, Recursion_depth))}.

distributed_worker(Master_PID, Pixels, Width, Height, Scene, Recursion_depth) ->
    %io:format("~pworker doing ~p pixels=~p~n", [node(), length(Pixels), Pixels]),
    lists:foreach(
      fun({X, Y}) ->
	      Master_PID ! {X+Y*Width,
			    colour_to_pixel(
			      trace_ray_through_pixel(
				{X/Width, Y/Height}, Scene, Recursion_depth))}
      end,
      Pixels).

trace_ray_through_pixel({X, Y}, [Camera|Rest_of_scene], Recursion_depth) ->
    pixel_colour_from_ray(
      ray_through_pixel(X, Y, Camera),
      Rest_of_scene,
      Recursion_depth).

pixel_colour_from_ray(_Ray, _Scene, 0) ->
    #colour{r=0, g=0, b=0};
pixel_colour_from_ray(Ray, Scene, Recursion_depth) ->
    case nearest_object_intersecting_ray(Ray, Scene) of
	{Nearest_object, _Distance, Hit_location, Hit_normal} ->
	    %io:format("hit: ~w~n", [{Nearest_object, _Distance}]),

	      vector_to_colour(
		lighting_function(
		  Ray,
		  Nearest_object,
		  Hit_location,
		  Hit_normal,
		  Scene,
		  Recursion_depth));
	_Else ->
	    ?BACKGROUND_COLOUR
    end.

% my own illumination formula
% ideas were borrowed from:
% http://www.devmaster.net/wiki/Lighting
% http://svn.icculus.org/darkwar/trunk/base/shaders/light.frag?rev=1067&view=auto
lighting_function(Ray, Object, Hit_location, Hit_normal, Scene,
		  Recursion_depth) ->
    lists:foldl(
      fun (#point_light{diffuse_colour=Light_colour,
			location=Light_location,
			specular_colour=Specular_colour},
	   Final_colour) ->
	      Reflection = vector_scalar_mult(
			     colour_to_vector(		
			       pixel_colour_from_ray(
				 #ray{origin=Hit_location,
				      direction=vector_bounce_off_plane(
						  Ray#ray.direction, Hit_normal)},
				 Scene,
				 Recursion_depth-1)),
			     object_reflectivity(Object)),
	      Light_contribution = vector_add(
				     diffuse_term(
				       Object,
				       Light_location,
				       Hit_location,
				       Hit_normal),
				     specular_term(
				       Ray#ray.direction,
				       Light_location,
				       Hit_location,
				       Hit_normal,
				       object_specular_power(Object),
				       object_shininess(Object),
				       Specular_colour)),
	      vector_add(
		Final_colour,
		vector_add(
		  Reflection,
		  vector_scalar_mult(
		    vector_component_mult(
		      colour_to_vector(Light_colour),
		      Light_contribution),
		    shadow_factor(Light_location, Hit_location, Object, Scene))));
	  (_Not_a_point_light, Final_colour) ->
	      Final_colour
      end,
      #vector{x=0, y=0, z=0},
      Scene).

% returns 0 if Object is occluded from the light at Light_location, otherwise
% returns 1 if light can see Object 
shadow_factor(Light_location, Hit_location, Object, Scene) ->
    Light_vector = vector_sub(Hit_location, Light_location),
    Light_direction = vector_normalize(Light_vector),
    Shadow_ray = #ray{origin=Light_location,
		      direction=Light_direction},
    case nearest_object_intersecting_ray(Shadow_ray, Scene) of
	% this could match another copy of the same object
	{Object, _Distance, _Loc, _Normal} ->
	    1;
	_Else ->
	    0
    end.

% based on
% http://www.devmaster.net/wiki/Lambert_diffuse_lighting
% http://svn.icculus.org/darkwar/trunk/base/shaders/light.frag?rev=1067&view=auto
diffuse_term(Object, Light_location, Hit_location, Hit_normal) ->
    vector_scalar_mult(
      colour_to_vector(object_diffuse_colour(Object)),
      lists:max([0,
		 vector_dot_product(Hit_normal,
				    vector_normalize(
				      vector_sub(Light_location,
						 Hit_location)))])).

% based on
% http://svn.icculus.org/darkwar/trunk/base/shaders/light.frag?rev=1067&view=auto
% http://www.flipcode.com/archives/Raytracing_Topics_Techniques-Part_2_Phong_Mirrors_and_Shadows.shtml
% http://www.devmaster.net/wiki/Phong_shading
specular_term(EyeVector, Light_location, Hit_location, Hit_normal,
	     Specular_power, Shininess, Specular_colour) ->
	    vector_scalar_mult(
	      colour_to_vector(Specular_colour),
	      Shininess*math:pow(
		lists:max([0,
			   vector_dot_product(
			     vector_normalize(
			       vector_add(
				 vector_normalize(
				   vector_sub(Light_location, Hit_location)),
				 vector_neg(EyeVector))),
			     Hit_normal)]), Specular_power)).

% object agnostic intersection function
nearest_object_intersecting_ray(Ray, Scene) ->
    nearest_object_intersecting_ray(
      Ray, none, hitlocation, hitnormal, infinity, Scene).
nearest_object_intersecting_ray(
  _Ray,	_NearestObj, _Hit_location, _Normal, infinity, []) ->
    none;
nearest_object_intersecting_ray(
  _Ray, NearestObj, Hit_location, Normal, Distance, []) ->
%    io:format("intersecting ~w at ~w~n", [NearestObj, Distance]),
    {NearestObj, Distance, Hit_location, Normal};
nearest_object_intersecting_ray(Ray,
				NearestObj,
				Hit_location,
				Normal,
				Distance,
				[CurrentObject|Rest_of_scene]) ->
    case ray_object_intersect(Ray, CurrentObject) of
	{NewDistance, New_hit_location, New_normal} ->
	    %io:format("Distace=~w NewDistace=~w~n", [Distance, NewDistance]),
	    if (Distance == infinity) or (Distance > NewDistance) ->
		    %io:format("another closer object found~n", []),
		    nearest_object_intersecting_ray(
		      Ray,
		      CurrentObject,
		      New_hit_location,
		      New_normal,
		      NewDistance,
		      Rest_of_scene);
	       true ->
		    %io:format("no closer obj found~n", []),
		    nearest_object_intersecting_ray(
		      Ray,
		      NearestObj,
		      Hit_location,
		      Normal,
		      Distance,
		      Rest_of_scene)
	    end;
	none ->
	    nearest_object_intersecting_ray(
	      Ray,
	      NearestObj,
	      Hit_location,
	      Normal,
	      Distance,
	      Rest_of_scene)
    end.

% object specific intersection function
ray_object_intersect(Ray, Object) ->
    case Object of
	#sphere{} ->
	    ray_sphere_intersect(Ray, Object);
	#triangle{} ->
	    ray_triangle_intersect(Ray, Object);
	#plane{} ->
	    ray_plane_intersect(Ray, Object);
	_Else ->
	    none
    end.

% based on
% http://www.devmaster.net/articles/raytracing/
% http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter1.htm
ray_sphere_intersect(
  #ray{origin=#vector{
	 x=X0, y=Y0, z=Z0},
       direction=#vector{
	 x=Xd, y=Yd, z=Zd}},
  #sphere{radius=Radius, center=#vector{
			   x=Xc, y=Yc, z=Zc}}) ->
    Epsilon = 0.001,
    A = Xd*Xd + Yd*Yd + Zd*Zd,
    B = 2 * (Xd*(X0-Xc) + Yd*(Y0-Yc) + Zd*(Z0-Zc)),
    C = (X0-Xc)*(X0-Xc) + (Y0-Yc)*(Y0-Yc) + (Z0-Zc)*(Z0-Zc) - Radius*Radius,
    Discriminant = B*B - 4*A*C,
    %io:format("A=~w B=~w C=~w discriminant=~w~n",
%	      [A, B, C, Discriminant]),
    if Discriminant >= Epsilon ->
	    T0 = (-B + math:sqrt(Discriminant))/2,
	    T1 = (-B - math:sqrt(Discriminant))/2,
	    if (T0 >= 0) and (T1 >= 0) ->
		    %io:format("T0=~w T1=~w~n", [T0, T1]),
		    Distance = lists:min([T0, T1]),
		    Intersection = vector_add(
				     #vector{x=X0, y=Y0, z=Z0},
				     vector_scalar_mult(
				        #vector{x=Xd, y=Yd, z=Zd}, Distance)),
		    Normal = vector_normalize(
			       vector_sub(Intersection,
					  #vector{x=Xc, y=Yc, z=Zc})),
		    {Distance, Intersection, Normal};
	       true ->
		    none
	    end;
       true ->
	    none
    end.

% based on
% http://www.graphics.cornell.edu/pubs/1997/MT97.html
% http://jgt.akpeters.com/papers/GuigueDevillers03/addendum.html
ray_triangle_intersect(Ray, Triangle) ->
    Epsilon = 0.000001,
    
    % find vectors for two edges sharing v1
    Edge1 = vector_sub(Triangle#triangle.v2, Triangle#triangle.v1),
    Edge2 = vector_sub(Triangle#triangle.v3, Triangle#triangle.v1),

    % begin calculating determinant
    P = vector_cross_product(Ray#ray.direction, Edge2),
    Determinant = vector_dot_product(Edge1, P),

    % negative determinant means the triangle is facing away
    % from the ray
    
    if Determinant < Epsilon ->
	    % for our purposes we ignore such triangles
%%  	    io:format("ray is either behind or on the triangle: ~p~n", [Determinant]),
	    none;
       true ->
	    % calculate the distance from v1 to ray origin
	    T = vector_sub(Ray#ray.origin, Triangle#triangle.v1),

	    % calculate the U parameter and test bounds
	    U = vector_dot_product(T, P),
	    if (U < 0) or (U > Determinant) ->
%%  		    io:format("U is negative or greater than det: ~p~n", [U]),
		    none;
	       true ->
		    % prepare to test the V parameter
		    Q = vector_cross_product(T, Edge1),
		    % calculate the V parameter and test bounds
		    V = vector_dot_product(Ray#ray.direction, Q),
		    if (V < 0) or (U+V > Determinant) ->
%%  			    io:format("V less than 0.0 or U+V greater than det: ~p ~p~n",
%%  				      [U, V]),
			    none;
		       true ->
			    % calculate the distance to the
			    % intersection point and return
%%   			    io:format("found ray/triangle intersection ~n", []),
			    Distance = vector_dot_product(Edge2, Q) / Determinant,
			    Intersection = vector_add(
					     Ray#ray.origin,
					     vector_scalar_mult(
					       Ray#ray.direction,
					       Distance)),
			    Normal = vector_normalize(
				       vector_cross_product(
					 Triangle#triangle.v1,
					 Triangle#triangle.v2)),
			    {Distance, Intersection, Normal}
		    end
	    end
    end.

% based on
% http://www.siggraph.org/education/materials/HyperGraph/raytrace/rayplane_intersection.htm
% http://www.cs.princeton.edu/courses/archive/fall00/cs426/lectures/raycast/sld017.htm
% http://www.devmaster.net/articles/raytracing/
ray_plane_intersect(Ray, Plane) ->
    Epsilon = 0.001,
    Vd = vector_dot_product(Plane#plane.normal, Ray#ray.direction),
    if Vd < 0 ->
	    V0 = -(vector_dot_product(Plane#plane.normal, Ray#ray.origin)
		   + Plane#plane.distance),
	    Distance = V0 / Vd,
	    if Distance < Epsilon ->
		    none;
	       true ->
		    Intersection = vector_add(
				     Ray#ray.origin,
				     vector_scalar_mult(
				       Ray#ray.direction,
				       Distance)),
		    {Distance, Intersection, Plane#plane.normal}
	    end;
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
    #ray{origin=From, direction=vector_normalize(vector_sub(Through, From))}.

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

vector_component_mult(#vector{x=X1, y=Y1, z=Z1}, #vector{x=X2, y=Y2, z=Z2}) ->
    #vector{x=X1*X2, y=Y1*Y2, z=Z1*Z2}.

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

% based on
% http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtreflec.htm
% http://www.devmaster.net/articles/raytracing/
vector_bounce_off_plane(Vector, Normal) ->
    vector_add(
      vector_scalar_mult(
	Normal,
	2*vector_dot_product(Normal, vector_neg(Vector))),
      Vector).

object_diffuse_colour(#sphere{material=#material{colour=C}}) ->
    C;
object_diffuse_colour(#plane{material=#material{colour=C}}) ->
    C;
object_diffuse_colour(#triangle{material=#material{colour=C}}) ->
    C.

object_specular_power(#sphere{material=#material{specular_power=SP}}) ->
    SP;
object_specular_power(#plane{material=#material{specular_power=SP}}) ->
    SP;
object_specular_power(#triangle{material=#material{specular_power=SP}}) ->
    SP.

object_shininess(#sphere{material=#material{shininess=S}}) ->
    S;
object_shininess(#plane{material=#material{shininess=S}}) ->
    S;
object_shininess(#triangle{material=#material{shininess=S}}) ->
    S.

object_reflectivity(#sphere{material=#material{reflectivity=R}}) ->
    R;
object_reflectivity(#plane{material=#material{reflectivity=R}}) ->
    R;
object_reflectivity(#triangle{material=#material{reflectivity=R}}) ->
    R.

point_on_sphere(#sphere{radius=Radius, center=#vector{x=XC, y=YC, z=ZC}},
		#vector{x=X, y=Y, z=Z}) ->
    Epsilon = 0.001,
    Epsilon > abs(
      ((X-XC)*(X-XC) + (Y-YC)*(Y-YC) + (Z-ZC)*(Z-ZC)) - Radius*Radius).

colour_to_vector(#colour{r=R, g=G, b=B}) ->
    #vector{x=R, y=G, z=B}.
vector_to_colour(#vector{x=X, y=Y, z=Z}) ->
    #colour{r=X, g=Y, b=Z}.
colour_to_pixel(#colour{r=R, g=G, b=B}) ->
    {R, G, B}.

% returns a list of objects in the scene
% camera is assumed to be the first element in the scene
scene() ->
    [#camera{location=#vector{x=0, y=0, z=-2},
	     rotation=#vector{x=0, y=0, z=0},
	     fov=90,
	     screen=#screen{width=4, height=3}},
     #point_light{diffuse_colour=#colour{r=1, g=1, b=0.5},
		  location=#vector{x=5, y=-2, z=0},
		  specular_colour=#colour{r=1, g=1, b=1}},
     #point_light{diffuse_colour=#colour{r=1, g=0, b=0.5},
		  location=#vector{x=-10, y=0, z=7},
		  specular_colour=#colour{r=1, g=0, b=0.5}},
     #sphere{radius=4,
	     center=#vector{x=4, y=0, z=10},
	     material=#material{
	       colour=#colour{r=0, g=0.5, b=1},
	       specular_power=20,
	       shininess=1,
	       reflectivity=0.1}},
     #sphere{radius=4,
	     center=#vector{x=-5, y=3, z=9},
	     material=#material{
	       colour=#colour{r=1, g=0.5, b=0},
	       specular_power=4,
	       shininess=0.25,
	       reflectivity=0.5}},
     #sphere{radius=4,
	     center=#vector{x=-4.5, y=-2.5, z=14},
	     material=#material{
	       colour=#colour{r=0.5, g=1, b=0},
	       specular_power=20,
	       shininess=0.25,
	       reflectivity=0.7}},
     #triangle{v1=#vector{x=-2, y=5, z=5},
	       v2=#vector{x=4, y=5, z=10},
	       v3=#vector{x=4, y=-5, z=10},
	       material=#material{
		 colour=#colour{r=1, g=0.5, b=0},
		 specular_power=4,
		 shininess=0.25,
		 reflectivity=0.5}},
     #plane{normal=#vector{x=0, y=-1, z=0},
	    distance=5,
	    material=#material{
	      colour=#colour{r=1, g=1, b=1},
	      specular_power=1,
	      shininess=0,
	      reflectivity=0.01}}
    ].

% assumes Pixels are ordered in a row by row fasion
write_pixels_to_ppm(Width, Height, MaxValue, Pixels, Filename) ->
    case file:open(Filename, write) of
	{ok, IoDevice} ->
	    io:format("file opened~n", []),
	    io:format(IoDevice, "P3~n", []),
	    io:format(IoDevice, "~p ~p~n", [Width, Height]),
	    io:format(IoDevice, "~p~n", [MaxValue]),
	    lists:foreach(
	      fun({_Num, {R, G, B}}) ->
		      io:format(IoDevice, "~p ~p ~p ",
				[lists:min([trunc(R*MaxValue), MaxValue]),
				 lists:min([trunc(G*MaxValue), MaxValue]),
				 lists:min([trunc(B*MaxValue), MaxValue])]) end,
	      Pixels),
	    file:close(IoDevice);
	error ->
	    io:format("error opening file~n", [])
    end.

% various invocation style functions
standalone([Width, Height, Filename, Recursion_depth, Strategy]) ->
    standalone(list_to_integer(Width),
	       list_to_integer(Height),
	       Filename,
	       list_to_integer(Recursion_depth),
	       tracing_function(list_to_atom(Strategy))).

standalone(Width, Height, Filename, Recursion_depth, Function) ->
    {Time, _Value} = timer:tc(
		       raytracer,
		       raytrace,
		       [Width,
			Height,
			Filename,
			Recursion_depth,
			Function]),
    io:format("Done in ~w seconds~n", [Time/1000000]),
    halt().

go(Strategy) ->
    raytrace(tracing_function(Strategy)).

go(Width, Height, Filename, Recursion_depth, Strategy) ->
    raytrace(Width, Height, Filename, Recursion_depth,
       tracing_function(Strategy)).

tracing_function(simple) ->
    fun raytraced_pixel_list_simple/4;
tracing_function(concurrent) ->
    fun raytraced_pixel_list_concurrent/4;
tracing_function(distributed) ->
    fun raytraced_pixel_list_distributed/4.

raytrace(Function) ->
    raytrace(4, 3, "/tmp/traced.ppm", 5, Function).
raytrace(Width, Height, Filename, Recursion_depth, Function) ->
    write_pixels_to_ppm(
      Width,
      Height,
      255,
      Function(
	Width,
	Height,
	scene(),
	Recursion_depth),
      Filename).

% testing
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
%	     fun ray_through_pixel_test/0,
	     fun ray_shooting_test/0,
	     fun point_on_screen_test/0,
	     fun nearest_object_intersecting_ray_test/0,
	     fun focal_length_test/0,
%	     fun vector_rotation_test/0,
	     fun vector_bounce_off_plane_test/0,
	     fun ray_sphere_intersection_test/0
	    ],
    run_tests(Tests, 1, true).

scene_test() ->
    io:format("testing the scene function", []),
    case scene() of
	[{camera,
	  {vector, 0, 0, -2},
	  {vector, 0, 0, 0},
	  90,
	  {screen, 4, 3}},
	 {point_light,
	  {colour, 1, 1, 0.5},
	  {vector, 5, -2, 0},
	  {colour, 1, 1, 1}},
	 {point_light,
	  {colour, 1, 0, 0.5},
	  {vector, -10, 0, 7},
	  {colour, 1, 0, 0.5}},
	 {sphere,
	  4,
	  {vector, 4, 0, 10},
	  {material, {colour, 0, 0.5, 1}, 20, 1, 0.1}},
	 {sphere,
	  4,
	  {vector, -5, 3, 9},
	  {material, {colour, 1, 0.5, 0}, 4, 0.25, 0.5}},
	 {sphere,
	  4,
	  {vector, -4.5, -2.5, 14},
	  {material, {colour, 0.5, 1, 0}, 20, 0.25, 0.7}},
	 {triangle,
	  {vector, -2, 5, 5},
	  {vector, 4, 5, 10},
	  {vector, 4, -5, 10},
	  {material, {colour, 1, 0.5, 0}, 4, 0.25, 0.5}},
	 {plane,
	  {vector, 0, -1, 0},
	  5,
	  {material, {colour, 1, 1, 1}, 1, 0, 0.01}}
	] ->
	    true;
_Else ->
	    false
    end.

passing_test() ->
    io:format("this test always passes", []),
    true.

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

ray_shooting_test() ->
    io:format("ray shooting test"),
    Vector1 = #vector{x=0, y=0, z=0},
    Vector2 = #vector{x=1, y=0, z=0},
    
    Subtest1 = vectors_equal(
		 (shoot_ray(Vector1, Vector2))#ray.direction,
		 Vector2),
    
    Subtest1.

ray_sphere_intersection_test() ->
    io:format("ray sphere intersection test", []),

    Sphere = #sphere{
      radius=3,
      center=#vector{x = 0, y=0, z=10},
      material=#material{
      colour=#colour{r=0.4, g=0.4, b=0.4}}},
    Ray1 = #ray{
      origin=#vector{x=0, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    Ray2 = #ray{
      origin=#vector{x=3, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    Ray3 = #ray{
      origin=#vector{x=4, y=0, z=0},
      direction=#vector{x=0, y=0, z=1}},
    {Distance1, _Hit_location1, _Hit_normal1} = ray_sphere_intersect(Ray1, Sphere),
    Subtest1 = Distance1 == 7.0, 
    Subtest2 = ray_sphere_intersect(Ray2, Sphere) == none,
    Subtest3 = ray_sphere_intersect(Ray3, Sphere) == none,
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
    % test to make sure that we really get the closest object
    Sphere1=#sphere{radius=5,
		    center=#vector{x=0, y=0, z=10},
		    material=#material{
		      colour=#colour{r=0, g=0, b=0.03}}},
    Sphere2=#sphere{radius=5,
		    center=#vector{x=0, y=0, z=20},
		    material=#material{
		      colour=#colour{r=0, g=0, b=0.06}}},
    Sphere3=#sphere{radius=5,
		    center=#vector{x=0, y=0, z=30},
		    material=#material{
		      colour=#colour{r=0, g=0, b=0.09}}},
    Sphere4=#sphere{radius=5,
		    center=#vector{x=0, y=0, z=-10},
		    material=#material{
		      colour=#colour{r=0, g=0, b=-0.4}}},
    Scene1=[Sphere1, Sphere2, Sphere3, Sphere4],
    Ray1=#ray{origin=#vector{x=0, y=0, z=0},
	      direction=#vector{x=0, y=0, z=1}},

    {Object1, Distance1, Hit_location, Normal} = nearest_object_intersecting_ray(
				      Ray1, Scene1),
    Subtest1 = (Object1 == Sphere1) and (Distance1 == 5)
	and vectors_equal(Normal, vector_neg(Ray1#ray.direction))
	and point_on_sphere(Sphere1, Hit_location),
    
    Subtest1.

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

vector_bounce_off_plane_test() ->
    io:format("vector reflect about normal", []),
    Vector1 = #vector{x=1, y=1, z=0},
    Vector2 = #vector{x=0, y=-1, z=0},
    Vector3 = #vector{x=1, y=-1, z=0},
    Vector4 = #vector{x=1, y=0, z=0},

    Subtest1 = vectors_equal(vector_bounce_off_plane(
			      Vector1,
			      vector_normalize(Vector2)),
			    Vector3),

    Subtest2 = vectors_equal(
		 vector_bounce_off_plane(
		   Vector2,
		   vector_normalize(Vector1)),
		 Vector4),
    
    Subtest1 and Subtest2.
