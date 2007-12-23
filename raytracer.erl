-module(raytracer).
-include_lib("esdl/include/sdl.hrl").
-include_lib("esdl/include/sdl_video.hrl").
-include_lib("esdl/include/sdl_events.hrl").
%-include_lib("/home/plouj/programming/sdl_util.hrl").
%-export([go/0]).
-compile(export_all).
go() ->
    go([]).
go(Config) ->
    sdl:init(?SDL_INIT_VIDEO),
    %sdl_util:debug(1),
    
    Flags = case lists:member(fullscreen, Config) of
		true ->
		    ?SDL_ANYFORMAT bor ?SDL_FULLSCREEN bor ?SDL_RESIZABLE;
		_ ->
		    ?SDL_ANYFORMAT bor ?SDL_RESIZABLE
	    end,
    ScreenRef = sdl_video:setVideoMode(640, 480, 32, Flags),
    sdl_video:wm_setCaption(["Erlang SDL"], []),
    %io:format("Video Driver Name: ~s~n", [sdl_video:videoDriverName()]),
    Screen=sdl_video:getSurface(ScreenRef),
    {R1, R2, R3} = erlang:now(),
    random:seed(R1, R2, R3),
%    sdl_video:fillRect(Screen, null, sdl_video:mapRGB(Screen, 0, 128, 255)),
%    io:format("pixels: ~p~n", [sdl_video:getPixels(Screen,
%						  #sdl_rect{x = 0, y = 0,
%							   w = Screen#sdl_surface.w,
%							   h = Screen#sdl_surface.h})]),
%    fill_screen_randomly(Screen),
    sdl_video:blitSurface(trace_image(Screen), null, Screen, null),
    main_loop(Screen),
    sdl:quit().

main_loop(Screen) ->
    Event = sdl_events:pollEvent(),

    case Event of
	{quit} ->
	    done;
	_Else ->
	    %fill_screen_randomly(Screen),
	    %putRandomPixels(Screen),
	    %sdl_video:flip(Screen),
	    timer:sleep(10),
	    main_loop(Screen)
    end.

trace_image(Surface) ->
    PixelFormat = sdl_video:getPixelFormat(Surface),
    sdl_video:createRGBsurfaceFrom(raytrace(Surface#sdl_surface.w, Surface#sdl_surface.h, scene()),
			 Surface#sdl_surface.w,
			 Surface#sdl_surface.h,
			 PixelFormat#sdl_pixelformat.bitsPerPixel,
			 Surface#sdl_surface.pitch,
			 PixelFormat#sdl_pixelformat.rmask,
			 PixelFormat#sdl_pixelformat.gmask,
			 PixelFormat#sdl_pixelformat.bmask,
			 PixelFormat#sdl_pixelformat.amask).

raytrace(W, H, Scene) ->
    pixel_binary_from_list(
      lists:map(
	fun(ScreenCoordinates) -> trace_ray_from_pixel(Scene, ScreenCoordinates) end,
		     pixel_coordinates(W, H))).

% convert a list of pixels into a binary suitable for creating an SDL surface
pixel_binary_from_list(PixelList) ->
    false.

trace_ray_from_pixel(Scene, {X, Y}) ->
    false.

pixel_coordinates(Width, Height) when Width > 0, Height > 0 ->
    lists:flatmap(fun(Y) ->
			  lists:map(fun(X) ->
					    {X, Y} end,
				    lists:seq(0, Width - 1)) end,
		  lists:seq(0, Height - 1)).

% returns a list of objects in the scene
scene() ->
    [].


fill_screen_randomly(Screen) ->
    fill_screen_rows_randomly(Screen, Screen#sdl_surface.h).

fill_screen_rows_randomly(_Screen, 0) ->
    done;
fill_screen_rows_randomly(Screen, Rows) ->
    fill_screen_columns_randomly(Screen, Rows, Screen#sdl_surface.w),
    fill_screen_rows_randomly(Screen, Rows - 1).

fill_screen_columns_randomly(_Screen, _Rows, 0) ->
    done;
fill_screen_columns_randomly(Screen, Rows, Columns) ->
    putRandomPixelAt(Screen, Columns - 1, Rows - 1),
    sdl_video:flip(Screen),
    fill_screen_columns_randomly(Screen, Rows, Columns - 1).


putPixel(Surface, X, Y, {Red, Green, Blue}) ->
    %io:format("putting pixel ~w at ~w,~w~n",[{Red, Green, Blue}, X, Y]), 
    %PixelFormat = sdl_video:getPixelFormat(Surface),
    %OriginalPixels = sdl_video:getPixels(Surface),
    sdl_video:fillRect(Surface,
		       #sdl_rect{x = X, y = Y, w = 1, h = 1},
		       sdl_video:mapRGB(Surface, Red, Green, Blue)).

putRandomPixels(Surface) ->
    putRandomPixelAt(Surface, random:uniform(Surface#sdl_surface.w)-1,
		     random:uniform(Surface#sdl_surface.h)-1).

putRandomPixelAt(Surface, X, Y) ->
    putPixel(Surface, X, Y,
	     {random:uniform(256)-1, random:uniform(256)-1, random:uniform(256)-1}).
