-module(raytracer).
-include_lib("esdl/include/sdl.hrl").
-include_lib("esdl/include/sdl_video.hrl").
-include_lib("esdl/include/sdl_events.hrl").
%-include_lib("/home/plouj/programming/sdl_util.hrl").
-export([go/0]).

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
    main_loop(Screen),
    timer:sleep(1000),
    sdl:quit().

main_loop(Screen) ->
    Event = sdl_events:pollEvent(),

    case Event of
	{quit} ->
	    done;
	_Else ->
	    putRandomPixel(Screen),
	    sdl_video:flip(Screen),
	    %timer:sleep(1),
	    main_loop(Screen)
    end.

putPixel(Surface, X, Y, {Red, Green, Blue}) ->
    io:format("putting pixel ~w at ~w,~w~n",[{Red, Green, Blue}, X, Y]), 
    %PixelFormat = sdl_video:getPixelFormat(Surface),
    %OriginalPixels = sdl_video:getPixels(Surface),
    sdl_video:fillRect(Surface,
		       #sdl_rect{x = X, y = Y, w = 1, h = 1},
		       sdl_video:mapRGB(Surface, Red, Green, Blue)).

putRandomPixel(Surface) ->
    putPixel(Surface,
	     random:uniform(Surface#sdl_surface.w)-1,
	     random:uniform(Surface#sdl_surface.h)-1,
	     {random:uniform(256)-1, random:uniform(256)-1, random:uniform(256)-1}).
