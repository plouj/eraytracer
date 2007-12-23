-module(raytracer).
-include_lib("esdl/include/sdl.hrl").
-include_lib("esdl/include/sdl_video.hrl").
-include_lib("esdl/include/sdl_events.hrl").
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
    %io:format("Video Driver Name: ~s~n", [sdl_video:videoDriverName()]),
    Screen=sdl_video:getSurface(ScreenRef),
    {R1, R2, R3} = erlang:now(),
    random:seed(R1, R2, R3),
    main_loop(),
    timer:sleep(1000),
    sdl:quit().

main_loop() ->
    Event = sdl_events:pollEvent(),

    case Event of
	{quit} ->
	    done;
	_Else ->
	    timer:sleep(100),
	    main_loop()
    end.

