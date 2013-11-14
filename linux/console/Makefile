SDLLIBS  = $(shell sdl-config --libs) 
SDLFLAGS = $(shell sdl-config --cflags)


a: simple_console.cpp
	g++ -o simple_console simple_console.cpp -lSDL -lncurses -Wall $(SDLLIBS) $(SDLFLAGS)
