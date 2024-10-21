INCFLAGS  = -I /usr/include/GL
INCFLAGS += -I ../shared/vecmath/output/vecmath

LINKFLAGS = -lglut -lGL -lGLU
LINKFLAGS += -L ../shared/vecmath/output -lvecmath

CFLAGS    = -O2 -ggdb -Wall -ansi -DSOLN 
CC        = g++
SRCS      = main.cpp parse.cpp curve.cpp surf.cpp camera.cpp
OBJS      = $(SRCS:.cpp=.o)
PROG      = a1

all: $(SRCS) $(PROG)

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LINKFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -c -o $@ $(INCFLAGS)

depend:
	makedepend $(INCFLAGS) -Y $(SRCS)

clean:
	rm $(OBJS) $(PROG)
