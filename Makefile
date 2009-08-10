CC = g++

#ODE_DIR = /home/shane/.sbcl/site/cl-ode_original/ode-0.8
ODE_DIR = /home/shane


#CXXFLAGS = -Wall -fno-rtti -fno-exceptions -save-temps -I $(ODE_DIR)/include/ -ggdb
CXXFLAGS = -Wall -fno-rtti -fno-exceptions -I $(ODE_DIR)/include/ -ggdb

#LDFLAGS = $(ODE_DIR)/ode/src/libode.a $(ODE_DIR)/drawstuff/src/libdrawstuff.a -lm -framework OpenGL -framework GLUT -framework AGL -framework Carbon -lobjc -ljson
LDFLAGS = $(HOME)/lib/libode.a -L $(HOME)/lib -ljson -lode

#FILES = ssga readbest readgrab readspheric visual grab sphere run readrun
FILES = ssga grab sphere run 

all: $(FILES)

clean:
	$(RM) $(FILES)

tags:
	etags *.c *.cpp *.cc *.h

REPO = $(HOME)/src/creatures

push:
	darcs push -v $(REPO)

pull:
	darcs pull -v $(REPO)
