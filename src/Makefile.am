LDFLAGS += $(ODE_LIBS) $(JSON_LIBS) 
if DRAWSTUFF
CPPFLAGS += -DDRAWSTUFF
LDFLAGS  += $(DRAWSTUFF_LIBS)
endif

noinst_LTLIBRARIES = libanimat.la
libanimat_la_SOURCES = animat.h animat.cpp

LDADD = libanimat.la
bin_PROGRAMS = ssga grab sphere run runalps
if DRAWSTUFF 
bin_PROGRAMS += readgrab readspheric visual readrun
endif
run_SOURCES = run.cpp 
runalps_SOURCES = runalps.cpp  individ_animat.h
runalps_CPPFLAGS = $(CPPFLAGS) $(ALPS_CFLAGS)
runalps_LDFLAGS = $(LDFLAGS) $(ALPS_LIBS)
ssga_SOURCES = ssga.cpp 
grab_SOURCES = grab.cpp 
sphere_SOURCES = sphere.cpp 

if DRAWSTUFF
readspheric_SOURCES = readspheric.cpp 
readspheric_LDFLAGS = $(LDFLAGS) 
visual_SOURCES = visual.cpp 
visual_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readrun_SOURCES = readrun.cpp 
readrun_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readgrab_SOURCES = readgrab.cpp 
readgrab_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
endif
#hello_SOURCES = main.c
#hello_LDADD = libhello.la

# lib_LTLIBRARIES = libhello.la
# libhello_la_SOURCES = hello.c
# libhello_la_LDFLAGS = -release $(PACKAGE_VERSION)

# include_HEADERS = hello.h
