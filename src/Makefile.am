LDFLAGS += $(ODE_LIBS) $(JSON_LIBS) $(DRAWSTUFF_LIBS)
CPPFLAGS += -DDRAWSTUFF

noinst_LTLIBRARIES = libanimat.la
libanimat_la_SOURCES = animat.h animat.cpp

LDADD = libanimat.la
bin_PROGRAMS = ssga grab sphere run readgrab readspheric visual readrun runalps
run_SOURCES = run.cpp 
runalps_SOURCES = runalps.cpp  individ_animat.h
runalps_CPPFLAGS = $(CPPFLAGS) $(ALPS_CFLAGS)
runalps_LDFLAGS = $(LDFLAGS) $(ALPS_LIBS)
ssga_SOURCES = ssga.cpp 
grab_SOURCES = grab.cpp 
sphere_SOURCES = sphere.cpp 

#readbest_SOURCES = readbest.cpp
#readbest_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readspheric_SOURCES = readspheric.cpp 
readspheric_LDFLAGS = $(LDFLAGS) 
visual_SOURCES = visual.cpp 
visual_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readrun_SOURCES = readrun.cpp 
readrun_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readgrab_SOURCES = readgrab.cpp 
readgrab_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)

#hello_SOURCES = main.c
#hello_LDADD = libhello.la

# lib_LTLIBRARIES = libhello.la
# libhello_la_SOURCES = hello.c
# libhello_la_LDFLAGS = -release $(PACKAGE_VERSION)

# include_HEADERS = hello.h
