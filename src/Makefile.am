LDFLAGS += $(ODE_LIBS) $(JSON_LIBS)

bin_PROGRAMS = ssga grab sphere run readgrab readspheric visual readrun runalps
run_SOURCES = run.cpp debug.c
runalps_SOURCES = runalps.cpp debug.c individ_animat.h
runalps_CPPFLAGS = $(CPPFLAGS) $(ALPS_CFLAGS)
runalps_LDFLAGS = $(LDFLAGS) $(ALPS_LIBS)
ssga_SOURCES = ssga.cpp debug.c
grab_SOURCES = grab.cpp debug.c
sphere_SOURCES = sphere.cpp debug.c

#readbest_SOURCES = readbest.cpp
#readbest_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readspheric_SOURCES = readspheric.cpp debug.c
readspheric_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
visual_SOURCES = visual.cpp debug.c
visual_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readrun_SOURCES = readrun.cpp debug.c
readrun_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readgrab_SOURCES = readgrab.cpp debug.c
readgrab_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)

#hello_SOURCES = main.c
#hello_LDADD = libhello.la

# lib_LTLIBRARIES = libhello.la
# libhello_la_SOURCES = hello.c
# libhello_la_LDFLAGS = -release $(PACKAGE_VERSION)

# include_HEADERS = hello.h
