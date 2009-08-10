LDFLAGS += $(ODE_LIBS) $(JSON_LIBS)

bin_PROGRAMS = ssga grab sphere run readbest readgrab readspheric visual readrun
run_SOURCES = run.cpp
ssga_SOURCES = ssga.cpp
grab_SOURCES = grab.cpp
sphere_SOURCES = sphere.cpp

readbest_SOURCES = readbest.cpp
readbest_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
readspheric_SOURCES = readspheric.cpp
readspheric_LDFLAGS = $(LDFLAGS) $(DRAWSTUFF_LIBS)
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
