#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])
AC_INIT([creatures], [0.1], [bug-report@address])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([-Wall -Werror foreign])

# Checks for programs.
AC_PROG_CC
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_CACHE_SAVE

# Checks for libraries.
AX_CHECK_GL
AX_CHECK_GLU
AX_CHECK_GLUT

AC_CHECK_LIB(ode, dWorldSetGravity, 
                   ODE_LIBS="-lode -ldrawstuff"; AC_SUBST(ODE_LIBS), 
                   AC_MSG_ERROR([libode not found.]), )

AC_CHECK_LIB(drawstuff, dsDrawBoxD, 
                   DRAWSTUFF_LIBS="-lm -framework OpenGL -framework GLUT -framework AGL -framework Carbon -lstdc++ -ldrawstuff"; AC_SUBST(DRAWSTUFF_LIBS), 
                   AC_MSG_ERROR([libdrawstuff not found.]), -lm -framework OpenGL -framework GLUT -framework AGL -framework Carbon -lstdc++)
#$GL_LIBS $GLU_LIBS $GLUT_LIBS)

AC_CHECK_LIB(json, json_object_new_object, 
                   JSON_LIBS="-ljson"; AC_SUBST(JSON_LIBS), 
                   AC_MSG_ERROR([libjson not found.]), )

AC_PATH_GENERIC(alps, , , 
                AC_MSG_ERROR([alps-config not found]), 
                alps-config, --cflags, --ldflags)

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([stdio.h])
AC_CHECK_HEADERS([ode/ode.h])
AC_CHECK_HEADERS([json/json.h])
AC_CHECK_HEADERS([drawstuff/drawstuff.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_CHECK_FUNCS([puts])

# Output files.
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_FILES([ Makefile
                  src/Makefile 
                  support/Makefile
                  support/creatures-config
                  ])
AC_OUTPUT
