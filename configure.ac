#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.61])
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

PKG_CHECK_MODULES([ODE], [ode])
AC_SUBST(ODE_CFLAGS)
AC_SUBST(ODE_LIBS)

PKG_CHECK_MODULES([JSON], [json])
AC_SUBST(JSON_CFLAGS)
AC_SUBST(JSON_LIBS)
dnl AC_CHECK_LIB(ode, dWorldSetGravity, 
dnl                    ODE_LIBS="-lode"; AC_SUBST(ODE_LIBS), 
dnl                    AC_MSG_ERROR([libode not found.]), )

OS=`uname -s`
AC_SUBST(OS)
case $OS in
     Darwin)
        #DRAWSTUFF_DEP="-lm -framework OpenGL -framework GLUT -framework AGL -framework Carbon -lstdc++";
        DRAWSTUFF_DEP="$GL_LIBS $GLU_LIBS $GLUT_LIBS -lstdc++ -lX11";
        ;;
     Linux)
        DRAWSTUFF_DEP="$GL_LIBS $GLU_LIBS $GLUT_LIBS";
        ;;
        *)
        DRAWSTUFF_DEP="$GL_LIBS $GLU_LIBS $GLUT_LIBS";
        ;;
esac

DRAWSTUFF_LIBS=""
AC_CHECK_LIB(drawstuff, dsDrawBoxD, 
                   DRAWSTUFF_LIBS="$DRAWSTUFF_DEP -ldrawstuff"; AC_SUBST(DRAWSTUFF_LIBS), 
                   AC_MSG_WARN([libdrawstuff not found.]), $DRAWSTUFF_DEP)

AM_CONDITIONAL([DRAWSTUFF], [test -n "$DRAWSTUFF_LIBS"])

dnl AC_CHECK_LIB(json, json_object_new_object, 
dnl                    JSON_LIBS="-ljson"; AC_SUBST(JSON_LIBS), 
dnl                    AC_MSG_ERROR([libjson not found.]), )

AC_PATH_GENERIC(alps, , , 
                 AC_MSG_ERROR([alps-config not found]), 
                 alps-config, --cflags, --ldflags)

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([stdio.h])
AC_CHECK_HEADERS([ode/ode.h])
AC_CHECK_HEADERS([json/json.h])
#AC_CHECK_HEADERS([drawstuff/drawstuff.h])

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
