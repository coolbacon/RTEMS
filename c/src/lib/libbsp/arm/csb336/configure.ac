## Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([rtems-c-src-lib-libbsp-arm-csb336],[_RTEMS_VERSION],[https://devel.rtems.org/newticket])
AC_CONFIG_SRCDIR([make/custom/csb336.cfg])
RTEMS_TOP(../../../../../..)
RTEMS_SOURCE_TOP
RTEMS_BUILD_TOP
RTEMS_BSP_LINKCMDS

RTEMS_CANONICAL_TARGET_CPU
AM_INIT_AUTOMAKE([no-define nostdinc foreign 1.12.2])
RTEMS_BSP_CONFIGURE

RTEMS_PROG_CC_FOR_TARGET
RTEMS_CANONICALIZE_TOOLS
RTEMS_PROG_CCAS

RTEMS_CHECK_NETWORKING
AM_CONDITIONAL(HAS_NETWORKING,test "$HAS_NETWORKING" = "yes")

RTEMS_BSP_CLEANUP_OPTIONS

# Explicitly list all Makefiles here
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
