## Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([rtems-c-src-lib-libbsp-arm-board9x25],[_RTEMS_VERSION],[https://devel.rtems.org/newticket])
AC_CONFIG_SRCDIR([bsp_specs])
RTEMS_TOP(../../../../../..)

RTEMS_CANONICAL_TARGET_CPU
AM_INIT_AUTOMAKE([no-define nostdinc foreign 1.12.2])
RTEMS_BSP_CONFIGURE

RTEMS_PROG_CC_FOR_TARGET
RTEMS_CANONICALIZE_TOOLS
RTEMS_PROG_CCAS

RTEMS_CHECK_NETWORKING
AM_CONDITIONAL(HAS_NETWORKING,test "$HAS_NETWORKING" = "yes")

RTEMS_BSPOPTS_SET([ENABLE_UMON],[*],[1])
RTEMS_BSPOPTS_HELP([ENABLE_UMON],
[If defined, enable use of the uMon console.])
AM_CONDITIONAL(ENABLE_UMON,test "$ENABLE_UMON" = "1")

RTEMS_BSPOPTS_SET([ENABLE_UMON_CONSOLE],[*],[1])
RTEMS_BSPOPTS_HELP([ENABLE_UMON_CONSOLE],
[If defined, enable use of the MicroMonitor console device.])
AM_CONDITIONAL(ENABLE_UMON_CONSOLE,test "$ENABLE_UMON_CONSOLE" = "1")

## If ENABLE_UMON is 0 and ENABLE_UMON_CONSOLE is 1, then it is an error.
## TODO: Catch this error.

## RTEMS_BSPOPTS_SET([ENABLE_USART0],[*],[1])
## RTEMS_BSPOPTS_HELP([ENABLE_USART0],
## [If defined, enable use of the USART 0.])
## AM_CONDITIONAL(ENABLE_USART0,test "$ENABLE_USART0" = "1")

RTEMS_BSP_CLEANUP_OPTIONS(1, 1)
RTEMS_BSP_LINKCMDS

# Explicitly list all Makefiles here
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
