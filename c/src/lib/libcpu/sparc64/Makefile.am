ACLOCAL_AMFLAGS = -I ../../../aclocal

include $(top_srcdir)/../../../automake/compile.am

noinst_PROGRAMS =

# NOTE: shared applies to targets meant to run with supervisor privilege,
# i.e. sun4u or sun4v. Any targets (e.g. simulators) that cannot emulate 
# supervisor privilege should use their own score and sparc64-syscall.
# The other shared code is probably usable, but should be defined separately.
# See the example at the end of Makefile.am.
if shared

noinst_PROGRAMS += shared/shared-score.rel
shared_shared_score_rel_SOURCES = shared/score/cpu.c shared/score/interrupt.S
shared_shared_score_rel_CPPFLAGS = $(AM_CPPFLAGS)
shared_shared_score_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

noinst_PROGRAMS += shared/sparc64-syscall.rel
shared_sparc64_syscall_rel_SOURCES = shared/syscall/sparc64-syscall.S \
  shared/syscall/sparc64-syscall.h
shared_sparc64_syscall_rel_CPPFLAGS = $(AM_CPPFLAGS)
shared_sparc64_syscall_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
endif

### This is an example of how to define a separate score implementation.
## NOTE: Unlike other CPUs, we install into a subdirectory to avoid
##       file name conflicts (See sh CPU for simexampleilar approach)
#include_sparc64dir = $(includedir)/sparc64
#include_rtems_scoredir = $(includedir)/rtems/score
#
#if simexample
#include_sparc64_HEADERS =  
#include_rtems_score_HEADERS = $(include_rtems_scoredir)/sparc64.h \
#                              $(include_rtems_scoredir)/cpu.h \
#                              $(include_rtems_scoredir)/types.h

## simexample/score
#noinst_PROGRAMS += simexample/score.rel
#simexample_score_rel_SOURCES = 
#simexample_score_rel_CPPFLAGS = $(AM_CPPFLAGS)
#simexample_score_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
#
#endif
### End of example.

include $(top_srcdir)/../../../automake/local.am
