/*
at91sam9xx5
baconxu@gmail.com
*/

#ifndef __TRACE_H__
#define __TRACE_H__

/* Empty macro */
#define TRACE_DEBUG(...)
#define TRACE_INFO(...)
#define TRACE_WARNING(...)
#define TRACE_ERROR(...)
#define TRACE_FATAL(...)      { while(1); }

#define TRACE_DEBUG_WP(...)
#define TRACE_INFO_WP(...)
#define TRACE_WARNING_WP(...)
#define TRACE_ERROR_WP(...)
#define TRACE_FATAL_WP(...)   { while(1); }

#endif



