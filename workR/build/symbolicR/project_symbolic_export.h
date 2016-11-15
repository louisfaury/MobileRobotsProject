
#ifndef PROJECT_SYMBOLIC_EXPORT_H
#define PROJECT_SYMBOLIC_EXPORT_H

#ifdef PROJECT_SYMBOLIC_STATIC_DEFINE
#  define PROJECT_SYMBOLIC_EXPORT
#  define PROJECT_SYMBOLIC_NO_EXPORT
#else
#  ifndef PROJECT_SYMBOLIC_EXPORT
#    ifdef Project_symbolic_EXPORTS
        /* We are building this library */
#      define PROJECT_SYMBOLIC_EXPORT 
#    else
        /* We are using this library */
#      define PROJECT_SYMBOLIC_EXPORT 
#    endif
#  endif

#  ifndef PROJECT_SYMBOLIC_NO_EXPORT
#    define PROJECT_SYMBOLIC_NO_EXPORT 
#  endif
#endif

#ifndef PROJECT_SYMBOLIC_DEPRECATED
#  define PROJECT_SYMBOLIC_DEPRECATED __attribute__ ((__deprecated__))
#  define PROJECT_SYMBOLIC_DEPRECATED_EXPORT PROJECT_SYMBOLIC_EXPORT __attribute__ ((__deprecated__))
#  define PROJECT_SYMBOLIC_DEPRECATED_NO_EXPORT PROJECT_SYMBOLIC_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define PROJECT_SYMBOLIC_NO_DEPRECATED
#endif

#endif
