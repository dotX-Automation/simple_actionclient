#ifndef SIMPLE_ACTIONCLIENT__VISIBILITY_CONTROL_H_
#define SIMPLE_ACTIONCLIENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIMPLE_ACTIONCLIENT_EXPORT __attribute__ ((dllexport))
    #define SIMPLE_ACTIONCLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define SIMPLE_ACTIONCLIENT_EXPORT __declspec(dllexport)
    #define SIMPLE_ACTIONCLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIMPLE_ACTIONCLIENT_BUILDING_LIBRARY
    #define SIMPLE_ACTIONCLIENT_PUBLIC SIMPLE_ACTIONCLIENT_EXPORT
  #else
    #define SIMPLE_ACTIONCLIENT_PUBLIC SIMPLE_ACTIONCLIENT_IMPORT
  #endif
  #define SIMPLE_ACTIONCLIENT_PUBLIC_TYPE SIMPLE_ACTIONCLIENT_PUBLIC
  #define SIMPLE_ACTIONCLIENT_LOCAL
#else
  #define SIMPLE_ACTIONCLIENT_EXPORT __attribute__ ((visibility("default")))
  #define SIMPLE_ACTIONCLIENT_IMPORT
  #if __GNUC__ >= 4
    #define SIMPLE_ACTIONCLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define SIMPLE_ACTIONCLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIMPLE_ACTIONCLIENT_PUBLIC
    #define SIMPLE_ACTIONCLIENT_LOCAL
  #endif
  #define SIMPLE_ACTIONCLIENT_PUBLIC_TYPE
#endif

#endif  // SIMPLE_ACTIONCLIENT__VISIBILITY_CONTROL_H_
