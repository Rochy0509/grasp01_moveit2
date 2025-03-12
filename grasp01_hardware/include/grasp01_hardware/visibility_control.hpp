#ifndef GRASP01_HARDWARE_VISIBILITY_CONTROL
#define GRASP01_HARDWARE_VISIBILITY_CONTROL

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRASP01_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define GRASP01_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define GRASP01_HARDWARE_EXPORT __declspec(dllexport)
    #define GRASP01_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRASP01_HARDWARE_BUILDING_LIBRARY
    #define GRASP01_HARDWARE_PUBLIC GRASP01_HARDWARE_EXPORT
  #else
    #define GRASP01_HARDWARE_PUBLIC GRASP01_HARDWARE_IMPORT
  #endif
  #define GRASP01_HARDWARE_PUBLIC_TYPE GRASP01_HARDWARE_PUBLIC
  #define GRASP01_HARDWARE_LOCAL
#else
  #define GRASP01_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define GRASP01_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define GRASP01_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define GRASP01_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRASP01_HARDWARE_PUBLIC
    #define GRASP01_HARDWARE_LOCAL
  #endif
  #define GRASP01_HARDWARE_PUBLIC_TYPE
#endif

#endif  // GRASP01_HARDWARE_VISIBILITY_CONTROL