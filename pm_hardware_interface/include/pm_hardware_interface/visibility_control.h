#ifndef PM_SYSTEM_VISIBILITY_CONTROL_H
#define PM_SYSTEM_VISIBILITY_CONTROL_H

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PM_SYSTEM_EXPORT __attribute__((dllexport))
#define PM_SYSTEM_IMPORT __attribute__((dllimport))
#else
#define PM_SYSTEM_EXPORT __declspec(dllexport)
#define PM_SYSTEM_IMPORT __declspec(dllimport)
#endif
#ifdef PM_SYSTEM_BUILDING_DLL
#define PM_SYSTEM_PUBLIC PM_SYSTEM_EXPORT
#else
#define PM_SYSTEM_PUBLIC PM_SYSTEM_IMPORT
#endif
#define PM_SYSTEM_PUBLIC_TYPE PM_SYSTEM_PUBLIC
#define PM_SYSTEM_LOCAL
#else
#define PM_SYSTEM_EXPORT __attribute__((visibility("default")))
#define PM_SYSTEM_IMPORT
#if __GNUC__ >= 4
#define PM_SYSTEM_PUBLIC __attribute__((visibility("default")))
#define PM_SYSTEM_LOCAL __attribute__((visibility("hidden")))
#else
#define PM_SYSTEM_PUBLIC
#define PM_SYSTEM_LOCAL
#endif
#define PM_SYSTEM_PUBLIC_TYPE
#endif

#endif // PM_SYSTEM_VISIBILITY_CONTROL_H
