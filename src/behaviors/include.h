/**
 * @file include.h
 * @brief File that must be included by all behaviors.
 */

#ifndef INCLUDE_H
#define INCLUDE_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


#define ALWAYS_INLINE __attribute__((always_inline)) static inline
#define PACKED __attribute__((packed))
#define cli()
#define sei()

void open_resources();
void do_meta_stuff();


#define STEPS_TO_LOG 100

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !INCLUDE_H