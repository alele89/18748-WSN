#ifndef ZB_INTERDEMO_H
#define ZB_INTERDEMO_H 1

#include "zb_types.h"
#include "zb_scheduler.h"

#define ZB_SDCC_REENTRANT
#define ZB_CALLBACK
#define ZB_SCHEDULER_Q_SIZE 16
#define ZB_BUF_Q_SIZE 16

zb_sched_globals_t sched;
#endif
