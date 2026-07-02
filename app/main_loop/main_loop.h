/*
 * app/main_loop — orchestrator.
 *
 * See app/main_loop/README.md for the contract.
 */

#ifndef APP_MAIN_LOOP_H
#define APP_MAIN_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

void main_loop_init(void);   /* call once after CubeMX peripheral init */
void main_loop_run(void);    /* never returns */

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_LOOP_H */
