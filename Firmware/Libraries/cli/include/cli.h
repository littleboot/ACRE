/**
 *
 * @file cli.h
 * @author Tom Klijn
 * @brief Header of command line interface
 * @date 05-11-2019
 *
 */
#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CLI_H
#define CLI_H

/* ========================== include files =========================== */
#include <stdint.h>

/* ============================ constants ============================= */
#define EOL_LENGHT 1 /* change EOL sequence lenght */ 
#define EOL_SEQUENCE "\n"

#define CMD_LIST_SIZE 10 /* Number of commands, reduce to save on flash */
#define CMD_RESPONSE_SIZE 100 /* Maximum callback response string size */

/* ======================= public functions =========================== */
void cli_addCmd(const char *cmdName, const char *cmdDesc, uint8_t paramCnt, void *callback);
uint8_t cli_runCmd(char *cmdString, uint8_t size);
uint8_t cli_help(void);

#endif /* CLI_H */
#ifdef __cplusplus
}
#endif
