/**
 *
 * @file cli.h
 * @author Tom Klijn
 * @brief Header of command line interface
 * @date 05-11-2019
 *
 */

#ifndef CLI
#define CLI

#include <stdint.h>

/* CLI configuration */
#define EOL_LENGHT 1 //number of chars that make up the EOL sequence
#define EOL_SEQUENCE "\n"

#define CMD_LIST_SIZE 25 //maximum number of commands
#define CMD_RESPONSE_SIZE 200 //Maximum callback response size

/* CLI Functions */
void CLI_AddCmd(const char *cmdName, const char *cmdDesc, uint8_t paramCnt, void *callback);
uint8_t CLI_RunCmd(char *cmdString, uint8_t size);
uint8_t CLI_Help(void);

#endif /* CLI */
