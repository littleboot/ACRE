/**
 *
 * @file cli.c
 * @author Tom Klijn
 * @brief Implementation of command line interface
 * @date 05-11-2019
 *
 */
/* ========================== include files =========================== */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "cli.h"

/* =========================== data types ================+++========== */
typedef struct cmd
{
    const char *name; //Command name
    const char *cmdDesc; //Command description
    uint8_t paramCnt; //Command function parameter count, minus the default response param
    void *callback;
} cmd;

/* ======================== global variables ========================== */
static char eolSeq[] = EOL_SEQUENCE;
static uint8_t cmdCnt = 0; //count of added cmd's
static cmd cmdList[CMD_LIST_SIZE];

/* ============================ functions ============================= */
/**
 * @brief Adds's cmd to the cmd list
 * 
 * @param cmdName String name of the command (example HELP)
 * @param cmdDesc String description 
 * @param paramCnt The amount of parameters the cmd expects
 */
void cli_addCmd(const char *cmdName, const char *cmdDesc, uint8_t paramCnt, void *callback)
{
    // static_assert(cmdCnt < CMD_LIST_SIZE);

    cmdList[cmdCnt].name = cmdName;
    cmdList[cmdCnt].cmdDesc = cmdDesc;
    cmdList[cmdCnt].paramCnt = paramCnt;
    cmdList[cmdCnt].callback = callback;

    cmdCnt++;
}

/**
 * @brief Validate input buffer if its conform the cmd format
 *        <cmdname>' '<param1>' '<param2>\n
 * 
 * @param buff char buffer
 * @param size size cmd string
 * @return uint8_t errorNbr
 */
static uint8_t cli_validateSyntax(char *buff, uint8_t size)
{
    uint8_t eolCharCnt = 0;
    //Check if correct amount of EOL chars are present in the buffer. just one of each
    for (uint8_t i = 0; i < EOL_LENGHT; i++)
    {
        for (uint8_t j = 0; j < size; j++)
        {
            if (eolSeq[i] == buff[j])
            {
                eolCharCnt++;
                if (eolCharCnt > EOL_LENGHT)
                {
                    return 1;
                    // printf("ERROR EOL char count %i not oke\n", eolCharCnt);
                    // break;
                }
            }
        }
    }
    //Check for space on first location, this indicates an error
    //TODO

    //check for double spaces, this indicates an error
    //TODO
    return 0; //comamand format valid
}

/**
 * @brief Executes command 
 * 
 * @param cmdString 
 * @param size 
 * @return uint8_t 
 */
uint8_t cli_runCmd(char *cmdString, uint8_t size)
 {
	char response[CMD_RESPONSE_SIZE] = { '\0' }; //stores callback response

	// check cmd format matches: <cmdname>' '<param1>' '<param2>\n
	if (cli_validateSyntax(cmdString, size) != 0) {
		printf("NACK: Invalid cmd syntax '%s'\n", cmdString);
		return 1;
	}

	/* Create string from cmd input, needed for string functions*/
	char *eolLoc = strstr(cmdString, eolSeq);
	if (eolLoc != NULL) {
		*eolLoc = '\0'; //make string from input buffer
	} else {
		printf("NACK: Invalid cmd syntax EOL not detected\n");
		return 2;
	}

	// Split cmd string into cmd and parameter strings
	char *cmdName = cmdString;
	char *param1 = NULL;
	char *endP1 = NULL;
	char *param2 = NULL;
	uint8_t paramCnt = 0;

	//count parameters, convert to string and save location
	param1 = strchr(cmdName, ' '); //If there is a space, a cmd follows after this space
	if (param1 != NULL) {
		paramCnt = 1;
		*param1 = '\0'; //replace space with end of string, end of the cmdName String
		param1++;       //location of the first char of param1

		//If param1, first char of par1 == '"', the parameter has spaces in it, the string ends at next '"'
		if (*param1 == '"') {
			//the param1 string begins at the next char after the "
			param1++;

			endP1 = strchr(param1, '"'); //Find the end of param1 string
			if (endP1 != NULL) {
				*endP1 = '\0'; //replace the " with \0 Marking the end of the param1 string
				if (*(endP1 + 1) == ' ') {
					//a space after param1 indicates a second parameter is present
					param2 = endP1 + 1; //The second parameter starts 1 char after this space
				}
			} else {
				//Error no end " found.
			}
		}
		if (param2 == NULL)
			param2 = strchr(param1, ' ');

		if (param2 != NULL) {
			paramCnt = 2;
			*param2 = '\0'; //replace space with end of string, end of the cmdName
			param2++;       //location of the first char of param2
		}
	}

	//loop trough cmdList first check paramCnt
	for (uint8_t i = 0; i < cmdCnt; i++) {
		//check param count
		if (cmdList[i].paramCnt == paramCnt) {
			//check if command name matches
			if (strcmp(cmdList[i].name, cmdName) == 0) {
				uint8_t errorCode = 0; //callback return error code
				//run callback
				if (paramCnt == 0)
					errorCode =
							((uint8_t (*)(char *response)) cmdList[i].callback)(
									response);
				else if (paramCnt == 1)
					errorCode =
							((uint8_t (*)(char *response, char *p1)) cmdList[i].callback)(
									response, param1);
				else if (paramCnt == 2)
					errorCode = ((uint8_t (*)(char *response, char *p1,
							char *p2)) cmdList[i].callback)(response, param1,
							param2);

				printf("ACK|ERR=%i|RESP=\"%s\"\n", errorCode, response);
				return 0; //break the loop, return function
			}
		}
	}
	//cmd not found
	printf("NACK| cmd '%s' unkown\n", cmdString);
	return 2;
}

/**
 * @brief Displays all commands and their description
 * 
 * @return uint8_t 
 */
uint8_t cli_help()
{
    char funcName[30] = "";

    /* Print Header*/
    printf("|----------------------|-------------------------------------------------------------------------------------------------------------------------------|\n");
    printf("| Command              | Description                                                                                                                   |\n");
    printf("|----------------------|-------------------------------------------------------------------------------------------------------------------------------|\n");
    /* Print command and there description*/
    for (uint8_t i = 0; i < cmdCnt; i++)
    {
        if (cmdList[i].paramCnt == 0)
            printf("| %-20s ", cmdList[i].name);
        else if (cmdList[i].paramCnt == 1)
        {
            sprintf(funcName, "| %s $p1", cmdList[i].name);
            printf("%-20s   ", funcName);
        }
        else if (cmdList[i].paramCnt == 2)
        {
            sprintf(funcName, "| %s $p1 $p2", cmdList[i].name);
            printf("%-20s   ", funcName);
        }
        printf("| %-125s |\n", cmdList[i].cmdDesc);
    }
    /* Print footer */
    printf("|----------------------|-------------------------------------------------------------------------------------------------------------------------------|\n");

    return 0;
}
