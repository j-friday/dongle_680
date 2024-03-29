/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#ifndef __CLI_H__
#define __CLI_H__

#ifndef AOS_CLI_H
#define AOS_CLI_H
#include "uart.h"
#include "cli_api.h"

#define MAX_COMMANDS 64
#define INBUF_SIZE   128+512
#define OUTBUF_SIZE  128+512
#define HIS_SIZE     2


#define CONFIG_AOS_CLI

#define CLI_DEBUG 1




#ifndef FUNCPTR
typedef void (*FUNCPTR)(void);
#endif

/* Structure for registering CLI commands */
struct cli_command {
    const char *name;
    const char *help;

    void (*function)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
};

struct cli_st {
    int initialized;
    int echo_disabled;

    const struct cli_command *commands[MAX_COMMANDS];

    unsigned int num_commands;
    unsigned int bp; /* buffer pointer */

    char inbuf[INBUF_SIZE];
    char outbuf[OUTBUF_SIZE];

    int his_idx;
    int his_cur;
    char history[HIS_SIZE][INBUF_SIZE];
};

#define CLI_ARGS char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv
extern struct cli_st *cli;
extern int    cliexit;
extern const struct cli_command built_ins[BUILD_INS_NUM];

#ifdef CONFIG_AOS_CLI

#define cmd_printf(...)                                            \
    do {                                                           \
        if (xWriteBufferLen > 0) {                                 \
            snprintf(pcWriteBuffer, xWriteBufferLen, __VA_ARGS__); \
            xWriteBufferLen-= os_strlen(pcWriteBuffer);            \
            pcWriteBuffer+= os_strlen(pcWriteBuffer);              \
        }                                                          \
    } while(0)


/**
 * This function registers a command with the command-line interface.
 *
 * @param[in]  command  The structure to register one CLI command
 *
 * @return  0 on success, error code otherwise.
 */
int aos_cli_register_command(const struct cli_command *command);

/**
 * This function unregisters a command from the command-line interface.
 *
 * @param[in]  command  The structure to unregister one CLI command
 *
 * @return  0 on success,  error code otherwise.
 */
int aos_cli_unregister_command(const struct cli_command *command);

/**
 * Register a batch of CLI commands
 * Often, a module will want to register several commands.
 *
 * @param[in]  commands      Pointer to an array of commands.
 * @param[in]  num_commands  Number of commands in the array.
 *
 * @return  0 on success， error code otherwise.
 */
int aos_cli_register_commands(const struct cli_command *commands, int num_commands);

/**
 * Unregister a batch of CLI commands
 *
 * @param[in]  commands      Pointer to an array of commands.
 * @param[in]  num_commands  Number of commands in the array.
 *
 * @return  0 on success, error code otherwise.
 */
int aos_cli_unregister_commands(const struct cli_command *commands, int num_commands);


int csp_printf(const char *fmt,...);
/**
 * Print CLI msg
 *
 * @param[in]  buff  Pointer to a char * buffer.
 *
 * @return  0  on success, error code otherwise.
 */
#if defined BUILD_BIN || defined BUILD_KERNEL
 /* SINGLEBIN or KERNEL */
int aos_cli_printf(const char *buff, ...);
#else
 /* FRAMWORK or APP */
#define aos_cli_printf(fmt, ...) csp_printf("%s" fmt, aos_cli_get_tag(), ##__VA_ARGS__)

#endif

/**
 * CLI initial function
 *
 * @return  0 on success, error code otherwise
 */
int aos_cli_init(void);

/**
 * Stop the CLI thread and carry out the cleanup
 *
 * @return  0 on success, error code otherwise.
 *
 */
int aos_cli_stop(void);

/**
 * CLI get tag string
 *
 * @return cli tag storing buffer
 */
const char *aos_cli_get_tag(void);

#else /* CONFIG_AOS_CLI */

#define cmd_printf(...) do {} while(0)

static inline int aos_cli_register_command(const struct cli_command *command)
{
    return 0;
}

static inline int aos_cli_unregister_command(const struct cli_command *command)
{
    return 0;
}

static inline int aos_cli_register_commands(const struct cli_command *commands,
                                            int num_commands)
{
    return 0;
}

static inline int aos_cli_unregister_commands(const struct cli_command *commands,
                                              int num_commands)
{
    return 0;
}

#define aos_cli_printf csp_printf

int aos_cli_init(void)
{
    return 0;
}

int aos_cli_stop(void)
{
    return 0;
}
#endif


#endif /* AOS_CLI_H */

#endif
