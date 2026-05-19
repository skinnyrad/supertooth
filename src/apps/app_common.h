#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <stddef.h>

#include "receiver_session.h"

typedef enum
{
    APP_OUTPUT_MODE_FULL = 0,
    APP_OUTPUT_MODE_SUMMARY = 1,
    APP_OUTPUT_MODE_RSSI = 2
} app_output_mode_t;

typedef struct
{
    app_output_mode_t mode;
    const char *name;
} app_output_mode_option_t;

int app_parse_output_mode(const char *arg,
                          const app_output_mode_option_t *options,
                          size_t option_count,
                          app_output_mode_t *out_mode);
const app_output_mode_option_t *app_output_mode_option(app_output_mode_t mode,
                                                       const app_output_mode_option_t *options,
                                                       size_t option_count);
const char *app_output_mode_name(app_output_mode_t mode,
                                 const app_output_mode_option_t *options,
                                 size_t option_count);
void app_output_lock(void);
void app_output_unlock(void);
void app_install_sigint_handler(receiver_session_t **session_slot);

#endif
