#include "app_common.h"

#include <pthread.h>
#include <signal.h>
#include <string.h>

static receiver_session_t **g_session_slot = NULL;
static pthread_mutex_t g_output_mutex = PTHREAD_MUTEX_INITIALIZER;

static void app_handle_sigint(int sig)
{
    (void)sig;
    if (g_session_slot && *g_session_slot)
        receiver_session_request_stop(*g_session_slot);
}

int app_parse_output_mode(const char *arg,
                          const app_output_mode_option_t *options,
                          size_t option_count,
                          app_output_mode_t *out_mode)
{
    if (!arg || !options || option_count == 0u || !out_mode)
        return -1;

    for (size_t i = 0; i < option_count; i++)
    {
        if (strcmp(arg, options[i].name) == 0)
        {
            *out_mode = options[i].mode;
            return 0;
        }
    }

    return -1;
}

const app_output_mode_option_t *app_output_mode_option(app_output_mode_t mode,
                                                       const app_output_mode_option_t *options,
                                                       size_t option_count)
{
    if (!options || option_count == 0u)
        return NULL;

    for (size_t i = 0; i < option_count; i++)
    {
        if (options[i].mode == mode)
            return &options[i];
    }

    return &options[0];
}

const char *app_output_mode_name(app_output_mode_t mode,
                                 const app_output_mode_option_t *options,
                                 size_t option_count)
{
    const app_output_mode_option_t *option =
        app_output_mode_option(mode, options, option_count);
    return option ? option->name : "";
}

void app_output_lock(void)
{
    pthread_mutex_lock(&g_output_mutex);
}

void app_output_unlock(void)
{
    pthread_mutex_unlock(&g_output_mutex);
}

void app_install_sigint_handler(receiver_session_t **session_slot)
{
    g_session_slot = session_slot;
    signal(SIGINT, app_handle_sigint);
}
