#include "canlib.h"
#include "common.h"
#include "main.h"

#include "init.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

void can_callback_function(const can_msg_t *message, uint32_t) {
    switch (get_message_type(message)) {
        case MSG_RESET_CMD:
            if (check_board_need_reset(message)) {
                NVIC_SystemReset();
            }
            break;
        default:
            break;
    }
}

extern "C" w_status_t system_init(void) {
    can_init_stm(&hfdcan1, can_callback_function);

    return W_SUCCESS;
}
