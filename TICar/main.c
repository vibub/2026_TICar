#include "app_main.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();
    App_Init();

    while (1) {
        App_Loop();
    }
}
