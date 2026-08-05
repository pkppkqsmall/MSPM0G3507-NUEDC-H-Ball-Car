#include "ti_msp_dl_config.h"
#include "App/MapRun/map_run.h"

int main(void)
{
    SYSCFG_DL_init();
    MapRun_Init();

    while (1) {
        MapRun_RunStep();
    }
}
