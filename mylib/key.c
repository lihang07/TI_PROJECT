#include "ti_msp_dl_config.h"
#include "mylib/key.h"
#include "mylib/delay.h"


// 
uint8_t key_scan(Key_t *key)
{
    if(key->key_up && (DL_GPIO_readPins(key->port,key->pin) == 0))
    {
        delay_ms(10);

        if(DL_GPIO_readPins(key->port,key->pin) == 0)
        {
            key->key_up = 0;
            return 1;//按下
        }

    }
    else if (DL_GPIO_readPins(key->port,key->pin) != 0)
    {
        key->key_up = 1;       
    }

    return 0;
}

