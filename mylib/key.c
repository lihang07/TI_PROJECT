#include "ti_msp_dl_config.h"
#include "mylib/key.h"
#include "mylib/delay.h"

Key_t key1 = {KEY_KEY1_PORT,KEY_KEY1_PIN,1};
Key_t key2 = {KEY_KEY2_PORT,KEY_KEY2_PIN,1};
Key_t key3 = {KEY_KEY3_PORT,KEY_KEY3_PIN,1};
Key_t key4 = {KEY_KEY4_PORT,KEY_KEY4_PIN,1};

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


uint8_t key_read(void)
{
    static uint8_t last_status = 0;  // 记住上一次
    uint8_t status = 0;

    if(key_scan(&key1))       status = 1;
    else if(key_scan(&key2))  status = 2;
    else if(key_scan(&key3))  status = 3;
    else if(key_scan(&key4))  status = 4;

    if(status != 0)
    {
        last_status = status;  // 更新“记忆”
    }

    return last_status;        // 没按键就返回上一次
}
